from __future__ import annotations

import asyncio
import logging
import multiprocessing as mp
import signal
import sys
import time
from pathlib import Path
from typing import Any

import uvloop

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT))

from core.logger import build_logger
from core.event_bus import EventBus
from core.brain import Brain
from communication.zmq_link import ZmqLink
from vision.camera_stream import CameraStream
from vision.object_detector import ObjectDetector
from vision.face_analyzer import FaceAnalyzer
from audio.stt_engine import SttEngine
from audio.tts_engine import TtsEngine
from audio.intent_parser import IntentParser
from medical.scheduler import MedicineScheduler
from utils.time_utils import mono_us


_SHUTDOWN_TIMEOUT_S: float = 5.0
_VISION_PROC_START_TIMEOUT_S: float = 10.0


def _vision_process_entry(
    ready_event: mp.Event,
    stop_event: mp.Event,
    frame_queue: mp.Queue,
    result_queue: mp.Queue,
    log_level: int,
) -> None:
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

    logger = build_logger("vision_proc", log_level)

    camera = CameraStream()
    detector = ObjectDetector()
    analyzer = FaceAnalyzer()

    try:
        camera.start()
        detector.load()
        analyzer.load()
        ready_event.set()
        logger.info("vision process ready")

        while not stop_event.is_set():
            frame = camera.capture_nowait()
            if frame is None:
                time.sleep(0.005)
                continue

            detections = detector.infer(frame)
            face_data = analyzer.analyze(frame)

            result_queue.put_nowait(
                {"ts_us": mono_us(), "detections": detections, "face": face_data}
            )

    except Exception as exc:
        logger.exception("vision process crashed: %s", exc)
    finally:
        analyzer.unload()
        detector.unload()
        camera.stop()
        logger.info("vision process exited")


class Orchestrator:
    def __init__(self) -> None:
        self._log: logging.Logger = build_logger("orchestrator")
        self._bus: EventBus = EventBus()
        self._brain: Brain = Brain(self._bus)
        self._zmq: ZmqLink = ZmqLink(self._bus)
        self._stt: SttEngine = SttEngine(self._bus)
        self._tts: TtsEngine = TtsEngine(self._bus)
        self._intent: IntentParser = IntentParser(self._bus)
        self._scheduler: MedicineScheduler = MedicineScheduler(self._bus)
        self._shutdown_flag: asyncio.Event = asyncio.Event()

        self._vision_stop: mp.Event = mp.Event()
        self._vision_ready: mp.Event = mp.Event()
        self._frame_queue: mp.Queue = mp.Queue(maxsize=2)
        self._result_queue: mp.Queue = mp.Queue(maxsize=8)
        self._vision_proc: mp.Process | None = None

    def _spawn_vision(self, log_level: int) -> None:
        self._vision_proc = mp.Process(
            target=_vision_process_entry,
            args=(
                self._vision_ready,
                self._vision_stop,
                self._frame_queue,
                self._result_queue,
                log_level,
            ),
            daemon=True,
            name="baymax-vision",
        )
        self._vision_proc.start()

    async def _await_vision_ready(self) -> bool:
        deadline = asyncio.get_event_loop().time() + _VISION_PROC_START_TIMEOUT_S
        while not self._vision_ready.is_set():
            if asyncio.get_event_loop().time() > deadline:
                return False
            await asyncio.sleep(0.1)
        return True

    async def _drain_vision_results(self) -> None:
        while not self._shutdown_flag.is_set():
            try:
                result: dict[str, Any] = self._result_queue.get_nowait()
                await self._bus.publish("vision.result", result)
            except Exception:
                await asyncio.sleep(0.02)

    def _install_signal_handlers(self, loop: asyncio.AbstractEventLoop) -> None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self._on_signal)

    def _on_signal(self) -> None:
        self._log.info("shutdown signal received")
        self._shutdown_flag.set()

    async def _teardown(self) -> None:
        self._log.info("teardown initiated")

        self._vision_stop.set()
        if self._vision_proc and self._vision_proc.is_alive():
            self._vision_proc.join(timeout=_SHUTDOWN_TIMEOUT_S)
            if self._vision_proc.is_alive():
                self._vision_proc.kill()

        await asyncio.gather(
            self._scheduler.stop(),
            self._stt.stop(),
            self._tts.stop(),
            self._zmq.stop(),
            self._brain.stop(),
            return_exceptions=True,
        )

        self._log.info("teardown complete")

    async def run(self) -> int:
        loop = asyncio.get_running_loop()
        self._install_signal_handlers(loop)

        log_level = self._log.level

        self._spawn_vision(log_level)

        vision_ok = await self._await_vision_ready()
        if not vision_ok:
            self._log.error("vision process failed to start within timeout")

        await asyncio.gather(
            self._brain.start(),
            self._zmq.start(),
            self._stt.start(),
            self._tts.start(),
            self._scheduler.start(),
        )

        self._log.info("baymax online | ts=%d", mono_us())

        drain_task = loop.create_task(self._drain_vision_results())

        await self._shutdown_flag.wait()

        drain_task.cancel()
        try:
            await drain_task
        except asyncio.CancelledError:
            pass

        await self._teardown()
        return 0


def main() -> None:
    mp.set_start_method("forkserver", force=True)
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

    orch = Orchestrator()
    exit_code = asyncio.run(orch.run())
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
