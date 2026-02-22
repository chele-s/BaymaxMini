import logging
import multiprocessing as mp
import time
from typing import Any, Optional

try:
    from picamera2 import Picamera2
    from libcamera import controls
except ImportError:
    Picamera2 = None
    controls = None

from .camera_stream import CameraStream


class CameraController(mp.Process):
    def __init__(
        self,
        camera_stream: CameraStream,
        camera_index: int = 0,
    ) -> None:
        super().__init__(name="CameraProcess")
        self._stream = camera_stream
        self._cam_idx = camera_index
        self._running = mp.Event()
        self._picam: Optional[Any] = None
        self._logger = logging.getLogger("CameraProcess")

    def _init_camera(self) -> bool:
        if not Picamera2:
            self._logger.error("picamera2 not available.")
            return False

        try:
            self._picam = Picamera2(self._cam_idx)

            config = self._picam.create_video_configuration(
                main={
                    "size": (self._stream.width, self._stream.height),
                    "format": "RGB888",
                },
                controls={
                    "FrameRate": float(self._stream.fps),
                },
                buffer_count=4,
            )
            self._picam.configure(config)
            return True

        except Exception as exc:
            self._logger.exception("Camera init failed: %s", exc)
            return False

    def run(self) -> None:
        self._running.set()

        if not self._stream.create_shared_memory():
            self._logger.error("Failed to create shared memory.")
            self._running.clear()
            return

        if not self._init_camera():
            self._stream.cleanup(unlink=True)
            self._running.clear()
            return

        try:
            self._picam.start()

            while self._running.is_set():
                frame = self._picam.capture_array("main")
                self._stream.write_frame(frame)

        except KeyboardInterrupt:
            pass
        except Exception as exc:
            self._logger.exception("Capture loop error: %s", exc)
        finally:
            self._cleanup()

    def stop(self) -> None:
        self._running.clear()
        if self.is_alive():
            self.join(timeout=2.0)
            if self.is_alive():
                self.terminate()

    def _cleanup(self) -> None:
        if self._picam:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
            self._picam = None

        self._stream.cleanup(unlink=True)