import logging
import multiprocessing as mp
import threading
import time

from config.settings import CONFIG
from .core.event_bus import EventBus
from .utils.logger import SystemLogger, get_logger
from .communication.zmq_link import ZmqLink
from .medical.pharmacist import Pharmacist
from .medical.patient_history import PatientHistory
from .audio.stt_engine import STTEngine
from .audio.tts_engine import TTSEngine
from .audio.sound_fx import SoundFX
from .audio.audio_alerts import AudioAlerts
from .audio.voice_system import VoiceSystem
from .logic.state_machine import StateMachine
from .logic.scheduler import GlobalScheduler
from .vision.camera_stream import CameraStream
from .vision.camera import CameraController
from .vision.tpu_pipeline import VisionModule

class BaymaxBrain:
    def __init__(self) -> None:
        SystemLogger()
        self.logger = get_logger()
        self.bus = EventBus()
        
        self.zmq = ZmqLink(
            self.bus,
            sub_endpoint=f"tcp://{CONFIG.get('network.host')}:{CONFIG.get('network.zmq_sub_port')}",
            pub_endpoint=f"tcp://{CONFIG.get('network.host')}:{CONFIG.get('network.zmq_pub_port')}",
            tick_rate_hz=CONFIG.get('network.tick_rate_hz', 50.0)
        )
        self.pharmacist = Pharmacist(self.bus)
        self.history = PatientHistory(self.bus)
        
        self.stt = STTEngine(self.bus, model_path=CONFIG.get('audio.stt_model_path'))
        self.tts = TTSEngine(
            self.bus, 
            model_path=CONFIG.get('audio.tts_model_path'),
            piper_exec=CONFIG.get('audio.tts_exec_path')
        )
        self.sfx = SoundFX(self.bus)
        self.alerts = AudioAlerts(self.bus)
        self.voice = VoiceSystem(self.bus, self.stt, self.tts)
        
        self.fsm = StateMachine(self.bus)
        self.scheduler = GlobalScheduler(self.bus)
        
        self.cam_stream = CameraStream(width=640, height=480, fps=30)
        self.camera_ctrl = CameraController(
            self.cam_stream,
            camera_index=CONFIG.get('vision.camera_index', 0)
        )
        self.vision = VisionModule(
            self.bus,
            model_path=CONFIG.get('vision.model_path'),
            labels=CONFIG.get('vision.labels', {0: "person", 1: "face"})
        )
        
        self._running = False
        self._vision_thread = None

    def _vision_worker(self) -> None:
        if not self.cam_stream.attach_shared_memory():
            self.logger.error("Vision worker failed to map DMA Shared Memory.")
            return
            
        while self._running:
            frame = self.cam_stream.read_frame()
            if frame is not None:
                try:
                    self.vision.process_frame(frame)
                except Exception:
                    pass

    def start(self) -> None:
        self.logger.info("Booting BaymaxMini Core Cortex...")
        self._running = True
        
        self.fsm.start()
        self.scheduler.start()
        self.history.start()
        self.pharmacist.start()
        
        self.sfx.start(preload_paths=CONFIG.get('foley.sounds', {}))
        self.alerts.start()
        self.stt.start()
        self.tts.start()
        
        self.zmq.start()
        
        self.camera_ctrl.start()
        time.sleep(1.5)
        
        self._vision_thread = threading.Thread(target=self._vision_worker, daemon=True)
        self._vision_thread.start()
        
        self.logger.info("All operational sub-systems fully linked. Standing by.")
        
        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            self.logger.info("Received SIGINT. Halting operations.")
        finally:
            self.stop()

    def stop(self) -> None:
        self._running = False
        self.logger.info("Initiating graceful teardown...")
        
        self.camera_ctrl.stop()
        time.sleep(0.5)
        self.cam_stream.cleanup(unlink=False)
        
        self.zmq.stop()
        self.tts.stop()
        self.stt.stop()
        self.alerts.stop()
        self.sfx.stop()
        
        self.pharmacist.stop()
        self.history.stop()
        self.scheduler.stop()

def main() -> None:
    brain = BaymaxBrain()
    brain.start()

if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    main()
