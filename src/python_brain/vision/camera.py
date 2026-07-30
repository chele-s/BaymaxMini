import logging
import multiprocessing as mp
import time
from typing import Any, Optional
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

from .camera_stream import CameraStream

class CameraController(mp.Process):
    def __init__(
        self,
        camera_stream: CameraStream,
        source: str = "http://192.168.4.1:81/stream",
        camera_index: int = 0
    ) -> None:
        super().__init__(name="CameraProcess")
        self._stream = camera_stream
        self._source = source
        self._cam_idx = camera_index
        self._running = mp.Event()
        self._logger = logging.getLogger("CameraProcess")
        self._cap: Optional[cv2.VideoCapture] = None
        self._picam: Optional[Any] = None

    def _init_camera(self) -> bool:
        if isinstance(self._source, str) and (self._source.startswith("http://") or self._source.startswith("https://") or self._source.startswith("rtsp://")):
            self._cap = cv2.VideoCapture(self._source)
            if self._cap.isOpened():
                return True

        if isinstance(self._cam_idx, int):
            self._cap = cv2.VideoCapture(self._cam_idx)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._stream.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._stream.height)
                self._cap.set(cv2.CAP_PROP_FPS, self._stream.fps)
                return True

        if Picamera2:
            try:
                self._picam = Picamera2(self._cam_idx)
                config = self._picam.create_video_configuration(
                    main={"size": (self._stream.width, self._stream.height), "format": "RGB888"},
                    controls={"FrameRate": float(self._stream.fps)},
                    buffer_count=4
                )
                self._picam.configure(config)
                self._picam.start()
                return True
            except Exception:
                pass

        return False

    def run(self) -> None:
        self._running.set()

        if not self._stream.create_shared_memory():
            self._running.clear()
            return

        if not self._init_camera():
            self._stream.cleanup(unlink=True)
            self._running.clear()
            return

        try:
            while self._running.is_set():
                if self._cap and self._cap.isOpened():
                    ret, frame = self._cap.read()
                    if ret and frame is not None:
                        if frame.shape[0] != self._stream.height or frame.shape[1] != self._stream.width:
                            frame = cv2.resize(frame, (self._stream.width, self._stream.height))
                        if len(frame.shape) == 3 and frame.shape[2] == 3:
                            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            self._stream.write_frame(frame_rgb)
                    else:
                        time.sleep(0.01)
                elif self._picam:
                    frame = self._picam.capture_array("main")
                    self._stream.write_frame(frame)
                else:
                    time.sleep(0.05)

        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        finally:
            self._cleanup()

    def stop(self) -> None:
        self._running.clear()
        if self.is_alive():
            self.join(timeout=2.0)
            if self.is_alive():
                self.terminate()

    def _cleanup(self) -> None:
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        if self._picam:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
            self._picam = None

        self._stream.cleanup(unlink=True)