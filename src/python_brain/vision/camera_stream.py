import multiprocessing as mp
from logging import Logger
import time
from typing import Optional, Tuple, Any

import numpy as np

try:
    from multiprocessing.shared_memory import SharedMemory
except ImportError:
    SharedMemory = None

class CameraStream:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 60,
        shm_name: str = "baymax_camera_shm"
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.shm_name = shm_name
        self.frame_size = width * height * 3
        
        self._shm: Optional[SharedMemory] = None
        self._frame_buffer: Optional[np.ndarray] = None
        
        self._ready_event = mp.Event()
        self._lock = mp.Lock()
        
    def create_shared_memory(self) -> bool:
        if not SharedMemory:
            return False
            
        try:
            try:
                self._shm = SharedMemory(name=self.shm_name, create=True, size=self.frame_size)
            except FileExistsError:
                self._shm = SharedMemory(name=self.shm_name, create=False)
                
            self._frame_buffer = np.ndarray(
                (self.height, self.width, 3), 
                dtype=np.uint8, 
                buffer=self._shm.buf
            )
            return True
        except Exception:
            self.cleanup()
            return False
            
    def attach_shared_memory(self) -> bool:
        if not SharedMemory:
            return False
            
        try:
            self._shm = SharedMemory(name=self.shm_name, create=False)
            self._frame_buffer = np.ndarray(
                (self.height, self.width, 3), 
                dtype=np.uint8, 
                buffer=self._shm.buf
            )
            return True
        except Exception:
            return False

    def write_frame(self, frame_data: np.ndarray) -> None:
        if self._frame_buffer is not None:
            with self._lock:
                np.copyto(self._frame_buffer, frame_data)
            self._ready_event.set()

    def read_frame(self) -> Optional[np.ndarray]:
        if self._frame_buffer is not None:
            if self._ready_event.wait(timeout=1.0):
                self._ready_event.clear()
                with self._lock:
                    frame_copy = self._frame_buffer.copy()
                return frame_copy
        return None

    def cleanup(self, unlink: bool = False) -> None:
        if self._shm:
            try:
                self._shm.close()
                if unlink:
                    self._shm.unlink()
            except Exception:
                pass
            self._shm = None
        self._frame_buffer = None
