import logging
import queue
import subprocess
import tempfile
import threading
import time
import os
from typing import Any, Optional

try:
    import pyaudio
except ImportError:
    pyaudio = None

from ..core.event_bus import EventBus

class TTSEngine:
    def __init__(
        self,
        event_bus: EventBus,
        model_path: str = "models/piper_voices/en_US-lessac-high.onnx",
        exec_path: str = "piper",
        output_device_index: Optional[int] = None,
        sample_rate: int = 22050,
        chunk_size: int = 4096
    ) -> None:
        self._bus = event_bus
        self._model_path = model_path
        self._exec_path = exec_path
        self._out_idx = output_device_index
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size
        
        self._audio: Optional[pyaudio.PyAudio] = None
        self._out_stream: Optional[pyaudio.Stream] = None
        
        self._running = False
        self._interrupted = False
        self._current_process: Optional[subprocess.Popen] = None
        
        self._tts_queue: queue.Queue[str] = queue.Queue(maxsize=20)
        self._tts_thread: Optional[threading.Thread] = None
        
        self._logger = logging.getLogger(__name__)

    def _init_audio(self) -> bool:
        if not pyaudio:
            return False
            
        try:
            self._audio = pyaudio.PyAudio()
            self._out_stream = self._audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self._sample_rate,
                output=True,
                frames_per_buffer=self._chunk_size,
                output_device_index=self._out_idx
            )
            return True
        except Exception:
            self._cleanup_audio()
            return False

    def _cleanup_audio(self) -> None:
        if self._out_stream:
            self._out_stream.stop_stream()
            self._out_stream.close()
            self._out_stream = None
            
        if self._audio:
            self._audio.terminate()
            self._audio = None

    def start(self) -> bool:
        if self._running:
            return True
            
        if not self._init_audio():
            return False
            
        self._bus.subscribe("COMMAND_SPEAK", self._on_command_speak)
        self._bus.subscribe("INTERRUPT_SPEECH", self._on_interrupt)
        self._bus.subscribe("HIGH_TEMPERATURE", self._on_interrupt)
        self._bus.subscribe("BATTERY_CRITICAL", self._on_interrupt)
        
        self._running = True
        self._interrupted = False
        
        self._tts_thread = threading.Thread(target=self._tts_loop, name="TTSEngineThread", daemon=True)
        self._tts_thread.start()
        
        return True

    def stop(self) -> None:
        self._running = False
        self._interrupted = True
        
        self._bus.unsubscribe("COMMAND_SPEAK", self._on_command_speak)
        self._bus.unsubscribe("INTERRUPT_SPEECH", self._on_interrupt)
        self._bus.unsubscribe("HIGH_TEMPERATURE", self._on_interrupt)
        self._bus.unsubscribe("BATTERY_CRITICAL", self._on_interrupt)
        
        if self._current_process:
            try:
                self._current_process.terminate()
            except Exception:
                pass
                
        if self._tts_thread and self._tts_thread.is_alive():
            self._tts_thread.join(timeout=2.0)
            
        self._cleanup_audio()

    def _on_interrupt(self, data: Any) -> None:
        self._interrupted = True
        if self._current_process:
            try:
                self._current_process.terminate()
            except Exception:
                pass
        
        while not self._tts_queue.empty():
            try:
                self._tts_queue.get_nowait()
            except queue.Empty:
                break

    def _on_command_speak(self, data: Any) -> None:
        if not self._running:
            return
            
        if isinstance(data, dict):
            text = data.get("text", "")
        elif isinstance(data, str):
            text = data
        else:
            return
            
        if text.strip():
            try:
                self._tts_queue.put_nowait(text)
            except queue.Full:
                pass

    def _tts_loop(self) -> None:
        if not self._out_stream:
            return
            
        while self._running:
            try:
                text = self._tts_queue.get(timeout=0.1)
                self._interrupted = False
                self._bus.publish("SPEECH_START", {"text": text, "timestamp": time.time()})
                self._synthesize_and_play(text)
                
                if not self._interrupted:
                    self._bus.publish("SPEECH_END", {"text": text, "timestamp": time.time()})
                else:
                    self._bus.publish("SPEECH_INTERRUPTED", {"text": text, "timestamp": time.time()})
                    
            except queue.Empty:
                continue
            except Exception:
                time.sleep(0.01)

    def _synthesize_and_play(self, text: str) -> None:
        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as tmp_file:
                tmp_path = tmp_file.name

            cmd = [
                self._exec_path,
                "--model", self._model_path,
                "--output_raw",
                "--output_file", tmp_path
            ]
            
            self._current_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            if self._current_process.stdin:
                self._current_process.stdin.write(text.encode('utf-8'))
                self._current_process.stdin.close()
                
            while self._current_process.poll() is None:
                if self._interrupted or not self._running:
                    self._current_process.terminate()
                    break
                time.sleep(0.05)
            
            if not self._interrupted and self._running and self._out_stream:
                if os.path.exists(tmp_path):
                    with open(tmp_path, "rb") as f:
                        while self._running and not self._interrupted:
                            chunk = f.read(self._chunk_size)
                            if not chunk:
                                break
                            try:
                                self._out_stream.write(chunk)
                            except Exception:
                                break
                                
        except Exception:
            pass
        finally:
            self._current_process = None
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
