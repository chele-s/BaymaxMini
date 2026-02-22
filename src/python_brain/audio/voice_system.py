import json
import logging
import queue
import re
import subprocess
import tempfile
import threading
import time
from typing import Any, Dict, List, Optional, Set

import numpy as np

try:
    import pyaudio
except ImportError:
    pyaudio = None

try:
    import webrtcvad
except ImportError:
    webrtcvad = None

try:
    from vosk import Model as VoskModel, KaldiRecognizer
except ImportError:
    VoskModel = None
    KaldiRecognizer = None

from ..core.event_bus import EventBus

class VoiceSystem:
    def __init__(
        self,
        event_bus: EventBus,
        vosk_model_path: str = "models/vosk-model-small-en-us-0.15",
        piper_model_path: str = "models/piper_voices/en_US-lessac-high.onnx",
        piper_exec_path: str = "piper",
        wakewords: Optional[List[str]] = None,
        input_device_index: Optional[int] = None,
        output_device_index: Optional[int] = None,
        sample_rate: int = 16000,
        chunk_size: int = 480
    ) -> None:
        self._bus = event_bus
        self._vosk_model_path = vosk_model_path
        self._piper_model_path = piper_model_path
        self._piper_exec_path = piper_exec_path
        
        self._wakewords: Set[str] = set(wd.lower() for wd in (wakewords or ["baymax", "help", "pain", "doctor"]))
        
        self._in_idx = input_device_index
        self._out_idx = output_device_index
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size
        
        self._audio: Optional[pyaudio.PyAudio] = None
        self._in_stream: Optional[pyaudio.Stream] = None
        self._out_stream: Optional[pyaudio.Stream] = None
        
        self._vad = webrtcvad.Vad(3) if webrtcvad else None
        self._vosk_model: Optional[VoskModel] = None
        self._recognizer: Optional[KaldiRecognizer] = None
        
        self._running = False
        self._listen_thread: Optional[threading.Thread] = None
        self._tts_thread: Optional[threading.Thread] = None
        
        self._speech_queue: queue.Queue[bytes] = queue.Queue(maxsize=100)
        self._tts_queue: queue.Queue[str] = queue.Queue(maxsize=20)
        
        self._logger = logging.getLogger(__name__)

    def _init_audio(self) -> bool:
        if not pyaudio or not webrtcvad or not VoskModel:
            return False
            
        try:
            self._vosk_model = VoskModel(self._vosk_model_path)
            self._recognizer = KaldiRecognizer(self._vosk_model, self._sample_rate)
        except Exception:
            return False

        try:
            self._audio = pyaudio.PyAudio()
            self._in_stream = self._audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self._sample_rate,
                input=True,
                frames_per_buffer=self._chunk_size,
                input_device_index=self._in_idx
            )
            
            self._out_stream = self._audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=22050,
                output=True,
                frames_per_buffer=4096,
                output_device_index=self._out_idx
            )
            return True
        except Exception:
            self._cleanup_audio()
            return False

    def _cleanup_audio(self) -> None:
        if self._in_stream:
            self._in_stream.stop_stream()
            self._in_stream.close()
            self._in_stream = None
            
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
        
        self._running = True
        
        self._listen_thread = threading.Thread(target=self._listen_loop, name="VoiceListenThread", daemon=True)
        self._listen_thread.start()
        
        self._stt_thread = threading.Thread(target=self._stt_loop, name="VoiceSTTThread", daemon=True)
        self._stt_thread.start()
        
        self._tts_thread = threading.Thread(target=self._tts_loop, name="VoiceTTSThread", daemon=True)
        self._tts_thread.start()
        
        return True

    def stop(self) -> None:
        self._running = False
        
        self._bus.unsubscribe("COMMAND_SPEAK", self._on_command_speak)
        
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=2.0)
            
        if hasattr(self, '_stt_thread') and self._stt_thread.is_alive():
            self._stt_thread.join(timeout=2.0)
            
        if self._tts_thread and self._tts_thread.is_alive():
            self._tts_thread.join(timeout=2.0)
            
        self._cleanup_audio()

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

    def _listen_loop(self) -> None:
        if not self._in_stream or not self._vad:
            return
            
        consecutive_silence = 0
        is_speaking = False
        frame_duration_ms = 1000 * self._chunk_size // self._sample_rate
        
        while self._running:
            try:
                pcm_data = self._in_stream.read(self._chunk_size, exception_on_overflow=False)
                is_speech = self._vad.is_speech(pcm_data, self._sample_rate)
                
                if is_speech:
                    consecutive_silence = 0
                    is_speaking = True
                    try:
                        self._speech_queue.put_nowait(pcm_data)
                    except queue.Full:
                        pass
                else:
                    consecutive_silence += frame_duration_ms
                    if is_speaking:
                        try:
                            self._speech_queue.put_nowait(pcm_data)
                        except queue.Full:
                            pass
                            
                        if consecutive_silence > 1000:
                            is_speaking = False
                            try:
                                self._speech_queue.put_nowait(b'SILENCE_MARKER')
                            except queue.Full:
                                pass
                                
            except Exception:
                time.sleep(0.01)

    def _clean_text(self, text: str) -> str:
        text = text.lower().strip()
        text = re.sub(r'[^\w\s]', '', text)
        return text

    def _stt_loop(self) -> None:
        if not self._recognizer:
            return
            
        while self._running:
            try:
                data = self._speech_queue.get(timeout=0.1)
                
                if data == b'SILENCE_MARKER':
                    res = self._recognizer.FinalResult()
                    self._process_recognized_text(res)
                    continue
                    
                if self._recognizer.AcceptWaveform(data):
                    res = self._recognizer.Result()
                    self._process_recognized_text(res)
                    
            except queue.Empty:
                continue
            except Exception:
                time.sleep(0.01)

    def _process_recognized_text(self, json_res: str) -> None:
        try:
            parsed = json.loads(json_res)
            text = parsed.get("text", "")
            if not text:
                return
                
            cleaned = self._clean_text(text)
            if not cleaned:
                return
                
            words = set(cleaned.split())
            if self._wakewords.intersection(words):
                self._bus.publish("WAKEWORD_DETECTED", {"text": cleaned, "timestamp": time.time()})
                self._bus.publish("COMMAND_CPP_CMD", {"cmd": "wake", "data": {"transition": 0.5}})
            else:
                self._bus.publish("VOICE_HEARD", {"text": cleaned, "timestamp": time.time()})
                
        except json.JSONDecodeError:
            pass

    def _tts_loop(self) -> None:
        if not self._out_stream:
            return
            
        while self._running:
            try:
                text = self._tts_queue.get(timeout=0.1)
                self._bus.publish("SPEECH_START", {"text": text, "timestamp": time.time()})
                self._synthesize_and_play(text)
                self._bus.publish("SPEECH_END", {"text": text, "timestamp": time.time()})
            except queue.Empty:
                continue
            except Exception:
                time.sleep(0.01)

    def _synthesize_and_play(self, text: str) -> None:
        try:
            with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as tmp_file:
                tmp_path = tmp_file.name

            cmd = [
                self._piper_exec_path,
                "--model", self._piper_model_path,
                "--output_raw",
                "--output_file", tmp_path
            ]
            
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            if process.stdin:
                process.stdin.write(text.encode('utf-8'))
                process.stdin.close()
                
            process.wait(timeout=10.0)
            
            if process.returncode == 0 and self._out_stream:
                import os
                if os.path.exists(tmp_path):
                    with open(tmp_path, "rb") as f:
                        while self._running:
                            chunk = f.read(4096)
                            if not chunk:
                                break
                            try:
                                self._out_stream.write(chunk)
                            except Exception:
                                break
                            
            try:
                import os
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except OSError:
                pass
                
        except Exception:
            pass
