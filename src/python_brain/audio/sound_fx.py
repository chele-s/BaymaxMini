import logging
import threading
import wave
from typing import Any, Dict

try:
    import pyaudio
except ImportError:
    pyaudio = None

from ..core.event_bus import EventBus

class SoundFX:
    def __init__(self, event_bus: EventBus) -> None:
        self._bus = event_bus
        self._audio: pyaudio.PyAudio = None
        self._sounds_cache: Dict[str, Dict[str, Any]] = {}
        
        self._running = False
        self._playback_threads = []
        
        self._logger = logging.getLogger(__name__)

    def _init_audio(self) -> bool:
        if not pyaudio:
            return False
            
        try:
            self._audio = pyaudio.PyAudio()
            return True
        except Exception:
            return False

    def load_sound(self, name: str, filepath: str) -> bool:
        try:
            with wave.open(filepath, "rb") as wf:
                format_idx = self._audio.get_format_from_width(wf.getsampwidth())
                channels = wf.getnchannels()
                rate = wf.getframerate()
                data = wf.readframes(wf.getnframes())
                
                self._sounds_cache[name] = {
                    "format": format_idx,
                    "channels": channels,
                    "rate": rate,
                    "data": data
                }
            return True
        except Exception:
            return False

    def start(self, preload_paths: Dict[str, str] = None) -> bool:
        if self._running:
            return True
            
        if not self._init_audio():
            return False
            
        if preload_paths:
            for name, path in preload_paths.items():
                self.load_sound(name, path)
                
        self._bus.subscribe("PLAY_SOUND_FX", self._on_play_sound)
        self._bus.subscribe("WAKEWORD_DETECTED", lambda d: self.play("blip_confirm"))
        self._bus.subscribe("FACE_DETECTED", lambda d: self.play("servo_short"))
        
        self._running = True
        return True

    def stop(self) -> None:
        self._running = False
        self._bus.unsubscribe("PLAY_SOUND_FX", self._on_play_sound)
        
        for t in self._playback_threads:
            if t.is_alive():
                t.join(timeout=0.5)
                
        self._playback_threads.clear()
        
        if self._audio:
            self._audio.terminate()
            self._audio = None

    def play(self, sound_name: str) -> None:
        if not self._running or sound_name not in self._sounds_cache:
            return
            
        def _play_worker() -> None:
            sound = self._sounds_cache[sound_name]
            stream = None
            try:
                stream = self._audio.open(
                    format=sound["format"],
                    channels=sound["channels"],
                    rate=sound["rate"],
                    output=True
                )
                stream.write(sound["data"])
            except Exception:
                pass
            finally:
                if stream:
                    try:
                        stream.stop_stream()
                        stream.close()
                    except Exception:
                        pass
        
        t = threading.Thread(target=_play_worker, daemon=True)
        t.start()
        
        self._playback_threads = [thread for thread in self._playback_threads if thread.is_alive()]
        self._playback_threads.append(t)

    def _on_play_sound(self, data: Any) -> None:
        if isinstance(data, dict):
            name = data.get("name")
            if name:
                self.play(name)
