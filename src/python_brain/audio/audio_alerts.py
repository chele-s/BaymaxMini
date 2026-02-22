import logging
import math
import struct
import threading
import time
from typing import Any, Optional

try:
    import pyaudio
except ImportError:
    pyaudio = None

from ..core.event_bus import EventBus

class AudioAlerts:
    def __init__(
        self,
        event_bus: EventBus,
        output_device_index: Optional[int] = None,
        sample_rate: int = 44100
    ) -> None:
        self._bus = event_bus
        self._out_idx = output_device_index
        self._sample_rate = sample_rate
        
        self._audio: Optional[pyaudio.PyAudio] = None
        self._stream: Optional[pyaudio.Stream] = None
        
        self._running = False
        self._active_alert = False
        self._alert_thread: Optional[threading.Thread] = None
        
        self._logger = logging.getLogger(__name__)

    def _init_audio(self) -> bool:
        if not pyaudio:
            return False
            
        try:
            self._audio = pyaudio.PyAudio()
            self._stream = self._audio.open(
                format=pyaudio.paFloat32,
                channels=1,
                rate=self._sample_rate,
                output=True,
                output_device_index=self._out_idx
            )
            return True
        except Exception:
            self._cleanup_audio()
            return False

    def _cleanup_audio(self) -> None:
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
            
        if self._audio:
            self._audio.terminate()
            self._audio = None

    def start(self) -> bool:
        if self._running:
            return True
            
        if not self._init_audio():
            return False
            
        self._bus.subscribe("BATTERY_CRITICAL", self._on_critical_alert)
        self._bus.subscribe("HIGH_TEMPERATURE", self._on_critical_alert)
        self._bus.subscribe("SYSTEM_ERROR", self._on_critical_alert)
        
        self._running = True
        return True

    def stop(self) -> None:
        self._running = False
        self._active_alert = False
        
        self._bus.unsubscribe("BATTERY_CRITICAL", self._on_critical_alert)
        self._bus.unsubscribe("HIGH_TEMPERATURE", self._on_critical_alert)
        self._bus.unsubscribe("SYSTEM_ERROR", self._on_critical_alert)
        
        if self._alert_thread and self._alert_thread.is_alive():
            self._alert_thread.join(timeout=2.0)
            
        self._cleanup_audio()

    def _on_critical_alert(self, data: Any) -> None:
        if not self._running or self._active_alert:
            return
            
        self._active_alert = True
        self._alert_thread = threading.Thread(target=self._play_critical_alarm, daemon=True)
        self._alert_thread.start()

    def _generate_tone(self, frequency: float, duration: float, volume: float = 0.8) -> bytes:
        num_samples = int(self._sample_rate * duration)
        samples = []
        for i in range(num_samples):
            time_sec = float(i) / self._sample_rate
            val = volume * math.sin(2.0 * math.pi * frequency * time_sec)
            
            envelope = 1.0
            if i < self._sample_rate * 0.05:
                envelope = float(i) / (self._sample_rate * 0.05)
            elif i > num_samples - (self._sample_rate * 0.05):
                envelope = float(num_samples - i) / (self._sample_rate * 0.05)
                
            samples.append(val * envelope)
            
        return struct.pack(f"{len(samples)}f", *samples)

    def _play_critical_alarm(self) -> None:
        if not self._stream:
            self._active_alert = False
            return
            
        tone_high = self._generate_tone(880.0, 0.2, 0.9)
        tone_low = self._generate_tone(659.25, 0.2, 0.9)
        silence = self._generate_tone(0.0, 0.1, 0.0)
        
        try:
            for _ in range(5):
                if not self._running:
                    break
                    
                self._stream.write(tone_high)
                self._stream.write(silence)
                self._stream.write(tone_low)
                self._stream.write(silence)
                
                time.sleep(0.5)
                
        except Exception:
            pass
        finally:
            self._active_alert = False
