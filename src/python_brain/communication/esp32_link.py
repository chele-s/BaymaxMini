import time
import threading
import logging
from typing import Optional, Dict, Any, Callable

try:
    import baymax_native
    NATIVE_AVAILABLE = True
except ImportError:
    NATIVE_AVAILABLE = False

class ESP32Link:
    def __init__(self, port_name: str = "COM3", baud_rate: int = 921600) -> None:
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.logger = logging.getLogger("ESP32Link")
        
        self._native_link: Optional[Any] = None
        self._connected = False
        self._running = False
        self._poll_thread: Optional[threading.Thread] = None
        
        self._telemetry_callbacks: list = []
        self._last_telemetry: Optional[Dict[str, Any]] = None

    def connect(self) -> bool:
        if not NATIVE_AVAILABLE:
            self.logger.error("baymax_native module not available")
            return False
            
        try:
            self._native_link = baymax_native.SerialLink()
            if self._native_link.open_port(self.port_name, self.baud_rate):
                self._connected = True
                self._running = True
                self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._poll_thread.start()
                self.logger.info(f"Connected to ESP32 on {self.port_name} at {self.baud_rate}")
                return True
            else:
                self.logger.error(f"Failed to open port {self.port_name}")
                return False
        except Exception as e:
            self.logger.error(f"Error connecting to ESP32: {e}")
            return False

    def disconnect(self) -> None:
        self._running = False
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
            
        if self._native_link and self._connected:
            try:
                self._native_link.close_port()
            except Exception:
                pass
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and (self._native_link.is_open() if self._native_link else False)

    def register_telemetry_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        if callback not in self._telemetry_callbacks:
            self._telemetry_callbacks.append(callback)

    def get_latest_telemetry(self) -> Optional[Dict[str, Any]]:
        return self._last_telemetry

    def set_expression(self, expression_name: str, transition_sec: float = 0.3) -> bool:
        if not self.is_connected() or not NATIVE_AVAILABLE:
            return False
        
        expr_map = {
            "NEUTRAL": baymax_native.ExpressionType.NEUTRAL,
            "HAPPY": baymax_native.ExpressionType.HAPPY,
            "SAD": baymax_native.ExpressionType.SAD,
            "SURPRISED": baymax_native.ExpressionType.SURPRISED,
            "ANGRY": baymax_native.ExpressionType.ANGRY,
            "SLEEPY": baymax_native.ExpressionType.SLEEPY,
            "CONCERNED": baymax_native.ExpressionType.CONCERNED,
            "CURIOUS": baymax_native.ExpressionType.CURIOUS,
            "LOVE": baymax_native.ExpressionType.LOVE,
            "THINKING": baymax_native.ExpressionType.THINKING,
        }
        
        expr = expr_map.get(expression_name.upper(), baymax_native.ExpressionType.NEUTRAL)
        return self._native_link.send_expression(expr, transition_sec)

    def set_eyelid(self, openness: float, duration_sec: float = 0.2) -> bool:
        if not self.is_connected():
            return False
        return self._native_link.send_eyelid(openness, duration_sec)

    def set_gaze(self, x: float, y: float, speed: float = 1.0) -> bool:
        if not self.is_connected():
            return False
        return self._native_link.send_gaze(x, y, speed)

    def set_breath(self, intensity: float) -> bool:
        if not self.is_connected():
            return False
        return self._native_link.send_breath(intensity)

    def emergency_stop(self) -> bool:
        if not self.is_connected():
            return False
        return self._native_link.send_emergency_stop()

    def shutdown(self) -> bool:
        if not self.is_connected():
            return False
        return self._native_link.send_shutdown()

    def _poll_loop(self) -> None:
        while self._running:
            if self._native_link and self._native_link.is_open():
                frame = self._native_link.get_latest_telemetry()
                if frame:
                    data = {
                        "sequence": frame.sequence,
                        "timestamp_us": frame.timestamp_us,
                        "distance_mm": frame.distance_mm,
                        "heart_rate_bpm": frame.heart_rate_bpm,
                        "spo2_percent": frame.spo2_percent,
                        "skin_temp_c": frame.skin_temp_c,
                        "ambient_temp_c": frame.ambient_temp_c,
                        "bus_voltage_v": frame.bus_voltage_v,
                        "current_ma": frame.current_ma,
                        "power_mw": frame.power_mw,
                        "battery_pct": frame.battery_pct,
                        "eyelid_openness": frame.eyelid_openness,
                        "gaze_x": frame.gaze_x,
                        "gaze_y": frame.gaze_y,
                        "breath_level": frame.breath_level,
                        "state": frame.state,
                        "expression": frame.expression,
                        "alert": frame.alert
                    }
                    self._last_telemetry = data
                    for cb in self._telemetry_callbacks:
                        try:
                            cb(data)
                        except Exception as e:
                            self.logger.error(f"Callback error: {e}")
            time.sleep(0.01)
