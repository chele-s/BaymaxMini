import json
import logging
import struct
import threading
import time
from typing import Any, Dict, Optional

import zmq

from ..core.event_bus import EventBus

class ZmqLink:
    def __init__(
        self,
        event_bus: EventBus,
        sub_endpoint: str = "tcp://127.0.0.1:5556",
        pub_endpoint: str = "tcp://127.0.0.1:5555",
        telem_topic: str = "TELEM",
        cmd_topic: str = "CMD",
        tick_rate_hz: float = 50.0
    ) -> None:
        self._bus = event_bus
        self._sub_endpoint = sub_endpoint
        self._pub_endpoint = pub_endpoint
        self._telem_topic = telem_topic
        self._cmd_topic = cmd_topic
        self._tick_period = 1.0 / tick_rate_hz
        
        self._context = zmq.Context()
        self._sub_socket: Optional[zmq.Socket] = None
        self._pub_socket: Optional[zmq.Socket] = None
        self._poller = zmq.Poller()
        
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        
        self._telem_struct_format = "=LLffffffffBffffBffBBBx"
        self._telem_struct_size = struct.calcsize(self._telem_struct_format)
        
        self._logger = logging.getLogger(__name__)

    def start(self) -> bool:
        if self._running:
            return True
            
        try:
            self._sub_socket = self._context.socket(zmq.SUB)
            self._sub_socket.setsockopt(zmq.CONFLATE, 1)
            self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, self._telem_topic)
            self._sub_socket.connect(self._sub_endpoint)
            
            self._pub_socket = self._context.socket(zmq.PUB)
            self._pub_socket.setsockopt(zmq.SNDHWM, 1)
            self._pub_socket.setsockopt(zmq.CONFLATE, 1)
            self._pub_socket.connect(self._pub_endpoint)
            
            self._poller.register(self._sub_socket, zmq.POLLIN)
            self._bus.subscribe("COMMAND_CPP_CMD", self._on_brain_command)
            
            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, name="ZmqRxThread", daemon=True)
            self._rx_thread.start()
            
            return True
        except zmq.ZMQError:
            self.stop()
            return False

    def stop(self) -> None:
        self._running = False
        
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
            
        self._bus.unsubscribe("COMMAND_CPP_CMD", self._on_brain_command)
        
        if self._sub_socket:
            self._poller.unregister(self._sub_socket)
            self._sub_socket.close(linger=0)
            self._sub_socket = None
            
        if self._pub_socket:
            self._pub_socket.close(linger=0)
            self._pub_socket = None

    def _on_brain_command(self, data: Any) -> None:
        if not self._running or not self._pub_socket:
            return
            
        try:
            if not isinstance(data, dict):
                return
                
            payload = json.dumps(data).encode('utf-8')
            
            self._pub_socket.send_multipart([
                self._cmd_topic.encode('utf-8'),
                payload
            ], flags=zmq.NOBLOCK)
            
        except (TypeError, ValueError, zmq.ZMQError):
            pass

    def _parse_telemetry(self, raw_data: bytes) -> Optional[Dict[str, Any]]:
        if len(raw_data) != self._telem_struct_size:
            return None
            
        try:
            unpacked = struct.unpack(self._telem_struct_format, raw_data)
            
            return {
                "sequence": unpacked[0],
                "timestamp_us": unpacked[1],
                "distance_mm": unpacked[2],
                "heart_rate_bpm": unpacked[3],
                "spo2_percent": unpacked[4],
                "skin_temp_c": unpacked[5],
                "ambient_temp_c": unpacked[6],
                "bus_voltage_v": unpacked[7],
                "current_ma": unpacked[8],
                "power_mw": unpacked[9],
                "proximity_valid": bool(unpacked[10] & 0x01),
                "vitals_valid": bool(unpacked[10] & 0x02),
                "power_valid": bool(unpacked[10] & 0x04),
                "face_valid": bool(unpacked[10] & 0x08),
                "battery_pct": unpacked[11],
                "eyelid_openness": unpacked[12],
                "gaze_x": unpacked[13],
                "gaze_y": unpacked[14],
                "expression": unpacked[15],
                "state": unpacked[16],
                "alert": unpacked[17]
            }
        except struct.error:
            return None

    def _rx_loop(self) -> None:
        next_tick = time.perf_counter()
        
        while self._running:
            now = time.perf_counter()
            timeout_ms = max(0, int((next_tick - now) * 1000))
            
            try:
                socks = dict(self._poller.poll(timeout_ms))
                
                if self._sub_socket and socks.get(self._sub_socket) == zmq.POLLIN:
                    frames = self._sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                    if len(frames) >= 2:
                        topic = frames[0].decode('utf-8', errors='ignore')
                        if topic == self._telem_topic:
                            telem_data = self._parse_telemetry(frames[1])
                            if telem_data:
                                self._process_telemetry(telem_data)
                
                now = time.perf_counter()
                if now >= next_tick:
                    self._bus.publish("TICK", {"timestamp": now})
                    next_tick += self._tick_period
                    
                    if now > next_tick + self._tick_period:
                        next_tick = now + self._tick_period
                        
            except zmq.ZMQError as e:
                if e.errno == zmq.ETERM:
                    break
                time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def _process_telemetry(self, telem: Dict[str, Any]) -> None:
        self._bus.publish("TELEMETRY_UPDATE", telem)
        
        if telem.get("vitals_valid"):
            skin_temp = telem.get("skin_temp_c", 0.0)
            if skin_temp > 37.5:
                self._bus.publish("HIGH_TEMPERATURE", {
                    "temperature": skin_temp,
                    "timestamp": telem["timestamp_us"]
                })
                
        if telem.get("power_valid"):
            battery = telem.get("battery_pct", 100.0)
            if battery < 15.0:
                self._bus.publish("BATTERY_CRITICAL", {
                    "level": battery,
                    "timestamp": telem["timestamp_us"]
                })
