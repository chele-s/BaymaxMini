import threading
import time
from enum import Enum, auto
from typing import Any, Dict

from .event_bus import EventBus

class BrainState(Enum):
    BOOTING = auto()
    IDLE = auto()
    OBSERVING = auto()
    MEDICAL_ASSIST = auto()
    EMERGENCY = auto()

class BrainFSM:
    def __init__(self, event_bus: EventBus) -> None:
        self._bus: EventBus = event_bus
        self._state: BrainState = BrainState.BOOTING
        self._lock = threading.RLock()
        self._memory: Dict[str, Any] = {
            "high_temperature": False,
            "patient_in_sight": False,
            "last_face_detected": 0.0,
            "last_temperature_alert": 0.0,
            "battery_critical": False
        }
        
        self._register_subscriptions()
        with self._lock:
            self._transition_to(BrainState.IDLE)

    def _register_subscriptions(self) -> None:
        self._bus.subscribe("HIGH_TEMPERATURE", self._on_high_temperature)
        self._bus.subscribe("PATIENT_IN_SIGHT", self._on_patient_in_sight)
        self._bus.subscribe("FACE_DETECTED", self._on_face_detected)
        self._bus.subscribe("BATTERY_CRITICAL", self._on_battery_critical)
        self._bus.subscribe("TICK", self._on_tick)

    def _on_high_temperature(self, data: Any) -> None:
        with self._lock:
            self._memory["high_temperature"] = True
            self._memory["last_temperature_alert"] = time.time()
            self._evaluate_state()

    def _on_patient_in_sight(self, data: Any) -> None:
        with self._lock:
            self._memory["patient_in_sight"] = True
            self._memory["last_face_detected"] = time.time()
            self._evaluate_state()

    def _on_face_detected(self, data: Any) -> None:
        with self._lock:
            self._memory["last_face_detected"] = time.time()
            self._memory["patient_in_sight"] = True
            self._evaluate_state()

    def _on_battery_critical(self, data: Any) -> None:
        with self._lock:
            self._memory["battery_critical"] = True
            self._evaluate_state()

    def _on_tick(self, data: Any) -> None:
        with self._lock:
            current_time = time.time()
            
            if self._memory["high_temperature"] and (current_time - self._memory["last_temperature_alert"] > 30.0):
                self._memory["high_temperature"] = False
                
            if self._memory["patient_in_sight"] and (current_time - self._memory["last_face_detected"] > 5.0):
                self._memory["patient_in_sight"] = False

            self._evaluate_state()

    def _evaluate_state(self) -> None:
        if self._memory["battery_critical"]:
            if self._state != BrainState.EMERGENCY:
                self._transition_to(BrainState.EMERGENCY)
            return

        if self._memory["high_temperature"] and self._memory["patient_in_sight"]:
            if self._state != BrainState.MEDICAL_ASSIST:
                self._transition_to(BrainState.MEDICAL_ASSIST)
        elif self._memory["patient_in_sight"]:
            if self._state not in (BrainState.OBSERVING, BrainState.MEDICAL_ASSIST):
                self._transition_to(BrainState.OBSERVING)
        else:
            if self._state != BrainState.IDLE:
                self._transition_to(BrainState.IDLE)

    def _transition_to(self, new_state: BrainState) -> None:
        if self._state == new_state:
            return

        old_state = self._state
        self._state = new_state
        
        self._bus.publish("STATE_CHANGED", {"old": old_state.name, "new": new_state.name})
        self._execute_state_entry_actions(new_state)

    def _execute_state_entry_actions(self, state: BrainState) -> None:
        if state == BrainState.MEDICAL_ASSIST:
            self._bus.publish("COMMAND_SPEAK", {"text": "Hello, I am Baymax. Your temperature is elevated. How varying is your pain on a scale of 1 to 10?"})
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "concerned", "transition": 0.5}})
        
        elif state == BrainState.OBSERVING:
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "curious", "transition": 0.3}})
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "wake", "data": {"transition": 1.0}})
        
        elif state == BrainState.IDLE:
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "neutral", "transition": 1.0}})
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "sleep", "data": {"transition": 2.0}})
            
        elif state == BrainState.EMERGENCY:
            self._bus.publish("COMMAND_SPEAK", {"text": "Battery level critical. Entering power saving mode."})
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "sleepy", "transition": 2.0}})
            self._bus.publish("COMMAND_CPP_CMD", {"cmd": "sleep", "data": {"transition": 1.0}})

    @property
    def current_state(self) -> BrainState:
        with self._lock:
            return self._state
