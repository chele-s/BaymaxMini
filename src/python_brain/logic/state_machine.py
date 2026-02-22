import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Type

from ..core.event_bus import EventBus
from ..utils.logger import get_logger

class BaseState(ABC):
    def __init__(self, machine: 'StateMachine') -> None:
        self.machine = machine
        self.logger = get_logger()

    @abstractmethod
    def on_enter(self) -> None:
        pass

    @abstractmethod
    def on_exit(self) -> None:
        pass

    @abstractmethod
    def handle_event(self, event_type: str, data: Any) -> Optional[Type['BaseState']]:
        pass

class BootingState(BaseState):
    def on_enter(self) -> None:
        self.logger.info("Entering BOOTING state.")
        self.machine.bus.publish("COMMAND_CPP_CMD", {"cmd": "wake", "data": {"transition": 2.0}})
        
    def on_exit(self) -> None:
        self.logger.info("Exiting BOOTING state.")
        
    def handle_event(self, event_type: str, data: Any) -> Optional[Type[BaseState]]:
        if event_type == "SYSTEM_READY":
            return IdleState
        return None

class IdleState(BaseState):
    def on_enter(self) -> None:
        self.logger.info("Entering IDLE state.")
        self.machine.bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "neutral", "transition": 1.0}})
        
    def on_exit(self) -> None:
        self.logger.info("Exiting IDLE state.")
        
    def handle_event(self, event_type: str, data: Any) -> Optional[Type[BaseState]]:
        if event_type == "BATTERY_CRITICAL":
            return EmergencyShutdownState
        if event_type == "FACE_DETECTED" or event_type == "WAKEWORD_DETECTED":
            return InteractingState
        if event_type == "HIGH_TEMPERATURE":
            return MedicalScanState
        return None

class InteractingState(BaseState):
    def on_enter(self) -> None:
        self.logger.info("Entering INTERACTING state.")
        self.machine.bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "curious", "transition": 0.5}})
        self.machine.bus.publish("COMMAND_SPEAK", {"text": "Hello, I am Baymax, your personal healthcare companion."})
        self._interaction_start = time.time()
        
    def on_exit(self) -> None:
        self.logger.info("Exiting INTERACTING state.")
        
    def handle_event(self, event_type: str, data: Any) -> Optional[Type[BaseState]]:
        if event_type == "BATTERY_CRITICAL":
            return EmergencyShutdownState
        if event_type == "HIGH_TEMPERATURE":
            return MedicalScanState
            
        if event_type == "TICK":
            if time.time() - self._interaction_start > 30.0:
                return IdleState
                
        if event_type == "WAKEWORD_DETECTED":
            self._interaction_start = time.time()
            self.machine.bus.publish("COMMAND_SPEAK", {"text": "I am listening."})
            
        return None

class MedicalScanState(BaseState):
    def on_enter(self) -> None:
        self.logger.info("Entering MEDICAL_SCAN state.")
        self.machine.bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "concerned", "transition": 0.5}})
        self.machine.bus.publish("COMMAND_SPEAK", {"text": "I have detected an anomaly in your vitals. Initiating a scan."})
        
    def on_exit(self) -> None:
        self.logger.info("Exiting MEDICAL_SCAN state.")
        self.machine.bus.publish("COMMAND_SPEAK", {"text": "Scan complete."})
        
    def handle_event(self, event_type: str, data: Any) -> Optional[Type[BaseState]]:
        if event_type == "BATTERY_CRITICAL":
            return EmergencyShutdownState
        if event_type == "SCAN_COMPLETE":
            return IdleState
        return None

class EmergencyShutdownState(BaseState):
    def on_enter(self) -> None:
        self.logger.error("Entering EMERGENCY_SHUTDOWN state.")
        self.machine.bus.publish("INTERRUPT_SPEECH", {})
        self.machine.bus.publish("COMMAND_SPEAK", {"text": "Battery critical. Shutting down to prevent data loss."})
        self.machine.bus.publish("COMMAND_CPP_CMD", {"cmd": "sleep", "data": {"transition": 3.0}})
        
    def on_exit(self) -> None:
        pass
        
    def handle_event(self, event_type: str, data: Any) -> Optional[Type[BaseState]]:
        if event_type == "POWER_RESTORED":
            return IdleState
        return None

class StateMachine:
    def __init__(self, event_bus: EventBus) -> None:
        self.bus = event_bus
        self.logger = get_logger()
        self._lock = threading.RLock()
        
        self._current_state: BaseState = BootingState(self)
        self._memory: Dict[str, Any] = {}
        
        self.bus.subscribe("*", self._on_any_event)
        
    def start(self) -> None:
        with self._lock:
            self._current_state.on_enter()
            
    def _transition_to(self, new_state_cls: Type[BaseState]) -> None:
        with self._lock:
            self.logger.info(f"FSM Transition: {self._current_state.__class__.__name__} -> {new_state_cls.__name__}")
            self._current_state.on_exit()
            self._current_state = new_state_cls(self)
            self._current_state.on_enter()
            self.bus.publish("STATE_CHANGED", {"new_state": new_state_cls.__name__})
            
    def _on_any_event(self, event_type: str, data: Any) -> None:
        with self._lock:
            next_state_cls = self._current_state.handle_event(event_type, data)
            if next_state_cls:
                self._transition_to(next_state_cls)
                
    def get_memory(self, key: str, default: Any = None) -> Any:
        with self._lock:
            return self._memory.get(key, default)
            
    def set_memory(self, key: str, value: Any) -> None:
        with self._lock:
            self._memory[key] = value
