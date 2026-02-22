import threading
from typing import Any, Callable, Dict, Set
from collections import defaultdict

class EventBus:
    def __init__(self) -> None:
        self._subscribers: Dict[str, Set[Callable[[Any], None]]] = defaultdict(set)
        self._lock = threading.RLock()

    def subscribe(self, event_type: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            self._subscribers[event_type].add(callback)

    def unsubscribe(self, event_type: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            if event_type in self._subscribers and callback in self._subscribers[event_type]:
                self._subscribers[event_type].remove(callback)
                if not self._subscribers[event_type]:
                    del self._subscribers[event_type]

    def publish(self, event_type: str, data: Any = None) -> None:
        with self._lock:
            if event_type not in self._subscribers:
                return
            callbacks = list(self._subscribers[event_type])
            
        for callback in callbacks:
            try:
                callback(data)
            except Exception:
                pass

    def clear(self) -> None:
        with self._lock:
            self._subscribers.clear()
