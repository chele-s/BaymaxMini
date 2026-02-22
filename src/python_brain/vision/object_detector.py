from typing import Any, Dict, List

class ObjectDetector:
    def __init__(self, allowed_classes: List[str] = None):
        self._allowed = allowed_classes or []

    def filter(self, objects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not self._allowed:
            return objects
            
        filtered = []
        for obj in objects:
            if obj.get("class", "") in self._allowed:
                filtered.append(obj)
        return filtered
