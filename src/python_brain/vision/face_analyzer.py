import numpy as np
from typing import Any, Dict, List

class FaceAnalyzer:
    def __init__(self):
        pass

    def analyze(self, frame: np.ndarray, person_boxes: List[np.ndarray]) -> List[Dict[str, Any]]:
        faces = []
        h, w = frame.shape[:2]

        for box in person_boxes:
            x1, y1, x2, y2 = box
            
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w, x2)
            y2 = min(h, y2)

            if x2 <= x1 or y2 <= y1:
                continue

            cx = (x1 + x2) // 2
            cy = int(y1 + (y2 - y1) * 0.25)
            
            w_box = x2 - x1
            h_box = y2 - y1
            
            fx1 = max(0, cx - w_box // 4)
            fy1 = max(0, cy - h_box // 4)
            fx2 = min(w, cx + w_box // 4)
            fy2 = min(h, cy + w_box // 4)

            area = (fx2 - fx1) * (fy2 - fy1)

            faces.append({
                "bbox": [int(fx1), int(fy1), int(fx2), int(fy2)],
                "center": [int(cx), int(cy)],
                "area": int(area),
                "tracking_id": None
            })

        faces.sort(key=lambda f: f["area"], reverse=True)
        return faces
