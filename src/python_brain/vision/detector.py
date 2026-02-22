import numpy as np
import logging
from typing import Any, Dict, List, Optional
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from .face_analyzer import FaceAnalyzer
from .object_detector import ObjectDetector

class Detector:
    def __init__(self, model_path: str = "yolov8n.pt", conf_thresh: float = 0.5):
        self._logger = logging.getLogger("Detector")
        self._conf = conf_thresh
        self._model: Optional[Any] = None
        self._face_mod = FaceAnalyzer()
        self._obj_mod = ObjectDetector()
        
        if YOLO:
            try:
                self._model = YOLO(model_path)
            except Exception as e:
                self._logger.error("Failed to load YOLO model: %s", e)
        else:
            self._logger.warning("ultralytics not installed. Detection disabled.")

    def process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        result_dict: Dict[str, Any] = {
            "faces": [],
            "objects": [],
            "raw_detections": 0
        }

        if self._model is None or frame is None or frame.size == 0:
            return result_dict

        try:
            results = self._model.predict(
                source=frame,
                conf=self._conf,
                verbose=False,
                device="cpu",
                half=True
            )
            
            if not results:
                return result_dict

            det = results[0]
            if det.boxes is None or len(det.boxes) == 0:
                return result_dict

            boxes = det.boxes.xyxy.cpu().numpy()
            confs = det.boxes.conf.cpu().numpy()
            cls_ids = det.boxes.cls.cpu().numpy()
            names = det.names

            result_dict["raw_detections"] = len(boxes)

            persons: List[np.ndarray] = []
            objects_data: List[Dict[str, Any]] = []

            for i in range(len(boxes)):
                c_id = int(cls_ids[i])
                c_name = names.get(c_id, "unknown")
                score = float(confs[i])
                box = boxes[i].astype(int)

                if c_name == "person":
                    persons.append(box)
                else:
                    objects_data.append({
                        "class": c_name,
                        "confidence": score,
                        "bbox": box.tolist()
                    })

            if persons:
                result_dict["faces"] = self._face_mod.analyze(frame, persons)
            
            if objects_data:
                result_dict["objects"] = self._obj_mod.filter(objects_data)

        except Exception as e:
            self._logger.error("Error during detection: %s", e)

        return result_dict
