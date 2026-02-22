import time
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
from pycoral.adapters import common
from pycoral.utils.edgetpu import make_interpreter

class EdgeTPUPipeline:
    def __init__(self, model_path: str, confidence_threshold: float = 0.5, iou_threshold: float = 0.45) -> None:
        self._interpreter = make_interpreter(model_path)
        self._interpreter.allocate_tensors()
        
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        
        self._input_shape = self._input_details[0]['shape']
        self._model_h = self._input_shape[1]
        self._model_w = self._input_shape[2]
        
        self._scale, self._zero_point = self._input_details[0]['quantization']
        self._out_scale, self._out_zero_point = self._output_details[0]['quantization']
        
        self._conf_threshold = confidence_threshold
        self._iou_threshold = iou_threshold

    def _nms(self, boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray) -> List[int]:
        if len(boxes) == 0:
            return []
            
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        
        order = scores.argsort()[::-1]
        keep = []
        
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            if order.size == 1:
                break
                
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            
            inds = np.where(iou <= self._iou_threshold)[0]
            order = order[inds + 1]
            
        return keep

    def _process_yolov8_output(self, output: np.ndarray, orig_w: int, orig_h: int) -> List[Dict[str, Any]]:
        output = output[0]
        if output.shape[0] > output.shape[1]:
            output = output.T
            
        boxes = output[:, :4]
        scores = output[:, 4:]
        
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1)
        
        mask = max_scores > self._conf_threshold
        boxes = boxes[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]
        
        if len(boxes) == 0:
            return []
            
        cx = boxes[:, 0]
        cy = boxes[:, 1]
        w = boxes[:, 2]
        h = boxes[:, 3]
        
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        
        scale_x = orig_w / self._model_w
        scale_y = orig_h / self._model_h
        
        x1 = np.clip(x1 * scale_x, 0, orig_w).astype(np.float32)
        y1 = np.clip(y1 * scale_y, 0, orig_h).astype(np.float32)
        x2 = np.clip(x2 * scale_x, 0, orig_w).astype(np.float32)
        y2 = np.clip(y2 * scale_y, 0, orig_h).astype(np.float32)
        
        scaled_boxes = np.stack([x1, y1, x2, y2], axis=1)
        keep_indices = self._nms(scaled_boxes, max_scores, class_ids)
        
        results = []
        for i in keep_indices:
            results.append({
                "class_id": int(class_ids[i]),
                "confidence": float(max_scores[i]),
                "coords": [
                    float(scaled_boxes[i, 0]),
                    float(scaled_boxes[i, 1]),
                    float(scaled_boxes[i, 2] - scaled_boxes[i, 0]),
                    float(scaled_boxes[i, 3] - scaled_boxes[i, 1])
                ]
            })
            
        return results

    def infer(self, frame: np.ndarray) -> Tuple[List[Dict[str, Any]], float]:
        orig_h, orig_w = frame.shape[:2]
        start_time = time.perf_counter()
        
        if frame.shape[0] != self._model_h or frame.shape[1] != self._model_w:
            resized_frame = cv2.resize(frame, (self._model_w, self._model_h), interpolation=cv2.INTER_LINEAR)
        else:
            resized_frame = frame
            
        if self._zero_point != 0 or self._scale != 1.0:
            quantized = np.clip(resized_frame / self._scale + self._zero_point, 0, 255).astype(np.uint8)
        else:
            quantized = resized_frame
            
        common.set_input(self._interpreter, quantized)
        self._interpreter.invoke()
        
        output_tensor = common.output_tensor(self._interpreter, 0)
        
        if self._out_scale != 0:
            output_tensor = (output_tensor.astype(np.float32) - self._out_zero_point) * self._out_scale
            
        detections = self._process_yolov8_output(output_tensor, orig_w, orig_h)
        inference_time = (time.perf_counter() - start_time) * 1000.0
        
        return detections, inference_time

class VisionModule:
    def __init__(self, event_bus: Any, model_path: str, labels: Dict[int, str]) -> None:
        self._bus = event_bus
        self._labels = labels
        self._pipeline = EdgeTPUPipeline(model_path)
        
    def process_frame(self, frame: np.ndarray) -> None:
        detections, inference_time = self._pipeline.infer(frame)
        
        formatted_results = []
        face_detected = False
        
        for det in detections:
            class_name = self._labels.get(det["class_id"], f"unknown_{det['class_id']}")
            result = {
                "object": class_name,
                "confidence": det["confidence"],
                "coords": det["coords"]
            }
            formatted_results.append(result)
            self._bus.publish("OBJECT_DETECTED", result)
            
            if class_name in ("person", "face"):
                face_detected = True
            
        if face_detected:
            self._bus.publish("FACE_DETECTED", {
                "timestamp": time.time(),
                "detections": formatted_results
            })
            
        self._bus.publish("VISION_TELEM", {
            "inference_time_ms": inference_time,
            "detections_count": len(detections)
        })
