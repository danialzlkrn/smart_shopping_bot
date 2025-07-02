#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import json

class VisionProcessor:
    def __init__(self, model_path='yolov8n.pt', product_json_path='products_data.json'):
        self.model = YOLO(model_path)
        self.product_info = self.load_product_info(product_json_path)

    def load_product_info(self, path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"[ERROR] Failed to load product info: {e}")
            return {}

    def detect_objects(self, frame):
        detections = []
        results = self.model(frame, verbose=False)[0]

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id].lower()

            info = self.product_info.get(label, None)
            detected_info = {
                "label": label,
                "price": info['price'] if info else "Unknown",
                "type": info['type'] if info else "Unknown",
                "location": info['location'] if info else "Unknown"
            }
            detections.append(detected_info)

        return detections

    def run_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[ERROR] Cannot open webcam")
            return

        print("[INFO] Starting camera... Press 'q' to quit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Failed to grab frame")
                break

            detections = self.detect_objects(frame)

            for box, detection in zip(self.model(frame, verbose=False)[0].boxes, detections):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                text = f"{detection['label'].upper()} | {detection['price']} | {detection['location']}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Display frame
            cv2.imshow("Vision Processor - Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    processor = VisionProcessor()
    processor.run_camera()