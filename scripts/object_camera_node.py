#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json
import os

# Load product info JSON (fix this path accordingly)
PRODUCT_INFO_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"
with open(PRODUCT_INFO_PATH, "r") as f:
    PRODUCT_INFO = json.load(f)

# Load YOLOv8 model
model = YOLO("yolov8n.pt")

bridge = CvBridge()
pub = None  # global publisher

def image_callback(msg):
    global pub
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    results = model(frame, verbose=False)[0]

    for box in results.boxes:
        xyxy = box.xyxy[0].cpu().numpy().astype(int)
        cls_id = int(box.cls[0])
        label = model.names[cls_id].lower()
        conf = box.conf[0].cpu().numpy()

        info = PRODUCT_INFO.get(label, {})
        price = info.get('price', 'N/A')
        location = info.get('location', 'unknown')

        # Log detection info
        if info:
            rospy.loginfo(f"[DETECTED] {label.upper()}: RM{price}, {info.get('type', 'unknown')} in {location}")
            pub.publish(json.dumps({label: info}))
        else:
            rospy.logwarn(f"[NO INFO] Detected '{label}' but not in database.")

        # Draw bounding box and text on frame
        display_text = f"{label} {conf:.2f} RM{price}, {location}"
        cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,255,0), 2)
        cv2.putText(frame, display_text, (xyxy[0], xyxy[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    # Show the frame with detection info
    cv2.imshow("YOLOv8 Detection", frame)
    cv2.waitKey(1)  # Needed to refresh the OpenCV window

def main():
    global pub
    rospy.init_node("object_camera_node", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    pub = rospy.Publisher("/object_info", String, queue_size=10)

    rospy.loginfo("🟢 YOLOv8 Vision Node Started")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
