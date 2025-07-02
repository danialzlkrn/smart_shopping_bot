#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json
import os

PRODUCT_INFO_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"
with open(PRODUCT_INFO_PATH, "r") as f:
    PRODUCT_INFO = json.load(f)

model = YOLO("yolov8n.pt")
bridge = CvBridge()
pub = None

# Store last detection
last_detected_label = None
last_detected_info = None

def image_callback(msg):
    global pub, last_detected_label, last_detected_info
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

        if info:
            last_detected_label = label
            last_detected_info = info
            rospy.loginfo(f"[DETECTED] {label.upper()}: RM{price}, {info.get('type', 'unknown')} in {location}")
            pub.publish(json.dumps({label: info}))
        else:
            rospy.logwarn(f"[NO INFO] Detected '{label}' but not in database.")

        display_text = f"{label} {conf:.2f} RM{price}, {location}"
        cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,255,0), 2)
        cv2.putText(frame, display_text, (xyxy[0], xyxy[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    cv2.imshow("YOLOv8 Detection", frame)
    cv2.waitKey(1)

def handle_get_last_object(req):
    if last_detected_label and last_detected_info:
        return TriggerResponse(success=True, message=json.dumps({
            "label": last_detected_label,
            "price": last_detected_info.get("price", "N/A"),
            "location": last_detected_info.get("location", "unknown")
        }))
    return TriggerResponse(success=False, message="No object detected.")

def main():
    global pub
    rospy.init_node("object_camera_node", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    pub = rospy.Publisher("/object_info", String, queue_size=10)

    # Start service for intent_handler
    rospy.Service("get_last_object_info", Trigger, handle_get_last_object)

    rospy.loginfo("🟢 YOLOv8 Vision Node Started")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
