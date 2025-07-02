#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import os

PRODUCTS_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"

# Load product data
if not os.path.exists(PRODUCTS_PATH):
    rospy.logerr(f"Product data file not found: {PRODUCTS_PATH}")
    PRODUCT_INFO = {}
else:
    with open(PRODUCTS_PATH, "r") as f:
        PRODUCT_INFO = json.load(f)

pub = None

def handle_intent(msg):
    user_text = msg.data.lower()
    rospy.loginfo(f"Received text: {user_text}")
    response = "Sorry, I didn't catch that."

    if "what is this" in user_text:
        try:
            rospy.wait_for_service("get_last_object_info", timeout=3)
            get_info = rospy.ServiceProxy("get_last_object_info", Trigger)
            srv_response = get_info()
            if srv_response.success:
                obj = json.loads(srv_response.message)
                response = f"This is a {obj['label']}. It costs RM{obj['price']} and is located in {obj['location']}."
            else:
                response = "I couldn't identify any object right now."
        except Exception as e:
            rospy.logerr(f"Service error: {e}")
            response = "Vision system is unavailable."
    else:
        for product in PRODUCT_INFO:
            if product in user_text:
                info = PRODUCT_INFO[product]
                if "where" in user_text or "location" in user_text:
                    response = f"{product.capitalize()} is in {info['location']}."
                elif "price" in user_text or "cost" in user_text:
                    response = f"The price of {product} is RM{info['price']}."
                else:
                    response = f"{product.capitalize()} is RM{info['price']} and located in {info['location']}."
                break

    rospy.loginfo(f"Processed response: {response}")
    pub.publish(response)

def main():
    global pub
    rospy.init_node("intent_handler_node")
    rospy.Subscriber("/intent_input", String, handle_intent)
    pub = rospy.Publisher("/intent_output", String, queue_size=10)

    rospy.loginfo("🧠 Intent handler running...")

    # Wait a moment to let TTS node get ready
    rospy.sleep(2.0)

    # Friendly greeting on startup
    greeting = "Hello, I am ready. May I help you?"
    rospy.loginfo(f"Greeting: {greeting}")
    pub.publish(greeting)

    rospy.spin()

if __name__ == "__main__":
    main()
