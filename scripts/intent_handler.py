#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import os

PRODUCTS_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"

# Load product database safely
if not os.path.exists(PRODUCTS_PATH):
    rospy.logerr(f"Product data file not found: {PRODUCTS_PATH}")
    PRODUCT_INFO = {}
else:
    with open(PRODUCTS_PATH, "r") as f:
        PRODUCT_INFO = json.load(f)

pub = None

def handle_intent(msg):
    user_text = msg.data.lower()
    rospy.loginfo(f"Received text: {user_text}")  # <-- Added logging

    response = "Sorry, I didn't catch that."

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

    rospy.loginfo(f"Processed response: {response}")  # <-- Added logging
    pub.publish(response)

def main():
    global pub
    rospy.init_node("intent_handler_node")
    rospy.Subscriber("/intent_input", String, handle_intent)
    pub = rospy.Publisher("/intent_output", String, queue_size=10)
    rospy.loginfo("🧠 Intent handler running...")
    rospy.spin()

if __name__ == "__main__":
    main()
