#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import os

PRODUCTS_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"

PRODUCT_INFO = {}
pub = None

def load_product_data():
    global PRODUCT_INFO
    if not os.path.exists(PRODUCTS_PATH):
        rospy.logerr(f"❌ Product data file not found: {PRODUCTS_PATH}")
        PRODUCT_INFO = {}
    else:
        with open(PRODUCTS_PATH, "r") as f:
            try:
                PRODUCT_INFO = json.load(f)
                rospy.loginfo(f"✅ Loaded {len(PRODUCT_INFO)} products.")
            except json.JSONDecodeError:
                rospy.logerr("❌ Failed to parse product data JSON.")
                PRODUCT_INFO = {}

def normalize(text):
    return text.lower().replace(" ", "_")

def handle_intent(msg):
    user_text = msg.data.lower()
    rospy.loginfo(f"📥 Received text: {user_text}")
    response = "Sorry, I didn't catch that."

    # Handle vision request
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
        except rospy.ROSException:
            response = "The object recognition service took too long to respond."
        except Exception as e:
            rospy.logerr(f"❌ Service error: {e}")
            response = "Vision system is unavailable."
    else:
        normalized_text = normalize(user_text)
        for product_key, info in PRODUCT_INFO.items():
            if product_key in normalized_text or product_key.replace("_", " ") in user_text:
                if "where" in user_text or "location" in user_text:
                    response = f"{product_key.replace('_', ' ').capitalize()} is in {info['location']}."
                elif "price" in user_text or "cost" in user_text:
                    response = f"The price of {product_key.replace('_', ' ')} is RM{info['price']}."
                else:
                    response = f"{product_key.replace('_', ' ').capitalize()} is RM{info['price']} and located in {info['location']}."
                break

    rospy.loginfo(f"📤 Response: {response}")
    pub.publish(response)

def main():
    global pub
    rospy.init_node("intent_handler_node")
    pub = rospy.Publisher("/intent_output", String, queue_size=10)
    rospy.Subscriber("/intent_input", String, handle_intent)

    load_product_data()

    rospy.loginfo("🧠 Intent handler running...")
    rospy.sleep(2.0)

    greeting = "Hello, I am ready. May I help you?"
    rospy.loginfo(f"Greeting: {greeting}")
    pub.publish(greeting)

    rospy.spin()

if __name__ == "__main__":
    main()