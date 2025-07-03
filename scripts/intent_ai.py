#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import os
import openai

# === CONFIGURATION ===
PRODUCTS_PATH = "/home/mustar/catkin_ws/src/smart_shopping_bot/config/products_data.json"
OPENAI_API_KEY = "sk-abcdqrstefgh5678abcdqrstefgh5678abcdqrst"
OPENAI_MODEL = "gpt-3.5-turbo"

# === GLOBALS ===
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
                rospy.logerr("❌ Failed to parse product data.")
                PRODUCT_INFO = {}

def normalize(text):
    return text.lower().replace(" ", "_")

def fallback_response(user_text):
    normalized_text = normalize(user_text)
    for key, info in PRODUCT_INFO.items():
        if key in normalized_text or key.replace("_", " ") in user_text:
            if "where" in user_text or "location" in user_text:
                return f"{key.replace('_', ' ').capitalize()} is in {info['location']}."
            elif "price" in user_text or "cost" in user_text or "how much" in user_text:
                return f"The price of {key.replace('_', ' ')} is RM{info['price']}."
            else:
                return f"{key.replace('_', ' ').capitalize()} costs RM{info['price']} and is in {info['location']}."
    return None

def call_vision_service():
    try:
        rospy.wait_for_service("get_last_object_info", timeout=3)
        get_info = rospy.ServiceProxy("get_last_object_info", Trigger)
        resp = get_info()
        if resp.success:
            obj = json.loads(resp.message)
            return f"This is a {obj['label']}. It costs RM{obj['price']} and is located in {obj['location']}."
        else:
            return "I couldn't identify any object right now."
    except rospy.ROSException:
        return "The object recognition service timed out."
    except Exception as e:
        rospy.logerr(f"❌ Vision service error: {e}")
        return "Vision system is unavailable."

def generate_ai_response(user_text, context=None):
    try:
        openai.api_key = OPENAI_API_KEY
        prompt = (
            f"You are a helpful shopping assistant robot in a supermarket.\n"
            f"Only answer questions related to products, prices, and locations.\n"
            f"If unsure or if the question is not shopping-related, reply with: 'How may I assist you with your shopping today?'\n"
            f"Available products: {', '.join(PRODUCT_INFO.keys())}\n"
            f"User query: {user_text}\n"
            f"Response:"
        )

        completion = openai.ChatCompletion.create(
            model=OPENAI_MODEL,
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": user_text}
            ],
            temperature=0.5,
            max_tokens=100
        )

        reply = completion.choices[0].message['content'].strip()
        if not reply or any(x in reply.lower() for x in ["i cannot", "i don't know", "as an ai"]):
            return "How may I assist you with your shopping today?"
        return reply
    except Exception as e:
        rospy.logerr(f"❌ OpenAI API error: {e}")
        return None

def handle_intent(msg):
    user_text = msg.data.lower()
    rospy.loginfo(f"📥 Received: {user_text}")
    response = None

    if "what is this" in user_text or "what's this" in user_text:
        response = call_vision_service()
    else:
        response = fallback_response(user_text)
        if not response:
            ai_reply = generate_ai_response(user_text)
            if ai_reply:
                response = ai_reply
            else:
                response = fallback_response(user_text) or "How may I assist you with your shopping today?"

    rospy.loginfo(f"📤 Responding: {response}")
    pub.publish(response)

def main():
    global pub
    rospy.init_node("intent_handler_ai_node")
    pub = rospy.Publisher("/intent_output", String, queue_size=10)
    rospy.Subscriber("/intent_input", String, handle_intent)

    rospy.loginfo("🧠 AI-based intent handler node starting...")
    load_product_data()
    rospy.sleep(2.0)

    greeting = "Hello! I'm your smart shopping assistant. How can I help you today?"
    pub.publish(greeting)
    rospy.spin()

if __name__ == "__main__":
    main()
