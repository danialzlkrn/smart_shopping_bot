#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

def print_object_info(msg):
    try:
        obj_info = json.loads(msg.data)
        for label, info in obj_info.items():
            print("\n📦 Detected Object:")
            print(f"   🔹 Label   : {label}")
            print(f"   💵 Price   : RM{info.get('price', 'N/A')}")
            print(f"   🏬 Location: {info.get('location', 'unknown')}")
            print(f"   🧩 Type    : {info.get('type', 'unknown')}")
            print("-" * 40)
    except Exception as e:
        rospy.logwarn(f"[CLI] Failed to parse object info: {e}")

def print_intent_output(msg):
    print(f"\n🧠 Assistant says: {msg.data}")
    print("-" * 40)

def main():
    rospy.init_node("cli_display_node")
    rospy.Subscriber("/intent_output", String, print_intent_output)

    rospy.loginfo("[CLI] Live display started.")
    rospy.spin()

if __name__ == "__main__":
    main()
