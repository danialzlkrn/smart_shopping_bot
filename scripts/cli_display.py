#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

def handle_intent_output(msg):
    rospy.loginfo("\n🧠 [INTENT OUTPUT]: %s\n", msg.data)

def handle_object_info(msg):
    try:
        data = json.loads(msg.data)
        for label, info in data.items():
            print("\n👁️ [OBJECT DETECTED]")
            print(f"  🏷️ Label    : {label}")
            print(f"  💵 Price    : RM{info.get('price', 'N/A')}")
            print(f"  🧭 Location : {info.get('location', 'unknown')}")
            print(f"  🗂️ Type     : {info.get('type', 'unknown')}\n")
    except json.JSONDecodeError:
        rospy.logwarn("⚠️ Failed to decode object info JSON.")

def main():
    rospy.init_node("cli_display_node", anonymous=True)
    rospy.Subscriber("/intent_output", String, handle_intent_output)
    rospy.Subscriber("/object_info", String, handle_object_info)

    rospy.loginfo("📺 CLI Display Node running. Showing live updates...")
    rospy.spin()

if __name__ == "__main__":
    main()
