#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import pyttsx3

def speak_text(text):
    rospy.loginfo("Speaking: %s", text)
    tts_engine.say(text)
    tts_engine.runAndWait()

def callback(data):
    rospy.loginfo("Received text for TTS: %s", data.data)
    speak_text(data.data)

def tts_listener():
    rospy.init_node('tts_output_node', anonymous=True)
    rospy.Subscriber('/intent_output', String, callback)
    rospy.loginfo("🔊 TTS node is active and listening on /intent_output")
    rospy.spin()

if __name__ == '__main__':
    try:
        tts_engine = pyttsx3.init()
        tts_engine.setProperty('rate', 150)
        tts_engine.setProperty('volume', 1.0)
        rospy.loginfo("✅ TTS Engine initialized")
        tts_engine.say("Text to speech node started. The robot is hearing")
        tts_engine.runAndWait()
        tts_listener()
    except rospy.ROSInterruptException:
        pass
