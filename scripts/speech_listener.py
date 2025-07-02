#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

pub = None

def recognize_google_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        rospy.loginfo("🎤 Listening for user speech...")
        audio = recognizer.listen(source)
    try:
        text = recognizer.recognize_google(audio)
        rospy.loginfo("✅ Recognized: %s", text)
        return text.lower()
    except sr.UnknownValueError:
        rospy.logwarn("🤷 Couldn't understand audio.")
    except sr.RequestError as e:
        rospy.logerr("❌ Google Speech API error: %s", e)
    return None

def handle_command(text):
    if not text:
        return
    rospy.loginfo("📤 Publishing: %s", text)
    pub.publish(text)

def listener():
    global pub
    rospy.init_node('speech_listener_node', anonymous=True)
    pub = rospy.Publisher('/intent_input', String, queue_size=10)
    rate = rospy.Rate(0.1)  # every 10 seconds
    while not rospy.is_shutdown():
        result = recognize_google_speech()
        handle_command(result)
        rate.sleep()

if __name__ == '__main__':
    listener()
