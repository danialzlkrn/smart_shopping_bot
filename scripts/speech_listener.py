#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

pub = None

def recognize_google_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        rospy.loginfo("🎤 Listening for user speech...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
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

def listener():
    global pub
    rospy.init_node('speech_listener_node', anonymous=True)
    pub = rospy.Publisher('/intent_input', String, queue_size=10)

    rospy.loginfo("🎙️ Speech listener node started")
    while not rospy.is_shutdown():
        result = recognize_google_speech()
        if result:
            pub.publish(result)

if __name__ == '__main__':
    listener()
