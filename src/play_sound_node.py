#!/usr/bin/env python3

import rospy
import hashlib
import os
from std_msgs.msg import String
from gtts import gTTS
from playsound import playsound
from std_srvs.srv import Empty
#from my_msgs.srv import CleanCache

cache_dir = os.path.join(os.path.expanduser('~'), '.cache', 'text_to_speech')
if not os.path.exists(cache_dir):
    os.makedirs(cache_dir)

def play_text(text):
    hash = hashlib.md5(text.data.encode()).hexdigest()
    filename = os.path.join(cache_dir, f"{hash}.mp3")
    if os.path.exists(filename):
        playsound(filename)
    else:
        tts = gTTS(text.data, lang='en')
        tts.save(filename)
        playsound(filename)

def clean_cache(req):
    for filename in os.listdir(cache_dir):
        os.remove(os.path.join(cache_dir, filename))
    return []

if __name__ == '__main__':
    #play_text("hello")
    rospy.init_node('text_to_speech')
    rospy.Subscriber('text_to_speak', String, play_text)
    rospy.Service('clean_cache', Empty, clean_cache)
    rospy.spin()