#!/usr/bin/env python
# rosrun rosbook service_server.py
# rosservice call word_count 'one two three'

import rospy
from rosbook.srv import WordCount, WordCountResponse

def count_words(request):
    return WordCountResponse(len(request.words.split()))

rospy.init_node('service_server')
service = rospy.Service('word_count', WordCount, count_words)
rospy.spin()