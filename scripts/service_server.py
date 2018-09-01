#!/usr/bin/env python
# rosrun rosbook service_server.py
# rosservice call word_count 'one two three'

import rospy
from rosbook.srv import WordCount, WordCountResponse

# Service handler, called when a client wants to invoke the service. It receives
# the string, computes the number of words, and returns it
def count_words(request):
    return WordCountResponse(len(request.words.split()))

# Declare a node
rospy.init_node('service_server')

# Announce that we are a service
service = rospy.Service('word_count', WordCount, count_words)

# Wait until ^c or a service request
rospy.spin()