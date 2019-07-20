#!/usr/bin/env python

import rospy
from prrexamples.srv import WordCount
import sys

# Declare a node
rospy.init_node('service_client')

# Block until the servce is up
rospy.wait_for_service('word_count')

# Get the method (service proxy)
word_counter = rospy.ServiceProxy('word_count', WordCount)

# Grab all CLI params
words = ' '.join(sys.argv[1:])

# Call the method on the words, and receive the word count
word_count = word_counter(words)

# Print it out
print words, '->', word_count.count