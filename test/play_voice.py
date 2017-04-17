#!/usr/bin/env python
import rospy
import pygame
import sys

from std_msgs.msg import Bool

#Topic names.
launching_topic = rospy.get_param("/topic/READY_FOR_DRIVING")

def playMp3(filename):
    pygame.mixer.init()
    pygame.mixer.music.load(filename)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
    	continue

def launchingCallback(msg):
	if msg.data: playMp3(sys.path[0] + '/../samples/Booting.mp3')

def main():
	#Init ros.
	rospy.init_node('arc_voice')
	#Init Subscriber.
	rospy.Subscriber(launching_topic, Bool, launchingCallback)

if __name__ == '__main__':
	main()




