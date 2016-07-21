#!/usr/bin/env python
#coding: utf-8

import rospy
from std_msgs.msg import String

def talker():
	# make this into a ROS node called 'teleOp'
	rospy.init_node('tele_op')

	# get params from launch file
	TOPIC_ANIMATIONS = rospy.get_param('~topic_animations', 'topic_animations')

	# create a publlisher for the 'command_animation' topic
	# of type std_msgs/String - do not forget to import the type
	cmd_publisher = rospy.Publisher(TOPIC_ANIMATIONS, String, queue_size=10)

	# infinite loop that listen to user commands
	while True:
		cmd = raw_input()
		if int(cmd) >= -4 and int(cmd) <= 14:
			# publish the command in the topic command_animation
			cmd_publisher.publish(cmd)


if __name__ == '__main__':
	talker()