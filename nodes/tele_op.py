#!/usr/bin/env python
#coding: utf-8

################################################################################################
## this node is a remote control allowing manual modification of several parameters as    
## well as the manual start of a animation
################################################################################################

import rospy
from std_msgs.msg import String, UInt8

def talker():
	# make this into a ROS node called 'teleOp'
	rospy.init_node('tele_op')

	# get params from launch file
	TOPIC_ANIMATIONS = rospy.get_param('~topic_animations', 'topic_animations')
	TOPIC_POSES = rospy.get_param('~topic_poses', 'topic_poses')
	TOPIC_SETTINGS = rospy.get_param('~topic_settings', 'topic_settings')

	# create a publlisher for the 'topic_animations' topic
	# of type std_msgs/Int8 - do not forget to import the type
	cmd_publisher_animations = rospy.Publisher(TOPIC_ANIMATIONS, UInt8, queue_size=10)

	# create a publlisher for the 'topic_poses' topic
	cmd_publisher_poses = rospy.Publisher(TOPIC_POSES, UInt8, queue_size=10)

	# create a publlisher for the 'topic_settings' topic
	cmd_publisher_settings = rospy.Publisher(TOPIC_SETTINGS, UInt8, queue_size=10)

	# infinite loop that listen to user commands
	while True:

		cmd = input()
		mode = str(cmd[0])
		arg = int(cmd[1])

		# publish the command in the corresponding topic
		if mode == "animation":
			cmd_publisher_animations.publish(arg)

		elif mode == "pose":
			cmd_publisher_poses.publish(arg)

		elif mode == "setting":
			cmd_publisher_settings.publish(arg)

if __name__ == '__main__':
	talker()



