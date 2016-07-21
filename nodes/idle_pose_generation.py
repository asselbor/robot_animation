#!/usr/bin/env python
#coding: utf-8

import rospy
from std_msgs.msg import String
import almath
import time
import random

def talker():
    # make this into a ROS node called 'teleOp'
    rospy.init_node('idle_pose_generation')

    # get params from launch file
    TOPIC_ANIMATIONS = rospy.get_param('~topic_animations', 'topic_animations')
    FREQ_ANIMATION = float(rospy.get_param('~frequecyAnimation', '0.03'))
    FREQ_UPDATE = float(rospy.get_param('~frequecyUpdate', '100'))

    # create a publlisher for the 'command_animation' topic
    # of type std_msgs/String - do not forget to import the type
    cmd_publisher = rospy.Publisher(TOPIC_ANIMATIONS, String, queue_size=10)


    t0 = time.time()
    t_animation = 0
    randomState = 0

    # infinite loop that listen to user commands
    while True:

        # if time for new animation reached
        if t_animation >= 1/FREQ_ANIMATION:

            # select random animation
            allState = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

            # remove last state in order to avoid same animation twice in a row
            allState.remove(randomState)

            #randomly select one state
            randomState = random.choice(allState)

            # argument to post
            arg = str(randomState)
            cmd_publisher.publish(arg)

            #update time
            t0 = time.time()

        # sleep for period of time according to frequency
        time.sleep(1/float(FREQ_UPDATE))

        # update time
        t_animation = time.time() - t0


if __name__ == '__main__':
    talker()
    rospy.spin()





   
