#!/usr/bin/env python
#coding: utf-8

################################################################################################
## this node contains transformation algorithmes allowing interpretation of behaviors given by    
## b_s_i node and will select and modify animation given from the pose repertory in order
## to reflect in the robot the behavior given from b_s_i node 
################################################################################################

import rospy
from std_msgs.msg import String, UInt8
import threading
from NaoMotion import *
from BSI import *


# global variable telling if the robot is busy or not
robotBusy = True


########################################## teleOp call back only ##########################################
def TOp_callback_animations(data):

    # from the TELE_OP only
    # launch nao animation given in the topic
    cNaoMotion.launchAnimation(data.data)

def TOp_callback_poses(data):

    state = data.data

    # posture -> only use by teleOp, idle mode cannot launch the posture animation
    if state == 0:
        # go to init pose
        cNaoMotion.stand()

    elif state == 1:
        # go to crouch pose
        cNaoMotion.sit_down()

    elif state == 2:
        # start breathing
        cNaoMotion.start_breathing()

    elif state == 3:
        # stop breathing
        cNaoMotion.stop_breathing()

def TOp_callback_settings(data):

    global parkinson_scale
    global robotBusy

    state = data.data

    if state == 0:
        robotBusy = False

    if state == 1:

        # launch timer that will triger animations at a cerxtain frequency given in param
        timerLaunchAnimation = rospy.Timer(rospy.Duration(cBSI.periodAnimation), callback_timer_animation, oneshot=True)
########################################## ##################### ##########################################

def callback_activityRobot(data):
  
    state = str(data.data)
    global robotBusy

    if state == "WAITING_FOR_FEEDBACK" or state == "WAITING_FOR_WORD":
        if cNaoMotion.get_posture() == "Crouch":
            # the robot needs to stand up in order to launch animations
            cNaoMotion.stand()

        # make nao breath
        cNaoMotion.breath(cBSI)

        # face tracking
        cNaoMotion.faceFolowing(cBSI.faceTracking)

        if cBSI.periodAnimation != None:
            # the robot is not busy anymore
            robotBusy = False

            #init CBSI time
            cBSI.init_time_interaction()

            # launch timer that will triger animations at a certain frequency given in param
            try:
                timerLaunchAnimation.shutdown()
            except:
                pass
            timerLaunchAnimation = rospy.Timer(rospy.Duration(cBSI.periodAnimation), callback_timer_animation, oneshot=True)

    elif state == "PUBLISHING_WORD":
        # the robot is busy (doing a task)
        robotBusy = True

        # stop animation timer
        try:
            timerLaunchAnimation.shutdown()
        except:
            pass

        cNaoMotion.stand()


def callback_timer_animation(event):

    global timerLaunchAnimation
    if robotBusy == False:
        cNaoMotion.nonFunctionalMove(cBSI)

        # update timer frequency and relaunch it
        timerLaunchAnimation.shutdown()
        timerLaunchAnimation = rospy.Timer(rospy.Duration(cBSI.periodAnimation), callback_timer_animation, oneshot=True)
 

if __name__ == "__main__":

    # create a unique node
    rospy.init_node("animations_manager")

    # get parameteres from launch file
    NAO_IP = rospy.get_param('~nao_ip')
    PORT = rospy.get_param('~nao_port')
    # get value of parkinson scale
    parkinson_scale = int(rospy.get_param('~parkinson_scale'))
    # get robot's id
    robotId = int(rospy.get_param('~id_robot'))
    # initial frequency at which animations are launch
    periodUpdateTimerBSI = float(rospy.get_param('~periodUpdateTimerBSI'))

    # create the BSI class that will contain the behavior that the robot needs to show
    cBSI = BSI(parkinson_scale)
    # create animation class that will launch the animations
    cNaoMotion = NaoMotion(NAO_IP, int(PORT), cBSI, robotId)

    # subscribers
    TOPIC_ANIMATIONS = rospy.get_param('~topic_animations')
    rospy.Subscriber(TOPIC_ANIMATIONS, UInt8, TOp_callback_animations)
    TOPIC_POSES = rospy.get_param('~topic_poses')
    rospy.Subscriber(TOPIC_POSES, UInt8, TOp_callback_poses)
    TOPIC_SETTINGS = rospy.get_param('~topic_settings')
    rospy.Subscriber(TOPIC_SETTINGS, UInt8, TOp_callback_settings)
    TOPIC_ACTIVITY = rospy.get_param('~topic_activity')
    rospy.Subscriber(TOPIC_ACTIVITY, String, callback_activityRobot)

    # debug topic
    debug = rospy.Publisher("debug", String, queue_size=10)

    # give time for rospy to connect
    rospy.sleep(1)

    # launch timer for animations
    if cBSI.periodAnimation != None:
        timerLaunchAnimation = rospy.Timer(rospy.Duration(cBSI.periodAnimation), callback_timer_animation)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





