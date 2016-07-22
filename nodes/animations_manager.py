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


# period (in seconds) at which animation are launched 
# @TODO in NAOMOTION ???
periodAnimation = 10.0
# global variable telling if the robot is busy or not
robotBusy = True


def callback_activityRobot(data):
    global robotBusy
    if str(data.data) == "WAITING_FOR_FEEDBACK":
        if cNaoMotion.get_posture() == "Crouch":
            # the robot needs to stand up in order to launch animations
            cNaoMotion.stand_init()
            cNaoMotion.start_breathing()

        # the robot is not busy anymore
        robotBusy = False

        #init CBSI time
        cBSI.init_time_interaction()

        # launch timer to update BSI mood
        timerUpdateBSI = rospy.Timer(rospy.Duration(0.1), callback_updateBSI)

        # launch timer that will triger animations at a certain frequency given in param
        timerLaunchAnimation = rospy.Timer(rospy.Duration(periodAnimation), callback_timer_animation)


    # @TODO change that to try catch or put error
    else:
        # the robot is busy (doing a task)
        robotBusy = True

        try:
            timerLaunchAnimation.shutdown()
            timerUpdateBSI.shutdown()
        except:
            pass


def callback_animations(data):

    # launch nao animation given in the topic
    cNaoMotion.launch(data.data)

def callback_poses(data):

    state = data.data

    # posture -> only use by teleOp, idle mode cannot launch the posture animation
    if state == 0:
        # go to init pose
        cNaoMotion.stand_init()

    elif state == 1:
        # go to crouch pose
        cNaoMotion.sit_down()

    elif state == 2:
        # start breathing
        cNaoMotion.start_breathing()

    elif state == 3:
        # stop breathing
        cNaoMotion.stop_breathing()

def callback_settings(data):

    global parkinson_scale
    global robotBusy
    state = data.data

    if state == 0:
        robotBusy = False

    if state == 1:
        launch_timer()

def callback_user_feedback(data):

    if data.data == "+":
        cBSI.set_mood("happy")
        
    elif data.data == "-":
        cBSI.set_mood("sad")

def callback_timer_animation(event):
    if robotBusy == False:
        cNaoMotion.launch_animation(cBSI)

def callback_updateBSI(event):
    cBSI.update_mood(event.last_duration)
 

if __name__ == "__main__":

    # create a unique node
    rospy.init_node("animations_manager")

    # get all parameteres from launch file
    NAO_IP = rospy.get_param('~nao_ip', '10.0.0.10')
    PORT = rospy.get_param('~nao_port', "9559")
    # topics name
    TOPIC_ANIMATIONS = rospy.get_param('~topic_animations', 'topic_animations')
    TOPIC_ACTIVITY = rospy.get_param('~topic_activity', 'state_activity')
    TOPIC_POSES = rospy.get_param('~topic_poses', 'topic_poses')
    TOPIC_SETTINGS = rospy.get_param('~topic_settings', 'topic_settings')
    TOPIC_USER_FEEDBACK = rospy.get_param('~topic_user_feedback', 'user_feedback')
    TOPIC_NB_REPETITION = rospy.get_param('~topic_nb_repetition', 'nb_repetition')

    # initial frequency at which animations are launch
    periodAnimation = float(rospy.get_param('~periodAnimation', '10'))

    # create animation class that will launch the animations
    cNaoMotion = NaoMotion(NAO_IP, int(PORT))

    # create the BSI class that will contain the behavior that the robot needs to show
    cBSI = BSI()

    # subscribe to topics that send animations, poses and settings orders
    rospy.Subscriber(TOPIC_ANIMATIONS, UInt8, callback_animations)
    rospy.Subscriber(TOPIC_POSES, UInt8, callback_poses)
    rospy.Subscriber(TOPIC_SETTINGS, UInt8, callback_settings)
    rospy.Subscriber(TOPIC_USER_FEEDBACK, String, callback_user_feedback)

    # subscribe to topic regarding the state machine
    rospy.Subscriber(TOPIC_ACTIVITY, String, callback_activityRobot)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()







