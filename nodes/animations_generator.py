#!/usr/bin/env python
#coding: utf-8

import rospy
from std_msgs.msg import String
from naoqi import ALProxy

# import all animations
import animations
from animations.nod_pose import * 
from animations.helicopter_pose import *
from animations.scratchBottom_pose import *
from animations.scratchHand_pose import *
from animations.scratchHead_pose import *
from animations.sneeze_pose import *
from animations.stretch1_pose import * 
from animations.stretch2_pose import *
from animations.stretch3_pose import *
from animations.showMuscles1_pose import *
from animations.showMuscles2_pose import *
from animations.lookHand_pose import *
from animations.backRubs_pose import *
from animations.relaxation_pose import *
from animations.rest_pose import *

ampBreath = 10.0
bpm = 20.0
robotBusy = True



def callback_activityRobot(data):
    global robotBusy
    if str(data.data) == "WAITING_FOR_FEEDBACK":
        if postureProxy.getPosture() == "Crouch":
            # the robot needs to stand up in order to launch animations
            wake_up()

        # the robot is not busy anymore
        robotBusy = False

            
    
    elif str(data.data) == "PUBLISHING_WORD":
        #if postureProxy.getPosture() == "StandInit" or postureProxy.getPosture() == "Stand":
             # the robot needs to sit down up in order to write
            #sit_down()

            # the robot is busy (doing a task)
        robotBusy = True

    else:
        robotBusy = True


def sit_down():
    # go to posture sit down and stop breathing
    motionProxy.setBreathEnabled("Body", False)
    postureProxy.goToPosture("Crouch", 0.5)

def wake_up():
    # go to posture stand init and start breathing
    postureProxy.goToPosture("StandInit", 0.5)
    motionProxy.setBreathConfig([['Bpm', bpm], ['Amplitude', ampBreath]])
    motionProxy.setBreathEnabled("Body", True)


def callback_animation(data):

    # if NAO isn't busy, animations are authorized to be launched
    if robotBusy == False:

        # if the robot isn't in standing posture, go to it
        if postureProxy.getPosture() == "Crouch":
            wake_up()

        elif postureProxy.getPosture() == "StandInit" or postureProxy.getPosture() == "Stand":

            #@TO_DO change that in order to do animation in function of other parameters
            state = int(data.data)

            # launch an animations in function of random state
            if state == 0:
                stretch1Animation(motionProxy)
            elif state == 1:
                stretch2Animation(motionProxy)
            elif state == 2:
                stretch3Animation(motionProxy)
            elif state == 3:
                nodAnimation(motionProxy)
            elif state == 4:
                helicopterAnimation(motionProxy, audioProxy)
            elif state == 5:
                sneezeAnimation(motionProxy, audioProxy)
            elif state == 6:
                backRubsAnimation(motionProxy)
            elif state == 7:
                lookHandAnimation(motionProxy)
            elif state == 8:
                relaxationAnimation(motionProxy)
            elif state == 9:
                restAnimation(motionProxy)
            elif state == 10:
                showMuscles1Animation(motionProxy)
            elif state == 11:
                showMuscles2Animation(motionProxy)
            elif state == 12:
                scratchHeadAnimation(motionProxy)
            elif state == 13:
                scratchHandAnimation(motionProxy)
            elif state == 14:
                scratchBottomAnimation(motionProxy)

        


        # posture -> only use by teleOp, idle mode cannot launcn the posture animation
        elif state == -1:
            # go to init pose
            postureProxy.goToPosture("StandInit", 0.5)
        elif state == -2:
            # activate breath module
            motionProxy.setBreathConfig([['Bpm', bpm], ['Amplitude', ampBreath]])
            motionProxy.setBreathEnabled("Body", True)
        elif state == -3:
            # stop breath module
            motionProxy.setBreathEnabled("Body", False)
        elif state == -4:
            # sit down
            motionProxy.sit_down()


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)



if __name__ == "__main__":

    # create a unique node
    rospy.init_node("animations_generator")

    # get all parameteres from launch file
    NAO_IP = rospy.get_param('~nao_ip', '10.0.0.10')
    PORT = rospy.get_param('~nao_port', "9559")
    TOPIC_ANIMATIONS = rospy.get_param('~topic_animations', 'topic_animations')
    TOPIC_ACTIVITY = rospy.get_param('~topic_activity', 'state_activity')

    # create proxies
    motionProxy = ALProxy("ALMotion", NAO_IP, int(PORT))
    audioProxy = ALProxy("ALAudioPlayer", NAO_IP, int(PORT))
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, int(PORT))

    # subscribe to topic that send animations order
    rospy.Subscriber(TOPIC_ANIMATIONS, String, callback_animation)

    # subscribe to topic regarding the state machine
    rospy.Subscriber(TOPIC_ACTIVITY, String, callback_activityRobot)

     # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()