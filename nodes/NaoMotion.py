#!/usr/bin/env python
#coding: utf-8

# import all animations
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

from BSI import *
from naoqi import ALProxy
import random

ampBreath = 10.0
bpm = 20.0
states = {"normal": [4, 5, 7, 12, 13, 14], "impatient": [0, 1, 2, 6, 8, 9], "happy": [10, 11], "sad": [3]}

class NaoMotion():

	def __init__(self, NAO_IP, PORT):

	    # create proxies
	    self.motionProxy = ALProxy("ALMotion", NAO_IP, PORT)
	    self.audioProxy = ALProxy("ALAudioPlayer", NAO_IP, PORT)
	    self.postureProxy = ALProxy("ALRobotPosture", NAO_IP, PORT)
	    self.lastAnimation = 0

	def launch(self, state):

		# launch an animations in function of random state
		if state == 0:
			stretch1Animation(self.motionProxy)
		elif state == 1:
			stretch2Animation(self.motionProxy)
		elif state == 2:
			stretch3Animation(self.motionProxy)
		elif state == 3:
			nodAnimation(self.motionProxy)
		elif state == 4:
			helicopterAnimation(self.motionProxy, self.audioProxy)
		elif state == 5:
			sneezeAnimation(self.motionProxy, self.audioProxy)
		elif state == 6:
			backRubsAnimation(self.motionProxy)
		elif state == 7:
			lookHandAnimation(self.motionProxy)
		elif state == 8:
			relaxationAnimation(self.motionProxy)
		elif state == 9:
			restAnimation(self.motionProxy)
		elif state == 10:
			showMuscles1Animation(self.motionProxy)
		elif state == 11:
			showMuscles2Animation(self.motionProxy)
		elif state == 12:
			scratchHeadAnimation(self.motionProxy)
		elif state == 13:
			scratchHandAnimation(self.motionProxy)
		elif state == 14:
			scratchBottomAnimation(self.motionProxy)

	def launch_animation(self, cBSI):
		
		# control robot posture in order to avoid animation if the robot in crouched
		self.control_posture()

		# get the mood that the robot should have
		mood = cBSI.get_mood()
		allStates = states[mood]

		# remove last state in order to avoid same animation twice in a row
		#allState.remove(self.lastAnimation)

		#randomly select one state
		randomState = random.choice(allStates)

		# update last state parameter
		self.lastAnimation = randomState

		# launch animation
		self.launch(randomState)

	def sit_down(self):
		# go to posture sit down
		self.postureProxy.goToPosture("Crouch", 0.5)

	def stand_init(self):
		# go to posture stand init 
		self.postureProxy.goToPosture("StandInit", 0.5)
		
	def start_breathing(self):
		# start breathing
		self.motionProxy.setBreathConfig([['Bpm', bpm], ['Amplitude', ampBreath]])
		self.motionProxy.setBreathEnabled("Body", True)

	def stop_breathing(self):
		# stop breathing
		self.motionProxy.setBreathEnabled("Body", False)

	def control_posture(self):

		# if the robot isn't in standing posture, go to it
		if self.postureProxy.getPosture() == "Crouch":
			self.stand_init()
			self.start_breathing()

	def get_posture(self):
		return self.postureProxy.getPosture()