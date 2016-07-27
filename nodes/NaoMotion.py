#!/usr/bin/env python
#coding: utf-8

# import all animations
import animations.nod_pose  
import animations.helicopter_pose 
import animations.scratchBottom_pose 
import animations.scratchHand_pose 
import animations.scratchHead_pose 
import animations.sneeze_pose 
import animations.stretch1_pose  
import animations.stretch2_pose 
import animations.stretch3_pose 
import animations.showMuscles1_pose 
import animations.showMuscles2_pose 
import animations.lookHand_pose 
import animations.backRubs_pose 
import animations.relaxation_pose 
import animations.rest_pose
import animations.winner_pose
import animations.winner2_pose
import animations.happy_pose
import animations.happy2_pose
import animations.happy3_pose
import animations.proud_pose
import animations.relieved_pose
import animations.alienated_pose
import animations.disappointed_pose
import animations.embarassed_pose
import animations.exhausted_pose
import animations.exhausted2_pose
import animations.excited_pose
import animations.excited2_pose
import animations.stubborn_pose
import animations.crying_pose
import animations.desperate_pose
import animations.desperate2_pose
import animations.desperate3_pose
import animations.shootGround_pose
import animations.pensive_pose
import animations.crying_pose

from BSI import *
from naoqi import ALProxy
import random
import time

ampBreath = 10.0
bpm = 20.0
# category pensive (25) ??
states = {"normal": [4, 5, 7, 12, 13, 14, 22, 35], "impatient": [0, 1, 2, 6, 8], "happy": [10, 11, 15, 16, 17, 18, 19, 20, 21], "sad": [3, 23, 24, 29, 30, 31, 32, 33, 34, 36], "tired": [9, 25, 26], "excited": [27, 28]}
bodyBottom = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]


class NaoMotion():

	def __init__(self, NAO_IP, PORT):

	    # create proxies
	    self.motionProxy = ALProxy("ALMotion", NAO_IP, PORT)
	    self.audioProxy = ALProxy("ALAudioPlayer", NAO_IP, PORT)
	    self.postureProxy = ALProxy("ALRobotPosture", NAO_IP, PORT)
	    self.faceDetectionProxy = ALProxy("ALBasicAwareness", NAO_IP, PORT)
	    self.lastAnimation = 0

	def runMotion(self, pose, factorSpeed, factorAmpl, audioFile = None, delayAudioInit = None):

		if audioFile != None:
			self.audioProxy.post.playFile(audioFile)

		times = self.increaseSpeed(pose.times, factorSpeed)
		keys = self.increaseAmplitude(pose.keys, pose.names, factorAmpl)
		names = pose.names

		self.motionProxy.angleInterpolationBezier(names, times, keys)

		if delayAudioInit != None:
			time.sleep(delayAudioInit)

	def launch(self, state):

		#@TODO parametre du BSI
		factorSpeed = 1
		factorAmpl = 1

		# launch an animations in function of random state
		if state == 0:
			self.runMotion(animations.stretch1_pose, factorSpeed, factorAmpl)

		elif state == 1:
			self.runMotion(animations.stretch2_pose, factorSpeed, factorAmpl)

		elif state == 2:
			self.runMotion(animations.stretch3_pose, factorSpeed, factorAmpl)

		#elif state == 3:
			#@TO_DO update that
			#nodAnimation(self.motionProxy)

		elif state == 4:
			#@TODO probleme avec musique et vitesse
			self.runMotion(animations.helicopter_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/helicopter.wav")

		elif state == 5:
			#@TODO probleme avec musique et vitesse
			self.runMotion(animations.sneeze_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/sneeze.wav", 1.0)

		elif state == 6:
			self.runMotion(animations.backRubs_pose, factorSpeed, factorAmpl)

		elif state == 7:
			self.runMotion(animations.lookHand_pose, factorSpeed, factorAmpl)

		elif state == 8:
			self.runMotion(animations.relaxation_pose, factorSpeed, factorAmpl)

		elif state == 9:
			self.runMotion(animations.rest_pose, factorSpeed, factorAmpl)

		elif state == 10:
			self.runMotion(animations.showMuscles1_pose, factorSpeed, factorAmpl)
			
		elif state == 11:
			self.runMotion(animations.showMuscles2_pose, factorSpeed, factorAmpl)

		elif state == 12:
			self.runMotion(animations.scratchHead_pose, factorSpeed, factorAmpl)

		elif state == 13:
			self.runMotion(animations.scratchHand_pose, factorSpeed, factorAmpl)

		elif state == 14:
			self.runMotion(animations.scratchBottom_pose, factorSpeed, factorAmpl)

		elif state == 15:
			self.runMotion(animations.winner_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/applause.wav")

		elif state == 16:
			self.runMotion(animations.winner2_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/applause.wav")

		elif state == 17:
			self.runMotion(animations.happy_pose, factorSpeed, factorAmpl)

		elif state == 18:
			self.runMotion(animations.happy2_pose, factorSpeed, factorAmpl)

		elif state == 19:
			self.runMotion(animations.happy3_pose, factorSpeed, factorAmpl)

		elif state == 20:
			self.runMotion(animations.proud_pose, factorSpeed, factorAmpl)

		elif state == 21:
			self.runMotion(animations.relieved_pose, factorSpeed, factorAmpl)

		elif state == 22:
			self.runMotion(animations.alienated_pose, factorSpeed, factorAmpl)

		elif state == 23:
			self.runMotion(animations.disappointed_pose, factorSpeed, factorAmpl)

		elif state == 24:
			self.runMotion(animations.embarassed_pose, factorSpeed, factorAmpl)

		elif state == 25:
			self.runMotion(animations.exhausted_pose, factorSpeed, factorAmpl)

		elif state == 26:
			self.runMotion(animations.exhausted2_pose, factorSpeed, factorAmpl)

		elif state == 27:
			self.runMotion(animations.excited_pose, factorSpeed, factorAmpl)

		elif state == 28:
			self.runMotion(animations.excited2_pose, factorSpeed, factorAmpl)

		elif state == 29:
			self.runMotion(animations.stubborn_pose, factorSpeed, factorAmpl)

		elif state == 30:
			self.runMotion(animations.stubborn_pose, factorSpeed, factorAmpl)
			
		elif state == 31:
			self.runMotion(animations.desperate_pose, factorSpeed, factorAmpl)

		elif state == 32:
			self.runMotion(animations.desperate2_pose, factorSpeed, factorAmpl)

		elif state == 33:
			self.runMotion(animations.desperate3_pose, factorSpeed, factorAmpl)

		elif state == 34:
			self.runMotion(animations.shootGround_pose, factorSpeed, factorAmpl)

		elif state == 35:
			self.runMotion(animations.pensive_pose, factorSpeed, factorAmpl)

		elif state == 36:
			self.runMotion(animations.crying_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/cry.wav")			

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

		# avoid eye traking and launch animation
		self.faceDetectionProxy.stopAwareness()
		self.start_breathing()
		self.launch(randomState)
		self.faceDetectionProxy.startAwareness()

	def sit_down(self):
		# go to posture sit down
		self.postureProxy.goToPosture("Crouch", 0.5)

	def stand_init(self):
		# go to posture stand init 
		self.postureProxy.goToPosture("StandInit", 0.5)
		
	def start_breathing(self):
		# start breathing
		self.motionProxy.setBreathConfig([['Bpm', bpm], ['Amplitude', ampBreath]])

		if self.motionProxy.getBreathEnabled("Body") == False:
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

	def increaseSpeed(self, times, factor):
		for i in xrange(len(times)):
			times[i] = [x / float(factor) for x in times[i]]

		return times

	def increaseAmplitude(self, keys, names, factor):
		for i in xrange(len(names)):
			if names[i] not in bodyBottom:
				keys[i] = [x * float(factor) for x in keys[i]]

		return keys


	