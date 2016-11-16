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
import animations.spaceShuttle_pose
import animations.monster_pose
import animations.yum_pose
import animations.robot_pose
import animations.thinkingCard_pose
import animations.bendCard_pose
import animations.thinking1_pose
import animations.thinking2_pose
import animations.thinking3_pose
import animations.thinking4_pose
import animations.thinking5_pose
import animations.thinking6_pose
import animations.thinking7_pose
import animations.thinking8_pose
import animations.hesitation_pose
import animations.hesitation2_pose
import animations.hesitation3_pose
import animations.reject_pose
import animations.reject2_pose
import animations.reject3_pose
import animations.reject4_pose
import animations.reject5_pose
import animations.IdontKnow_pose
import animations.IdontKnow2_pose
import animations.introduction_pose

import random
import time
import copy
import datetime
import rospy
from BSI import *
from naoqi import ALProxy
from std_msgs.msg import String
from memory.msg import Animation


states = {
"low": {"normal": [7, 13, 14, 20, 22, 24, 35, 39, 40, 41], "happy": [10, 11, 18, 19], "sad": [23, 32, 33]}, 
"medium": {"normal": [52, 6, 8, 9, 1, 21, 5, 12, 26], "happy": [11, 18, 19], "sad": [25, 29, 30, 31]}, 
"high": {"normal": [1, 2, 4, 37, 38, 42], "happy": [15, 16, 17], "sad": [27, 28, 34, 36]},
"thinking": [43, 44, 45, 46, 47, 48, 49, 50, 51], #45, 51 removed due to the robot standing up too quickly
"quotes": {"confident": ["A moi", "c'est mon tour", "Attend, je vais te montrer !", "Facile !", "Regarde, c'est facile !", "Regarde ce qu'il faut faire !"], "neutral": ["A moi de jouer", "C'est mon tour", "Facile, Regarde"], "random": ["Pas facile", "Je ne sais pas quoi jouer", "Au hasard"], "success": ["Et bim!", "C'est qui le patron ?", "Yes!", "Un point de plus pour moi"], "defeat": ["Au prochain", "J'ai pas de chance", "Pas de bol!", "Au suivant"]},
"introduction": {1: "Salut. Je m'appelle clem. Est ce que tu veux jouer avec moi ?", 2: "Hello. Moi c'est mimi. On fait une partie ?"},
"celebration": ["J'ai gagné la partie", "La victoire est pour moi", "Je suis le meilleur"]
}
bodyBottom = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]
animation_return_card = 0

class NaoMotion():

	def __init__(self, NAO_IP, PORT, cBSI, idRobot):

		# create proxies
		self.motionProxy = ALProxy("ALMotion", NAO_IP, PORT)
		self.audioProxy = ALProxy("ALAudioPlayer", NAO_IP, PORT)
		self.postureProxy = ALProxy("ALRobotPosture", NAO_IP, PORT)
		self.faceDetectionProxy = ALProxy("ALBasicAwareness", NAO_IP, PORT)
		self.textSpeakProxy = ALProxy("ALTextToSpeech", NAO_IP, PORT)
		

		self.lastAnimation = 0
		self.lastQuote = ""

		# set language to speak as french
		self.textSpeakProxy.setLanguage("French")

		# publisher topics
		self.debug = rospy.Publisher("debug", String, queue_size=10)
		self.publisher_end_animation = rospy.Publisher("topic_end_animation", Animation, queue_size=10)

		# activate module anti collision
		self.motionProxy.setExternalCollisionProtectionEnabled("All", True)
		# activate default behavior nao
		self.breath(cBSI)
		self.faceFolowing(cBSI.faceTracking)
		# Robot is defined by a unique id
		self.idRobot = idRobot

	def runMotion(self, pose, factorSpeed = 1.0, factorAmpl = 1.0, audioFile = None, delayAudioInit = None, post = True):

		# get times, angles and names
		times = self.increaseSpeed(pose.times, factorSpeed)
		keys = self.increaseAmplitude(pose.keys, pose.names, factorAmpl)
		names = pose.names

		# launch animation
		if post == True:
			self.motionProxy.post.angleInterpolationBezier(names, times, keys)
		else:
			self.motionProxy.angleInterpolationBezier(names, times, keys)

		# launch audio file
		if audioFile != None:
			if delayAudioInit != None:
				time.sleep(delayAudioInit)

			self.audioProxy.post.playFile(audioFile)

	def launchAnimation(self, state, cBSI = None):

		# get factor speed and amplitude
		factorSpeed = 1.0
		factorAmpl = 1.0
		if cBSI != None:
			factorSpeed = cBSI.get_speed_animation()
			factorAmpl = cBSI.get_amplitude_animation()

		# save the starting time of the animation
		msg = Animation()
		msg.idRobot = self.idRobot
		msg.idAnimation = state
		msg.dateStart = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


		# launch an animations in function of random state
		if state == 0:
			# the state 0 is the animation made by the robot to return the card
			self.runMotion(animations.bendCard_pose, factorSpeed, factorAmpl, post = False)

		elif state == 1:
			self.runMotion(animations.stretch2_pose, factorSpeed, factorAmpl)

		elif state == 2:
			self.runMotion(animations.stretch3_pose, factorSpeed, factorAmpl)

		elif state == 4:
			self.runMotion(animations.helicopter_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/helicopter.wav")

		elif state == 5:
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
			self.runMotion(animations.showMuscles1_pose, factorSpeed, factorAmpl, post = False)
			
		elif state == 11:
			self.runMotion(animations.showMuscles2_pose, factorSpeed, factorAmpl, post = False)

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
			self.runMotion(animations.happy2_pose, factorSpeed, factorAmpl, post = False)

		elif state == 19:
			self.runMotion(animations.happy3_pose, factorSpeed, factorAmpl, post = False)

		elif state == 20:
			self.runMotion(animations.proud_pose, factorSpeed, factorAmpl)

		elif state == 21:
			self.runMotion(animations.relieved_pose, factorSpeed, factorAmpl)

		elif state == 22:
			self.runMotion(animations.alienated_pose, factorSpeed, factorAmpl)

		elif state == 23:
			self.runMotion(animations.disappointed_pose, factorSpeed, factorAmpl, post = False)

		elif state == 24:
			self.runMotion(animations.embarassed_pose, factorSpeed, factorAmpl)

		elif state == 25:
			self.runMotion(animations.exhausted_pose, factorSpeed, factorAmpl, post = False)

		elif state == 26:
			self.runMotion(animations.exhausted2_pose, factorSpeed, factorAmpl)

		elif state == 27:
			self.runMotion(animations.excited_pose, factorSpeed, factorAmpl)

		elif state == 28:
			self.runMotion(animations.excited2_pose, factorSpeed, factorAmpl)

		elif state == 29:
			self.runMotion(animations.stubborn_pose, factorSpeed, factorAmpl, post = False)

		elif state == 30:
			self.runMotion(animations.stubborn_pose, factorSpeed, factorAmpl, post = False)
			
		elif state == 31:
			self.runMotion(animations.desperate_pose, factorSpeed, factorAmpl, post = False)

		elif state == 32:
			self.runMotion(animations.desperate2_pose, factorSpeed, factorAmpl, post = False)

		elif state == 33:
			self.runMotion(animations.desperate3_pose, factorSpeed, factorAmpl, post = False)

		elif state == 34:
			self.runMotion(animations.shootGround_pose, factorSpeed, factorAmpl)

		elif state == 35:
			self.runMotion(animations.pensive_pose, factorSpeed, factorAmpl)

		elif state == 36:
			self.runMotion(animations.crying_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/cry.wav")

		elif state == 37:
			self.runMotion(animations.spaceShuttle_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/rocket.wav", 2)

		elif state == 38:
			self.runMotion(animations.monster_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/monsterGrowl.wav", 2)

		elif state == 39:
			self.runMotion(animations.thinking1_pose, factorSpeed, factorAmpl)

		elif state == 40:
			self.runMotion(animations.thinking2_pose, factorSpeed, factorAmpl)

		elif state == 41:
			self.runMotion(animations.yum_pose, factorSpeed, factorAmpl)

		elif state == 42:
			self.runMotion(animations.robot_pose, factorSpeed, factorAmpl, "/home/nao/audio/wav/r2d2.wav", 2)

		elif state == 43:
			self.runMotion(animations.thinking3_pose, factorSpeed, factorAmpl)

		elif state == 44:
			self.runMotion(animations.thinking4_pose, factorSpeed, factorAmpl)

		elif state == 45:
			self.runMotion(animations.thinking5_pose, factorSpeed, factorAmpl)

		elif state == 46:
			self.runMotion(animations.thinking6_pose, factorSpeed, factorAmpl)

		elif state == 47:
			self.runMotion(animations.thinking7_pose, factorSpeed, factorAmpl)

		elif state == 48:
			self.runMotion(animations.thinking8_pose, factorSpeed, factorAmpl)

		elif state == 49:
			self.runMotion(animations.hesitation_pose, factorSpeed, factorAmpl)

		elif state == 50:
			self.runMotion(animations.hesitation2_pose, factorSpeed, factorAmpl)

		elif state == 51:
			self.runMotion(animations.hesitation3_pose, factorSpeed, factorAmpl)

		elif state == 52:
			self.runMotion(animations.stretch1_pose, factorSpeed, factorAmpl)

		# wait until the animation ends and then, save the ending time and post that it to the topic
		while (self.isMoving()):
			rospy.sleep(0.1)

		msg.dateEnd = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		self.publisher_end_animation.publish(msg)

	def nonFunctionalMove(self, cBSI):

		# if the robot is not moving, launch animation
		if self.isMoving() == False:

			# control robot posture in order to avoid animation if the robot is not standing up
			self.control_posture()

			# get type of animation available for this scale
			levelsType = cBSI.get_mood()

			# get the mood that the robot should have
			allStates = []
			for levelType in levelsType:
				allStates.extend(copy.deepcopy(states[levelType]["normal"]))

			# remove last state in order to avoid same animation twice in a row
			if self.lastAnimation in allStates:
				allStates.remove(self.lastAnimation)

			#randomly select one state
			randomState = random.choice(allStates)

			# update last animation parameter
			self.lastAnimation = randomState

			# avoid eye traking and launch animation
			self.faceFolowing(False)
			self.breath(cBSI)
			self.launchAnimation(randomState, cBSI)
			self.faceFolowing(cBSI.faceTracking)
				
	def functionalMove(self, success, cBSI):

		# get type of animaiton available for this scale
		levelsType = cBSI.get_mood()
		allStatesSpeak = []
		allStatesAnimation = []

		emotion = ""
		if success == False:
			emotion = "sad"
			allStatesSpeak = states["quotes"]["defeat"]
		else:
			emotion = "happy"
			allStatesSpeak = states["quotes"]["success"]

		# get the mood that the robot should have
		for levelType in levelsType:
			allStatesAnimation.extend(states[levelType][emotion])

		# randomly select one random qupte
		randomStateSpeak = random.choice(allStatesSpeak)
		# make nao say it
		self.textSpeakProxy.post.say(randomStateSpeak)

		#randomly select one animation state
		randomStateAnimation = random.choice(allStatesAnimation)
		# launch animation
		self.launchAnimation(randomStateAnimation)

	def return_card(self, cBSI = None):

		# wait that the robot finish moving
		while self.isMoving() == True:
			rospy.sleep(0.1)

		# randomly select two differents thinking states
		allStates = states["thinking"]
		randomState = random.choice(allStates)

		# update last animation parameter
		self.lastAnimation = randomState

		# launch animations
		self.launchAnimation(randomState)
		self.launchAnimation(animation_return_card, cBSI)

		self.stand()

	def introduceNextRound(self, state):

		# get a random qupte that the robot will say
		allQuotes = copy.deepcopy(states["quotes"][state])
		
		# remove last quote cited by robot
		if self.lastQuote in allQuotes:
			allQuotes.remove(self.lastQuote)

		# get a random new quote
		randomQuote = random.choice(allQuotes)
		self.lastQuote = randomQuote

		# make NAO speak
		self.textSpeakProxy.post.say(randomQuote)

	def introduceHimself(self):

		# launch the animation corresponding to the salutation
		self.runMotion(animations.introduction_pose)

		# get the presentation sentence
		stringToSay = states["introduction"][self.idRobot]

		# NAO introduce itself
		self.textSpeakProxy.post.say(stringToSay)
		
	def celebrateVictory(self):

		# randomly select one state and launch it
		randomState = random.choice(states["high"]["happy"])
		self.launchAnimation(randomState)

		# wait that the robot finish moving and say something to the kid
		while self.isMoving() == True:
			rospy.sleep(0.1)

		# get one random sentence that NAO needs to say
		randomStr = random.choice(states["celebration"])
		self.textSpeakProxy.say(randomStr)

	def sit_down(self):
		# go to posture sit down
		self.postureProxy.goToPosture("Crouch", 0.5)

	def stand(self):

		while self.isMoving():
			rospy.sleep(0.1)

		# go to posture stand init 
		self.postureProxy.goToPosture("Stand", 0.45)

	def control_posture(self):

		# if the robot isn't in standing posture, go to it
		if self.postureProxy.getPosture() == "Crouch":
			self.stand()

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

	def isMoving(self):
		for task in self.motionProxy.getTaskList():
			if task[0] == 'angleInterpolationBezier':
				return True

		return False

	def breath(self, cBSI):

		if cBSI.breathEnable == True:
			# start breathing
			self.motionProxy.setBreathConfig([['Bpm', cBSI.bpm], ['Amplitude', cBSI.ampBreath]])

			if self.motionProxy.getBreathEnabled("Body") == False:
				self.motionProxy.setBreathEnabled("Body", True)

		else:
			# stop breathing
			self.motionProxy.setBreathEnabled("Body", False)

	def faceFolowing(self, state):

		if state == True:
			self.faceDetectionProxy.startAwareness()
		else:
			self.faceDetectionProxy.stopAwareness()

	def rest(self):

		self.motionProxy.setBreathEnabled("Body", False)
		self.faceDetectionProxy.stopAwareness()
		rospy.sleep(10)
		self.motionProxy.rest()
