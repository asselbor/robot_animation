#!/usr/bin/env python
#coding: utf-8

#########################################################################################
## behavioral style input: contains the behavior that needs to be shown by the robot
## this node communicates with the style_filter node and is used as an input to know 
## the modifications that need to be bring to the style_filter node 
#########################################################################################

import threading
import math


# possible states for the moment:
# normal, sad, happy, impatient
NB_SECOND_IMPATIENCE = 80
NB_SECOND_TIRED = 160
NB_SECOND_SLEEP = 240

class BSI():

	def __init__(self, periodAnimationInit, periodAnimationMax):
		self.mood = "normal"
		self.timeSinceLastInteraction = 0
		self.periodAnimationInit = periodAnimationInit
		self.periodAnimation = periodAnimationInit
		self.periodAnimationMax = periodAnimationMax


	def set_mood(self, new_mood):
		self.mood = new_mood

	def get_mood(self):
		return self.mood

	def update_mood(self, time_now, time_previous):
		if time_previous != None:
			self.timeSinceLastInteraction += (time_now - time_previous).to_sec()
			
			# calculate the new period of animation
			self.updateFrequencyAnimation()

		if self.timeSinceLastInteraction > NB_SECOND_IMPATIENCE:
			self.mood = "impatient"

		if self.timeSinceLastInteraction > NB_SECOND_TIRED:
			self.mood = "tired"

		if self.timeSinceLastInteraction > NB_SECOND_SLEEP:
			self.mood = "sleep"

	def init_time_interaction(self):
		self.timeSinceLastInteraction = 0
		self.mood = "normal"

	def updateFrequencyAnimation(self):

		# slope of period decrease
		paramSpeed = 0.005

		# calculate the new period of animation
		self.periodAnimation = self.periodAnimationMax/(1 + (self.periodAnimationMax/float(self.periodAnimationInit) - 1)*math.exp(-paramSpeed*self.timeSinceLastInteraction))