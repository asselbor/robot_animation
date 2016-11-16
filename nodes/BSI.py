#!/usr/bin/env python
#coding: utf-8

#########################################################################################
## behavioral style input: contains the behavior that needs to be shown by the robot
## this node communicates with the style_filter node and is used as an input to know 
## the modifications that need to be bring to the style_filter node 
#########################################################################################

import math

class BSI():
	def __init__(self, parkinsonScale):

		self.parkinsonScale = parkinsonScale
		self.mood = list()
		self.speedAnimation = 1.0
		self.amplitudeAnimation = 1.0
		self.timeSinceLastInteraction = 0
		self.ampBreath = 1
		self.bpm = 30.0
		self.breathEnable = False
		self.periodAnimation = None
		self.faceTracking = True
		self.mood.append("low")

		# line equation of bpm/amp => bpm = 30 - 25*amp. amp [0, 1]; bpm [0, 30]
		if self.parkinsonScale == 2:
			self.mood.append("low")
			self.periodAnimation = 25
			self.breathEnable = True
			self.ampBreath = 0.8
			self.bpm = 10.0
		
		elif self.parkinsonScale == 3:
			self.mood.append("low")
			self.mood.append("medium")
			self.periodAnimation = 18
			self.breathEnable = True
			self.ampBreath = 0.6
			self.bpm = 15.0
		
		elif self.parkinsonScale == 4:
			self.mood.append("medium")
			self.periodAnimation = 12
			self.breathEnable = True
			self.ampBreath = 0.4
			self.bpm = 20.0	

	def get_speed_animation(self):
		return self.speedAnimation

	def get_amplitude_animation(self):
		return self.amplitudeAnimation

	def init_time_interaction(self):
		self.timeSinceLastInteraction = 0

	def get_mood(self):
		return self.mood