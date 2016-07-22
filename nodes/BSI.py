#!/usr/bin/env python
#coding: utf-8

#########################################################################################
## behavioral style input: contains the behavior that needs to be shown by the robot
## this node communicates with the style_filter node and is used as an input to know 
## the modifications that need to be bring to the style_filter node 
#########################################################################################

import threading


# possible states for the moment:
# normal, sad, happy, impatient
NB_SECOND_IMPATIENCE = 40

class BSI():

	def __init__(self):
		self.mood = "normal"
		self.timeSinceLastInteraction = 0

	def set_mood(self, new_mood):
		self.mood = new_mood

	def get_mood(self):
		return self.mood

	def update_mood(self, time):

		#@TODO virer ca a tout prix le plus vite possible
		self.timeSinceLastInteraction += 0.1

		if self.timeSinceLastInteraction > NB_SECOND_IMPATIENCE:
			self.mood = "impatient"

	def init_time_interaction(self):
		self.timeSinceLastInteraction = 0
		self.mood = "normal"
