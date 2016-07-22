#!/usr/bin/env python
#coding: utf-8

#########################################################################################
## behavioral style input: contains the behavior that needs to be shown by the robot
## this node communicates with the style_filter node and is used as an input to know 
## the modifications that need to be bring to the style_filter node 
#########################################################################################

class BSI():

	def __init__(self):
		self.mood = "normal"

	def set_mood(self, new_mood):
		self.mood = new_mood

	def get_mood(self):
		return self.mood