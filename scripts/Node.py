#!/usr/bin/env python

import openravepy

class Node:

	def __init__(self, env):
		self.env = env
		self.trajectory = openravepy.RaveCreateTrajectory(self.env, '')

	def addTrajectory(self, trajectory):
		self.trajectory = False

