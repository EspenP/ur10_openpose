#!/usr/bin/python

import sys
sys.path.append('/usr/local/python')
from openpose import pyopenpose as op
from os.path import join
import cv2
import numpy as np

# Please change this to your home dir
HOME_DIR = '/home/espen'

class openpose_hand_detection:
	def __init__(self):
		self.params = dict()
		self.params['model_folder'] = join(HOME_DIR, "openpose", "models")
		self.params['hand'] = True
	
		self.initial_frame = True
		self.initial_hand_size = 0.0

		self.image = None
		self.cps = list()

	# This is the only function that needs to be called besides the getters
	def process_image(self, image):
		opWrapper = op.WrapperPython()
		opWrapper.configure(self.params)
		opWrapper.start()
		datum = op.Datum()
		datum.cvInputData = image
		opWrapper.emplaceAndPop(op.VectorDatum([datum]))
		print("DATUM: ", datum.handKeypoints)
		kp = datum.handKeypoints[0] # Gets left hand key point, reduces to 2D array
		cp = self.calc_centerpoint(kp)
		img = datum.cvOutputData

		self.cps.append(cp)
		self.image = img

	def get_last_image(self):
		return self.image

	def get_center_points(self):
		return np.array(self.cps)

	def calc_centerpoint(self, key_points, min_confidence=0.50):
		kp = key_points[0]
		X = list()
		Y = list()
		for point in kp:
			if point[2] >= min_confidence:
				X.append(point[0])
				Y.append(point[1])
		X = np.array(X)
		Y = np.array(Y) 
		
		Xmin = np.min(X)
		Xmax = np.max(X)
		Ymax = np.max(Y)
		Ymin = np.min(Y)
		
		hand_size = float(Xmax - Xmin) * float(Ymax - Ymin)
		
		meanX = np.mean(X)
		meanY = np.mean(Y)
		
		# Store the first hand size and use it as a calibration for how far the hand moves
		if self.initial_frame:
			self.initial_frame = False
			self.initial_hand_size = hand_size
		
		relZ = hand_size / self.initial_hand_size
		
		return [meanX, meanY, relZ]


		