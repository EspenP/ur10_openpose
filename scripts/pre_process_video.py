#!/usr/bin/python

# This is both a script to pre process a video into discrete frames with data on where the hand is in the frame
# and a file that contains a class used for loading this data back into the current kernel in python. This is 
# to simulate a real time implementation of this project when the capability to run openpose in real time is not
# limited by hardware.

import cv2
import numpy as np
import pandas as pd
import os
import argparse
from pathlib import Path
from find_hand import openpose_hand_detection

NUM_FRAMES = 189

HOME_DIR = "/home/espen/"
ASSETS_DIR = HOME_DIR + "catkin_ws/assets/"

def test(frames, feed_name):
	print("Length of frames {}".format(len(frames)))
	for frame in frames:
		cv2.imshow(feed_name, frame)
		key = cv2.waitKey(1)
		if key == ('c'):
			break
 
	cv2.destroyAllWindows()

def display(img):
	cv2.imshow('test', img)
	cv2.waitKey(6)
	cv2.destroyAllWindows()

def grab_frames(num_frames, video_path):
	cap = cv2.VideoCapture(video_path) 
	frames = list()
	timestamps = [cap.get(cv2.CAP_PROP_POS_MSEC)]
	while(cap.isOpened()):
		ret, frame = cap.read()
		if not ret:
			break
		frames.append(frame)
		timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC))
	cap.release()

	if num_frames == 1:
		return [frames[0]]

	step = float(len(frames)) / float(num_frames)
	step = int(np.round(step))
	indices = np.array([i for i in range(len(frames))])
	indices = indices[0:-1:step]
	print('Selecting frames of this index: ', indices)
	result = list()
	res_timestamps = list()
	for index in indices:
		result.append(frames[index])
		res_timestamps.append(timestamps[index])

	return result, res_timestamps


class saved_data:
	def __init__(self):
		self.assets_dir = HOME_DIR + "catkin_ws/assets"
		self.frames = list()
		self.frame_centerpoints = dict()
		self.timestamps = list()

	# Load saved data from a pre_processed video
	def load(self, frame_folder): 
		save_dir = os.path.join(self.assets_dir, "frames", frame_folder)
		df = pd.read_csv(os.path.join(save_dir, "centerpoints.csv"))
		self.frame_centerpoints['X'] = df['X'].values
		self.frame_centerpoints['Y'] = df['Y'].values
		self.frame_centerpoints['Z'] = df['Z'].values
		self.frame_centerpoints['timestamps'] = df['timestamps'].values

		i = 0
		img_path = os.path.join(save_dir, "frame{}.png".format(str(i)))
		while os.path.exists(img_path):
			img = cv2.imread(img_path)
			self.frames.append(img)
			i += 1
			img_path = os.path.join(save_dir, "frame{}.png".format(str(i)))


	def get_frames(self):
		return self.frames

	def get_centerpoints(self):
		return self.frame_centerpoints

	def get_timestamps(self):
		return self.frame_centerpoints['timestamps']


	# Save data from a pre_processed video
	def save_frame(self, frame, save_dir, name):
		
		# Create path and file
		print(save_dir)
		if not os.path.exists(save_dir):
			os.makedirs(save_dir)
		Path(save_dir).touch()

		# Save to directory


		# Save images to directory
		cv2.imwrite(os.path.join(save_dir, name), frame)

		print("Data from frame successfull saved in ", save_dir)


	def save_frame_data(self, centerpoints, timestamps, save_dir):
		array_sz = len(centerpoints)
		self.frame_centerpoints['X'] = centerpoints[:, 0]
		self.frame_centerpoints['Y'] = centerpoints[:, 1]
		self.frame_centerpoints['Z'] = centerpoints[:, 2]
		self.frame_centerpoints['timestamps'] = np.array(timestamps[:array_sz])

		df = pd.DataFrame(self.frame_centerpoints)

		filename = "centerpoints.csv"
		if not os.path.exists(save_dir):
			os.makedirs(save_dir)
		Path(os.path.join(save_dir, filename)).touch()

		df.to_csv(os.path.join(save_dir, filename), index=False)
		print("Centerpoints and timestamps saved successfully to ", save_dir)



def main():
	# Parse Args
	parser = argparse.ArgumentParser(description="A script to save frames and hand centerpoints from a video of a hand")
	parser.add_argument('path', type=str, action='store', help='Path to video file')
	parser.add_argument('num_frames', type=int, action='store', help='However many frames you wish to extract')
	args = parser.parse_args()
	NUM_FRAMES = args.num_frames
	path_to_video = args.path
	print(NUM_FRAMES, path_to_video)

	save_dir = os.path.join(ASSETS_DIR, "frames", str(NUM_FRAMES) + "_frame_reduction")
	frames = list()
	timestamps = list()
	frames, timestamps = grab_frames(NUM_FRAMES, path_to_video)
	processor = openpose_hand_detection()
	archiver = saved_data()
	for i in range(len(frames)):
		image = cv2.rotate(frames[i], cv2.ROTATE_90_COUNTERCLOCKWISE) # Change this so your images look the way you want them to.
		try:														# In my case, opencv took them from the video rotates weirdly
			processor.process_image(image)
		except Exception as e:
			print(e)

		frame = processor.get_last_image()
		name  = "frame" + str(i) + ".png"
		archiver.save_frame(frame, save_dir, name)
	cps = processor.get_center_points()
	archiver.save_frame_data(cps, timestamps, save_dir)

if __name__ == '__main__':
	main()
