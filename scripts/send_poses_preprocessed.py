#!/usr/bin/python

from pre_process_video import saved_data
import numpy as np
import argparse
import rospy
import cv2
from std_msgs.msg import Bool, Float64MultiArray

# Should be whatever folder under ~/catkin_ws/src/assets/frames/ you want to load
# The image data from to perform an experiment
class pose_scaler:
    def __init__(self, data_folder):
        data = saved_data()
        data.load(data_folder)
        self.frames = data.get_frames()
        self.cps = data.get_centerpoints()

        # Robot Workspace Defs
        # We will assume the workspace is a half-cube cube, for now
        # Robot will not be able to move in -Z direction, and should not
        self.ws_sz = 0.5 # in meters

        # Initialize frame size
        frame = self.frames[0]
        print("SHAPE: {}".format(frame.shape))
        self.x_pix, self.y_pix, c = frame.shape
        self.neutral_y = 0.0 
        self.neutral_z = -0.0
        self.neutral_x = 0.0



    def scale_points(self):
        # Y will actually be our in and out coordinate here.
        # X will be left and right
        # Z will be the height of the end effector from the ground
        # TODO fix this logic in pre process so that it is consistent. For now,
        # Y and Z just get switched.
        X = self.cps['X']
        Y = self.cps['Z']
        Z = self.cps['Y']
        scaled_points = list()
        for i in range(len(X)):
            # Scale X
            x = X[i]
            n_x = x / float(self.x_pix)
            p_x = n_x * self.ws_sz * 2  + self.neutral_x

            # Scale Z
            z = Z[i]
            n_z = z / float(self.y_pix)
            p_z = n_z * self.ws_sz * 2 + self.neutral_z

            # Scale Y
            p_y = Y[i] # The value of z is already a percentage value.
            p_y *= -(self.ws_sz + self.neutral_y) # We just need to decide where the neutral position is

            scaled_points.append([p_x, p_y, p_z])
        return scaled_points


class TX:
    def __init__(self, pub):
        self.is_requested = False
        self.pub = pub

    def publish(self, point):
        msg = Float64MultiArray()
        # msg.dim.label = rospy.Time.now()
        msg.data = point
        self.pub.publish(msg)
        self.is_requested = False

    def callback(self, msg):
        self.is_requested = msg.data

    def get_is_requested(self):
        return self.is_requested

def hook():
    print("\nShutting down node...\n")

def display(name, image):
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    # Parse args
    parser = argparse.ArgumentParser(description="A script to save frames and hand centerpoints from a video of a hand")
    parser.add_argument('-n', '--num_frames', 
                        required=True,
                        type=int, 
                        help='How many frames you want to process. You must have already pre processed exactly this many frames')
    parser.add_argument('-p', '--pause_bt_frames',
                        action='store_true',
                        help='Pass this argument if you wish to have control of how long you stay at each frame. \
                        for each picture displayed, just press any key to advance to the next frame and observe \
                        how the robot moves from frame to frame.')

    parser.add_argument('-rt', '--real_time',
                        action='store_true',
                        help='This argument will cause the script the script to take into account the timestamps from each frame, \
                        and delay for the appropriate amount of time before sending the next message. ')

    args = parser.parse_args()
    pause = args.pause_bt_frames
    num_frames = args.num_frames
    real_time = args.real_time


    rospy.init_node('pose_publisher')
    rate = rospy.Rate(10) # Double the rate of the request rate
    rospy.on_shutdown(hook)

    data_pub = rospy.Publisher('openpose/points', Float64MultiArray, queue_size=10)

    tx = TX(data_pub)
    request_sub = rospy.Subscriber('/request', Bool, tx.callback)

    data_folder = str(num_frames) + '_frame_reduction'
    scaler = pose_scaler(data_folder)
    points = scaler.scale_points()

    i = 0
    data = saved_data()
    data.load(data_folder)
    frames = data.get_frames()
    timestamps = data.get_timestamps()
    while not rospy.is_shutdown() and i < len(points):
        if tx.get_is_requested():
            tx.publish(points[i])
            if pause:                               
                display('frame' + str(i), frames[i]) # If the user hit pause, we will show each image until they  

            # If the user chose real time, we want to wait the appropriate amount of time before sending the next one.
            if real_time and i < len(points) - 1:
                rospy.sleep((timestamps[i+1] - timestamps[i])/1000.0)

            i += 1

        rate.sleep()
    print("Done")
    rospy.spin()


if __name__ == '__main__':
    main()







