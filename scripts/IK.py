#!/usr/bin/python

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, Bool, Float64MultiArray
from kinematics import inv_kin


class RX:
	def __init__(self, request_pub):
		self.data_req = request_pub
		self.req_msg = Bool()
		self.req_msg.data = True
		self.rx_buf = None
		self.data_ready = False

	def get_data(self):
		self.data_ready = False
		return self.rx_buf

	def callback(self, msg):
		self.rx_buf = msg
		self.data_ready = True

	def is_ready(self):
		return self.data_ready

	# Requests data from the image data node
	def request_data(self):
		self.data_req.publish(self.req_msg)

def move_to_home_position(pub):
    Tmsg = JointTrajectory()
    # Tmsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
    Tmsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
    Tmsg.header.stamp = rospy.Time.now()

    Tpointmsg = JointTrajectoryPoint()
    Tpointmsg.velocities = [0,0,0,0,0,0]
    Tpointmsg.time_from_start = rospy.Duration(0.5)

    Tpointmsg.positions = [-np.pi/2,-np.pi/2,np.pi/2,0,0,0]
    Tmsg.points = [Tpointmsg]
    # print "publishing"
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joint_states.position = [round(num, 3) for num in joint_states.position]
    while (joint_states.position[0:3] != [round(np.pi/2,3),round(-np.pi/2,3),round(-np.pi/2,3)]):
        # print joint_states.position[0:3]
        pub.publish(Tmsg)
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joint_states.position = [round(num, 3) for num in joint_states.position]
    rospy.sleep(1)


def move_hand_to_point(point, pub):
	Tmsg = JointTrajectory()
	Tmsg.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
	Tmsg.header.stamp = rospy.Time.now()

	Tpointmsg = JointTrajectoryPoint()
	Tpointmsg.velocities = [0,0,0,0,0,0]
	Tpointmsg.time_from_start = rospy.Duration(0.5)

	desired_solution = [0,0,0,-np.pi/2, 0, 0]
	target_pose = [point[0], point[1], point[2], -np.pi/2, 0, 0]
	Tpointmsg.positions = inv_kin(target_pose, desired_solution)
	Tmsg.points = [Tpointmsg]
	pub.publish(Tmsg)

def main():
	rospy.init_node('IK')
	rate = rospy.Rate(100) # Check for new data at 5 Hz
	arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	request_pub = rospy.Publisher('/request', Bool, queue_size=1)
	rx = RX(request_pub)
	data_sub = rospy.Subscriber('/openpose/points', Float64MultiArray, rx.callback)

	move_to_home_position(arm_pub)
	# MAIN LOOP
	while not rospy.is_shutdown():
		rx.request_data()
		rate.sleep()
		if rx.is_ready():
			point =  rx.get_data()
			move_hand_to_point(point.data, arm_pub)

if __name__ == "__main__":
	main()