#!/usr/bin/env python

import rospy
import numpy as np
import sys
from numpy import inf
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

side_scanstartangle = 20 
side_scanrange      = 60          
front_scanrange     = 16
distancefromwall    = 0.4

x   = np.zeros((360))
s_d = 0
y_l = 0
y_r = 0

# PID parameters
kp = 4
kd = 450
ki = 0

k1 = kp + ki + kd
k2 = -kp - 2*kd
k3 = kp

robot_namespace = ""

def callback(data):
	global y_l, y_r, x, s_d, front_scanrange, side_scanstartangle, side_scanrange

	x  = list(data.ranges)
	for i in range(360):
		if x[i] == inf:
			x[i] = 7
		if x[i] == 0:
			x[i] = 6

	y_l = min(x[side_scanstartangle : side_scanstartangle + side_scanrange])
	y_r = min(x[360 - side_scanstartangle - side_scanrange : 360 - side_scanstartangle])
	s_d = min(min(x[0:int(front_scanrange/2)], x[int(360 - front_scanrange/2):360]))

def tmnt_controller():
	global k1, k2, k3, kp, kd, ki, s_d, x, y_r, y_l, distancefromwall, robot_namespace

	prev_PID_output = 0
	prev_error      = 0
	prev_prev_error = 0

	rospy.init_node(f'{robot_namespace}_wallfollowing', anonymous=True)

	# 
	cmd_vel_topic = f'/{robot_namespace}/cmd_vel' if robot_namespace else '/cmd_vel'
	scan_topic    = f'/{robot_namespace}/scan'    if robot_namespace else '/scan'

	velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
	scan_subscriber    = rospy.Subscriber(scan_topic, LaserScan, callback)
	rate               = rospy.Rate(10)

	while not rospy.is_shutdown():
		delta = distancefromwall - y_r

		PID_output  = kp * delta + kd * (delta - prev_error)

		prev_error      = delta
		prev_prev_error = prev_error
		prev_PID_output = PID_output

		angular_zvel = np.clip(PID_output, -1.2, 1.2)
		linear_vel   = np.clip((s_d - 0.35), -0.1, 0.4)

		print(f'[{robot_namespace}] distance from right wall: {int(y_r*100)} cm | front wall: {int(s_d*100)} cm')
		print(f'[{robot_namespace}] linear_vel={linear_vel:.2f} | angular_vel={angular_zvel:.2f}')
		rospy.loginfo('\n')

		vel_msg = Twist(Vector3(linear_vel, 0, 0), Vector3(0, 0, angular_zvel))
		velocity_publisher.publish(vel_msg)
		rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
	print(f'[{robot_namespace}] Turtlebot stopped')

if __name__ == '__main__':
	try:
		# 
		robot_namespace = rospy.get_param("~namespace", "")
		tmnt_controller()
	except rospy.ROSInterruptException:
		pass

