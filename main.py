#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, acos, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
import math

class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
		
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		self.w1 = 0
		self.w2 = 0
		self.w3 = 0
		self.w4 = 0

		self.iter = 0

		# PHYSICAL PARAMETERS
		self.m = 0.027 # mass in kg
		self.l = 0.046 # quadrotor length in meter 

		#inertia terms 
		self.I =1
		self.Ix = 16.571710 * (10**(-6))
		self.Iy = 16.571710 * (10**(-6))
		self.Iz = 29.261652 * (10**(-6))
		self.Ip = 12.65625 * (10**(-8))

		# propeller thrust and moment factors 
		self.kf = 1.28192 * (10**(-8))
		self.km = 5.964552 * (10**(-3))

		# TUNING PARAMETERS
		self.Kp = 40 
		self.Kd = 3

		self.lambda1 = 5	
		self.lambda2 = 13
		self.lambda3 = 16
		self.lambda4 = 65
		self.g = 9.81

		self.k1 = 6
		self.k2 = 142
		self.k3 = 142	
		self.k4 = 7

		self.t_count = 0

		self.s1 = 0
		self.s2 = 0
		self.s3 = 0
		self.s4 = 0


	def traj_evaluate(self):
		#evaluate the corresponding trajectories designed in Part 1to return the desired positions, velocities and accelerations

		# For p0(0, 0, 0) - p1(0, 0, 1) in 5 seconds
		if self.t <5:

			xd = 0
			xd_dot = 0
			xd_ddot = 0

			yd = 0
			yd_dot = 0
			yd_ddot = 0

			zd  = 0 + 0*self.t**1 + 0*self.t**2 + 2/25 * self.t **3 + -3/125*self.t **4 + 6/3125*self.t **5
			zd_dot = 0 + 0 * ((self.t)**1) + 3*(2/25)*((self.t)**2) + 4*(-3/125)*((self.t)**3) + 5*(6/3125)*self.t**4
			zd_ddot = 0 +  6*(2/25)* ((self.t)**1) + 12*(-3/125)*(self.t**2) + 20*(6/3125)*self.t**3 

		# For p1(0, 0, 1) - p2(1, 0, 1) in 15 seconds
		elif self.t>=5 and self.t<20:

			xd =   0*(self.t-5)**1 + 0*(self.t-5)**2 + 2/675 * (self.t-5) **3  -1/3375*(self.t-5) **4 + 2/253125*(self.t-5) **5
			xd_dot =  6/675*(self.t-5)**2 - 4/3375*(self.t-5)**3 + 10/253125*(self.t-5)**4
			xd_ddot =  12/675*(self.t-5) - 12/3375*(self.t-5)**2 + 40/253125*(self.t-5)**3

			yd = 0
			yd_dot = 0
			yd_ddot = 0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		# For p2(1, 0, 1) - p3(1, 1, 1) in 15 seconds
		elif self.t>=20 and self.t<35:

			xd = 1
			xd_dot = 0
			xd_ddot = 0

			yd = 0 + 0*(self.t-20)**1 + 0*(self.t-20)**2 + 2/675 * (self.t-20) **3  -1/3375*(self.t-20) **4 + 2/253125*(self.t-20) **5
			yd_dot = 6/675*(self.t-20)**2 - 4/3375*(self.t-20)**3 + 10/253125*(self.t-20)**4
			yd_ddot = 12/675*(self.t-20) - 12/3375*(self.t-20)**2 + 40/253125*(self.t-20)**3

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		# For p3(1, 1, 1) - p4(0, 1, 1) in 15 seconds
		elif self.t>=35 and self.t < 50:

			xd = 1 + 0*(self.t-35)**1 + 0*(self.t-35)**2 - 2/675 * (self.t-35) **3  +1/3375*(self.t-35) **4 - 2/253125*(self.t-35) **5
			xd_dot = -6/675*(self.t-35)**2 + 4/3375*(self.t-35)**3 - 10/253125*(self.t-35)**4
			xd_ddot =  -12/675*(self.t-35) + 12/3375*(self.t-35)**2 - 40/253125*(self.t-35)**3

			yd = 1
			yd_dot = 0
			yd_ddot = 0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		# For p4(0, 1, 1) - p5(0, 0, 1) in 15 seconds
		elif self.t>=50 and self.t < 65:

			xd = 0
			xd_dot =0
			xd_ddot = 0

			yd = 1 + 0*(self.t-50)**1 + 0*(self.t-50)**2 - 2/675 * (self.t-50) **3  +1/3375*(self.t-50) **4 - 2/253125*(self.t-50) **5
			yd_dot =  -6/675*(self.t-50)**2 + 4/3375*(self.t-50)**3 - 10/253125*(self.t-50)**4
			yd_ddot =  -12/675*(self.t-50) + 12/3375*(self.t-50)**2 - 40/253125*(self.t-50)**3

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		else:

			xd = 0
			xd_dot =0
			xd_ddot = 0

			yd = 0
			yd_dot =  0
			yd_ddot =  0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		return (xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot)

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):

		self.iter+=1
		# obtain the desired values by evaluating the corresponding trajectories

		# state space representation of the system 

		x1 = xyz[2,0] # z 
		x2 = rpy[0,0] # roll
		x3 = rpy[1,0] # pitch
		x4 = rpy[2,0] # yaw

		# print(x2,x3,x4);

		x5 = xyz_dot[2,0] # z_dot
		x6 = rpy_dot[0,0] # roll velocity
		x7 = rpy_dot[1,0] # pitch velocity 
		x8 = rpy_dot[2,0] # yaw velocity

		
		xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()

		# Implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"

		## ----------------- U1 -----------------

		self.s1 = (x5-zd_dot) + self.lambda1*(x1-zd)

		u1 = self.m*(self.g + zd_ddot - self.lambda1*(x5 - zd_dot) - self.k1*self.sat(self.s1))/(cos(x2)*cos(x3))

		den = u1

		Fx = self.m *(-self.Kp * (xyz[0,0]-xd) - self.Kd * (xyz_dot[0,0] - xd_dot) + xd_ddot)
		Fy = self.m *(-self.Kp * (xyz[1,0]-yd) - self.Kd * (xyz_dot[1,0] - yd_dot) + yd_ddot)

		if abs(u1<1e-5):
			u1 = 1e-5


		if abs(u1) < abs(Fx) or abs(u1)< abs(Fy):
			den = max(abs(Fx),abs(Fy))*((u1)/abs(u1)) + 1e-5  

		theta_d =  math.asin(Fx/den) # theta desired 
		PHI_d =  math.asin(-Fy/den) 
		psi_d = 0

		## ----------------- U2 -----------------

		self.s2 = (rpy_dot[0,0]) + self.lambda2*self.wrap_pipi((rpy[0,0])-(PHI_d))

		u2 = -(x7*x8*(self.Iy-self.Iz) - self.Ip*self.max_ohm()*x7 + self.lambda2*self.Ix*x6 + self.Ix*self.k2*self.sat1(self.s2))
		
		## ----------------- U3 -----------------
		
		self.s3 = (rpy_dot[1,0]) + self.lambda3*self.wrap_pipi((rpy[1,0])-(theta_d))

		u3 = -(x6*x8*(self.Iz-self.Ix) + self.Ip*self.max_ohm()*x6 + self.lambda3*self.Iy*x7+ self.Iy*self.k3*self.sat1(self.s3))
	
		## ----------------- U4 -----------------

		self.s4 = (rpy_dot[2,0]) + self.lambda4*self.wrap_pipi((rpy[2,0])-(psi_d))

		u4 = -(x6*x7*(self.Ix - self.Iy) + self.lambda4 * self.Iz * x8 + self.Iz*self.k4*self.sat2(self.s4))

		if self.iter%100 ==0:

			print("u1: {:.4f} u2: {:.4f} u3: {:.4f} u4: {:.4f}".format(u1, u2, u3, u4))

		if self.iter%100 == 0:
			print("theta: {:.4f}, thetad: {:,.4f}  theta-thetad {:.4f}".format(rpy[1,0], theta_d, rpy[1,0]-theta_d))
			print("phi: {:.4f}, tphidd: {:,.4f}  phi-phid {:.4f}".format(rpy[0,0], PHI_d, rpy[0,0]-PHI_d))
			print("xd {:.4f} x {:.4f} x-xd {:.4f}".format(xd, xyz[0,0], (xyz[0,0] - xd)))
			print("yd {:.4f} y {:.4f} y-yd {:.4f}".format(yd, xyz[1,0], (xyz[1,0] - yd)))
			print("zd {:.4f} z {:.4f} z-zd {:.4f}".format(zd, xyz[2,0], (xyz[2,0] - zd)))
			print("psid: {:.4f}, psi: {:,.4f}  psi-psid {:.4f}".format(psi_d, rpy[2,0], rpy[2,0]-psi_d))

		# convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"

		a1 = (1/(4*self.kf))
		b2 =  (1.4142/(4*self.kf*self.l))
		c3 = (1/(4*self.km*self.kf))

		## ----------------- W1 -----------------
		self.w1 = sqrt(abs(a1*u1 - b2*u2 - b2*u3 - c3*u4))

		# Setting max rotor speed
		if self.w1>2618:
			self.w1 = 2618
			
		## ----------------- W2 -----------------	
		self.w2 = sqrt(abs(a1*u1 - b2*u2 + b2*u3 + c3*u4))

		if self.w2>2618:
			self.w2 = 2618
		
		## ----------------- W3 -----------------
		self.w3 = sqrt( abs(a1*u1 + b2*u2 + b2*u3 - c3*u4) )

		if self.w3>2618:
			self.w3 = 2618

		## ----------------- W4 -----------------
		self.w4 = sqrt( abs(a1*u1 + b2*u2 - b2*u3 + c3*u4) )

		if self.w4>2618:
			self.w4 = 2618

		if self.iter%100 == 0:
			print("inputs: ", self.w1, self.w2, self.w3, self.w4)

		motor_vel = np.asarray([[self.w1], [self.w2], [self.w3], [self.w4]])

		res = True in (motor_vel_ > 2615 for motor_vel_ in motor_vel[:, 0])

		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0],motor_vel[2,0], motor_vel[3,0]]

		self.motor_speed_pub.publish(motor_speed)

	def wrap_pipi(self, angle):

		if angle > pi:
			return (angle - 2*(pi))
		elif angle < -pi:
			return (angle + 2*pi)
		else:
			return angle

	def max_ohm(self):
		return self.w1 + self.w3 - self.w2 - self.w4 

	def sat(self,s):
		phi = 0.1
	
		if abs(s) >= phi:
			return s/abs(s)
		else:
			return s/phi

	def sat1(self,s):
		phi = 0.75

		if abs(s) >= phi:
			return s/abs(s)
		else:
			return s/phi

	def sat2(self,s):
		phi = 0.6

		if abs(s) >= phi:
			return s/abs(s)
		else:
			return s/phi

	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0
		
		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])

		# print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
		
		q = msg.pose.pose.orientation
		
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		
		T[0:3, 3] = xyz[0:3, 0]
		
		R = T[0:3, 0:3]
		
		xyz_dot = np.dot(R, v_b)
		
		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
		
		rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
		
		rpy = np.expand_dims(rpy, axis=1)
		
		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])
		
		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path
		
		with open("/home/venkatesh/Documents/log.pkl", "wb") as fp:
		
			self.mutex_lock_on = True
		
			pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")