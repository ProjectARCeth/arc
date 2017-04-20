#!/usr/bin/env python
from math import sqrt

import rospy
import tf
from geometry_msgs.msg import Quaternion
from visensor_msgs.msg import visensor_imu
		
beta_gain = 0.1
sample_time = 0.01

def readOutVectorMsg(vector, normalize=True):
	x,y,z = vector.x, vector.y, vector.z
	if(normalize): norm = sqrt(x*x + y*y + z*z)
	else: norm = 1.0 
	return x/norm, y/norm, z/norm


class IMUOrientation:
	def __init__(self):
		#Init orientation.
		self.quat = [0,0,0,1]
		#Init ROS.
		rospy.init_node('imu_orientation')
		self.quat_pub = rospy.Publisher('imu_orientation', Quaternion, queue_size=10)
		rospy.Subscriber('cust_imu0', visensor_imu, self.imuCallback, queue_size=10)

	def imuCallback(self, data):
		#Update and normalize measurements.
		gx,gy,gz = readOutVectorMsg(data.angular_velocity, normalize=False)
		ax,ay,az = readOutVectorMsg(data.linear_acceleration)
		mx,my,mz = readOutVectorMsg(data.magnetometer)
		#Convert gyroscope degrees/sec to radians/sec
		gx *= 0.0174533
		gy *= 0.0174533
		gz *= 0.0174533
		#Get current quaternion.
		q0 = self.quat[0]; q1 = self.quat[1]; q2 = self.quat[2]; q3 = self.quat[3]
		#Rate of quaternion change from gyro.
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
		#Define auxiliary variables.
		_2q0mx = 2.0 * q0 * mx
		_2q0my = 2.0 * q0 * my
		_2q0mz = 2.0 * q0 * mz
		_2q1mx = 2.0 * q1 * mx
		_2q0 = 2.0 * q0
		_2q1 = 2.0 * q1
		_2q2 = 2.0 * q2
		_2q3 = 2.0 * q3
		_2q0q2 = 2.0 * q0 * q2
		_2q2q3 = 2.0 * q2 * q3
		q0q0 = q0 * q0
		q0q1 = q0 * q1
		q0q2 = q0 * q2
		q0q3 = q0 * q3
		q1q1 = q1 * q1
		q1q2 = q1 * q2
		q1q3 = q1 * q3
		q2q2 = q2 * q2
		q2q3 = q2 * q3
		q3q3 = q3 * q3
		#Reference direction of Earth's magnetic field.
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
		_2bx = sqrt(hx * hx + hy * hy)
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
		_4bx = 2.0 * _2bx
		_4bz = 2.0 * _2bz
		#Gradient decent algorithm corrective step.
		s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
		s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
		s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
		s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
		#Normalising.
		recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm
		#Apply feedback step.
		qDot1 -= beta_gain * s0
		qDot2 -= beta_gain * s1
		qDot3 -= beta_gain * s2
		qDot4 -= beta_gain * s3
		#Integrate rate of change of quaternion to yield quaternion.
		q0 += qDot1 * sample_time
		q1 += qDot2 * sample_time
		q2 += qDot3 * sample_time
		q3 += qDot4 * sample_time
		#Normalise quaternion
		recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
		self.quat[0] = q0*recipNorm
		self.quat[1] = q1*recipNorm
		self.quat[2] = q2*recipNorm
		self.quat[3] = q3*recipNorm
		#Publishing.
		quat_msg = Quaternion()
		quat_msg.x = self.quat[0]
		quat_msg.y = self.quat[1]
		quat_msg.z = self.quat[2]
		quat_msg.w = self.quat[3]
		self.quat_pub.publish(quat_msg)
		#Tf tree visualisation.
		br = tf.TransformBroadcaster()
		br.sendTransform((0,0,0), (self.quat[0],self.quat[1],self.quat[2],self.quat[3]), 
						 rospy.Time.now(), "imu", "world")


if __name__ == '__main__':
	imu = IMUOrientation()
	rospy.spin()
		