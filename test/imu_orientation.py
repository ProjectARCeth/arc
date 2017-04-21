#!/usr/bin/env python
from math import acos, asin, atan2, cos, sin, degrees, fmod, pi, sqrt

import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from visensor_msgs.msg import visensor_imu
        
beta_gain = 0.1

def readOutVectorMsg(vector, normalize=True):
    x,y,z = vector.x, vector.y, vector.z
    if(normalize): norm = sqrt(x*x + y*y + z*z)
    else: norm = 1.0 
    return x/norm, y/norm, z/norm

class Quat:
    def __init__(self,w,x,y,z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def mul(self, other):
        result = Quat(1,0,0,0)
        result.w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        result.x = self.x*other.w + self.w*other.x + self.y*other.z - self.z*other.y
        result.y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        result.z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return result

    def get(self):
        return self.w, self.x, self.y, self.z

    def getList(self):
        return [self.w, self.x, self.y, self.z]

    def inverse(self):
        result = Quat(1,0,0,0)
        norm_squared = self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z
        if(norm_squared == 0): norm_squared = 0.0000001
        result.w = self.w/norm_squared
        result.x = - self.x/norm_squared
        result.y = - self.y/norm_squared
        result.z = - self.z/norm_squared
        return result

    def conjugate(self):
        result = Quat(1,0,0,0)
        result.w = self.w
        result.x = - self.x
        result.y = - self.y
        result.z = - self.z
        return result

    def dot(self, other):
        result = self.w*other.w + self.x*other.x + self.y*other.y + self.z*other.z
        return result

    def norm(self):
        norm = sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        if(norm == 0): norm_squared = 0.0000001
        self.w /= norm
        self.x /= norm
        self.y /= norm
        self.z /= norm

    def to_euler(self):
        return tf.transformations.euler_from_quaternion([self.w, self.x, self.y, self.z])

    def minimum_distance(self, other):
        if(self.dot(other) < 0):
            other.w = - other.w
            other.x = - other.x
            other.y = - other.y
            other.z = - other.z
        return

    def slerp(self, other, weight):
        result = Quat(1,0,0,0)
        dot = self.dot(other)
        if(dot > 0.9995):
            result.w = self.w + (other.w - self.w)*weight
            result.x = self.x + (other.x - self.x)*weight
            result.y = self.y + (other.y - self.y)*weight
            result.z = self.z + (other.z - self.z)*weight
            return result.norm()
        if(dot > 1): dot = 1
        elif (dot < -1): dot = -1
        theta_0 = acos(dot)
        if(0.0 < theta_0 and theta_0 < pi/2): theta = theta_0*weight
        else: theta = (theta_0 - pi)*weight
        result.w = other.w - self.w*dot
        result.x = other.x - self.x*dot
        result.y = other.y - self.y*dot
        result.z = other.z - self.z*dot
        result.norm()
        result.w = self.w*cos(theta) + result.w*sin(theta)
        result.x = self.x*cos(theta) + result.x*sin(theta)
        result.y = self.y*cos(theta) + result.y*sin(theta)
        result.z = self.z*cos(theta) + result.z*sin(theta)
        return result

    def sendTransform(self, x,y,z, name, broadcaster):
        broadcaster.sendTransform((x,y,z), (self.x, self.y, self.z, self.w), rospy.Time.now(), name, "world")

    @property
    def w(self): 
        return self.w
    @property
    def x(self): 
        return self.x
    @property
    def y(self): 
        return self.y
    @property
    def z(self): 
        return self.z

    @w.setter
    def w(self,w): self.w = w
    @x.setter
    def x(self,x): self.x = x
    @y.setter
    def y(self,y): self.y = y
    @z.setter
    def z(self,z): self.z = z



class IMUOrientation:
    def __init__(self):
        #Init orientation.
        # self.mad = Quat(1,0,0,0)
        self.ard = Quat(1,0,0,0)
        #Init ROS.
        rospy.init_node('imu_orientation')
        self.quat_pub = rospy.Publisher('imu_orientation', Quaternion, queue_size=10)
        rospy.Subscriber('cust_imu0', visensor_imu, self.imuCallback, queue_size=10)
        rospy.Subscriber('rovio/odometry', Odometry, self.rovioCallback, queue_size=10)
        #Init clock.
        self.time = rospy.Time.now().to_sec()

    def imuCallback(self, data):
        #Update and normalize measurements.
        gx,gy,gz = readOutVectorMsg(data.angular_velocity, normalize=False)
        ax,ay,az = readOutVectorMsg(data.linear_acceleration)
        mx,my,mz = readOutVectorMsg(data.magnetometer)
        #Get timestep.
        timestep = data.header.stamp.to_sec() - self.time
        self.time = data.header.stamp.to_sec()
        #Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533
        #Madgewick.
        # self.mad = self.madgwick_estimation(gx, gy, gz, ax, ay, az, mx, my, mz, self.mad, timestep)
        #Arduino.
        self.ard = self.arduino_estimation(gx, gy, gz, ax, ay, az, mx, my, mz, self.ard, 0.005)
        #Tf tree visualisation.
        br = tf.TransformBroadcaster()
        # br.sendTransform((-1,0,0), ([self.mad.get()]), rospy.Time.now(), "mad", "world")
        self.ard.sendTransform(1,0,0,"ard", br)

    def rovioCallback(self, data):
        quat_msg = data.pose.pose.orientation
        rovio_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        rovio_euler = tf.transformations.euler_from_quaternion(rovio_quat)
        #Tf tree visualisation.
        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,0), (rovio_quat[0], rovio_quat[1], rovio_quat[2], rovio_quat[3]), 
                         rospy.Time.now(), "rovio", "world")

    def arduino_estimation(self, gx, gy, gz, ax, ay, az, mx, my, mz, quat, weight):
        #Calculate polar representation of accelerometer data
        acc_roll = atan2(az, ay)
        acc_mag = sqrt(az*az + ay*ay)
        acc_tilt = atan2(ax, acc_mag)
        acc_mag = sqrt(ax*ax + acc_mag*acc_mag)
        acc_mag *= 0.00390625
        #Calculate accelerometer quaternions.
        qar = Quat(cos(acc_roll*0.5), 0, sin(acc_roll*0.5), 0)
        qar_inv = qar.inverse()
        qat = Quat(cos(acc_tilt*0.5),sin(acc_tilt*0.5),0,0)
        qat_inv = qat.inverse()
        qa = qat.mul(qar)
        #Calculate polar representation of magnetometer data
        mag_roll = atan2(mz, my) - pi
        mag_mag = sqrt(mz*mz + my*my)
        mag_tilt = atan2(mx, mag_mag)
        #Calculate magnetometer quaternions.
        qmr = Quat(cos(mag_roll*0.5), 0, sin(mag_roll*0.5), 0)
        qmt = Quat(cos(mag_tilt*0.5), sin(mag_tilt*0.5), 0, 0)
        qm = qmt.mul(qmr)
        #Rotate the magnetometer quaternion.
        qm = qm.mul(qar_inv)
        #Extract azimuth.
        azimuth = atan2(qm.x, qm.y) + pi
        if (qm.w > 0.): azimuth += pi
        azimuth = fmod(azimuth, 2*pi) - pi
        #Replace qm with just azimuth.
        qm.w = cos(azimuth * 0.5)
        qm.x = 0
        qm.y = 0
        qm.z = sin(azimuth * 0.5)
        #Construct quaternion from combined accelerometer and magnetometer azimuth data.
        qam = qm.mul(qa) 
        #Construct quaternion from gyroscope axes.
        qg = Quat(cos((gx + gy + gz)*0.5), sin(gz*0.5), sin(gx*0.5), sin(gy*0.5))
        #Complementary filter:
        #integrate latest gyro quaternion with stored orientation.
        quat = quat.mul(qg)
        #SLERP between stored orientation and new accel + mag estimate.
        delayed = Quat(quat.w, quat.x, quat.y, quat.z) 
        orientation = qam.slerp(quat, weight)
        #Use the shortest distance from previous orientation.
        delayed.minimum_distance(quat)
        return quat

if __name__ == '__main__':
    imu = IMUOrientation()
    rospy.spin()
        



    # def madgwick_estimation(self, gx, gy, gz, ax, ay, az, mx, my, mz, quat, timestep):
    #     #Get current quaternion.
    #     q0, q1, q2, q3 = quat.get()
    #     #Rate of quaternion change from gyro.
    #     qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    #     qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    #     qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    #     qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    #     #Define auxiliary variables.
    #     _2q0mx = 2.0 * q0 * mx
    #     _2q0my = 2.0 * q0 * my
    #     _2q0mz = 2.0 * q0 * mz
    #     _2q1mx = 2.0 * q1 * mx
    #     _2q0 = 2.0 * q0
    #     _2q1 = 2.0 * q1
    #     _2q2 = 2.0 * q2
    #     _2q3 = 2.0 * q3
    #     _2q0q2 = 2.0 * q0 * q2
    #     _2q2q3 = 2.0 * q2 * q3
    #     q0q0 = q0 * q0
    #     q0q1 = q0 * q1
    #     q0q2 = q0 * q2
    #     q0q3 = q0 * q3
    #     q1q1 = q1 * q1
    #     q1q2 = q1 * q2
    #     q1q3 = q1 * q3
    #     q2q2 = q2 * q2
    #     q2q3 = q2 * q3
    #     q3q3 = q3 * q3
    #     #Reference direction of Earth's magnetic field.
    #     hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
    #     hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
    #     _2bx = sqrt(hx * hx + hy * hy)
    #     _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
    #     _4bx = 2.0 * _2bx
    #     _4bz = 2.0 * _2bz
    #     #Gradient decent algorithm corrective step.
    #     s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    #     s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    #     s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    #     s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    #     #Normalising.
    #     recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
    #     s0 *= recipNorm
    #     s1 *= recipNorm
    #     s2 *= recipNorm
    #     s3 *= recipNorm
    #     #Apply feedback step.
    #     qDot1 -= beta_gain * s0
    #     qDot2 -= beta_gain * s1
    #     qDot3 -= beta_gain * s2
    #     qDot4 -= beta_gain * s3
    #     #Integrate rate of change of quaternion to yield quaternion.
    #     q0 += qDot1 * timestep
    #     q1 += qDot2 * timestep
    #     q2 += qDot3 * timestep
    #     q3 += qDot4 * timestep
    #     #Normalise quaternion
    #     recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    #     quat.w = q0*recipNorm
    #     quat.x = q1*recipNorm
    #     quat.y = q2*recipNorm
    #     quat.z = q3*recipNorm
    #     return quat