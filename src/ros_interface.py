from PyQt4 import QtCore
import math
import numpy as np

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray

class ROSInterface(QtCore.QObject):

	info_signal = QtCore.pyqtSignal(list)
	programm_signal = QtCore.pyqtSignal(list)
	repeat_path_signal = QtCore.pyqtSignal(list)
	teach_path_signal = QtCore.pyqtSignal(list)

	def __init__(self, init_mode, name):
		QtCore.QObject.__init__(self)
		#Getting parameter.
		self.init_mode = init_mode
		#Init ROS.
		rospy.init_node('gui')
		#Get yaml parameter.
		k1_s = rospy.get_param("/control/K1_LAD_S")
		k2_s = rospy.get_param("/control/K2_LAD_S")
		k1_v = rospy.get_param("/control/K1_LAD_V")
		k2_v = rospy.get_param("/control/K2_LAD_V")
		#Define publisher and subscriber.
		self.gui_info_pub = rospy.Publisher('gui/commands',Int32MultiArray,queue_size=10)
		self.gui_info_sub = rospy.Subscriber('gui/data', Float64MultiArray,self.dataCallback,queue_size=10)
		self.programms_sub = rospy.Subscriber('programms',Int32MultiArray,self.programmCallback,queue_size=10) 
		self.repeat_path_sub = rospy.Subscriber('path', Path,self.pathCallback,queue_size=10) 
		self.teach_path_sub = rospy.Subscriber('teach_path', Path,self.teachCallback,queue_size=10)

	def publishInfo(self, info_list):
		info = Int32MultiArray()
		info.data.insert(1,info_list[0])
		info.data.insert(2,info_list[1])
		info.data.insert(3,info_list[2])
		self.gui_info_pub.publish(info)

	def dataCallback(self, msg):
		data = []
		for element in msg.data: data.append(round(element,3))
		self.info_signal.emit(data)

	def pathCallback(self, msg):
		index = len(msg.poses)-1
		if(index == -1) return
		lastest_pose = [-msg.poses[index].pose.position.y, msg.poses[index].pose.position.x]
		self.repeat_path_signal.emit(lastest_pose)

	def programmCallback(self, msg):
		data = []
		for element in msg.data: data.append(element)
		self.programm_signal.emit(data)

	def teachCallback(self, msg):
		teach_path = np.zeros((1,2))
		for element in msg.poses:
			path_element = np.array([-element.pose.position.y, element.pose.position.x])
			teach_path = np.vstack([teach_path, path_element])
		self.teach_path_signal.emit(teach_path.tolist())