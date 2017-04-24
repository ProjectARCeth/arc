#!/usr/bin/env python
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import math
import numpy as np
import os
import sys

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray

def getTwoArrays(array):
	x = []
	y = []
	for i in range(0, len(array)):
		x.append(array[i][0])
		y.append(array[i][1])
	return x,y

#Init ros.
rospy.init_node('pdf_tracking')
#General constants.
k1_s = rospy.get_param("/control/K1_LAD_S")
k2_s = rospy.get_param("/control/K2_LAD_S")
k1_v = rospy.get_param("/control/K1_LAD_V")
k2_v = rospy.get_param("/control/K2_LAD_V")

class PDFTracker:
	def __init__(self, name):
		#Setting file_name.
		self.file_name = name
		# Init lists.
		self.ready = False
		self.distance_start = []
		self.velocity = []
		self.velocity_should = []
		self.velocity_teach = []
		self.steering = []
		self.steering_should = []
		self.tracking_error = []
		self.repeat_path = np.zeros((1,2))
		self.teach_path = np.zeros((1,2))
		#Init Subscriber.
		self.gui_commands_pub = rospy.Subscriber('gui/commands',Int32MultiArray,self.commandsCallback,queue_size=10)
		self.gui_info_sub = rospy.Subscriber('gui/data', Float64MultiArray,self.dataCallback,queue_size=10)
		self.repeat_path_sub = rospy.Subscriber('path', Path,self.repeatCallback,queue_size=10) 
		self.teach_path_sub = rospy.Subscriber('teach_path', Path,self.teachCallback,queue_size=10)

	def createPDF(self):
		#Create plots.
		fig = plt.figure(figsize=(10, 7))
		gs = gridspec.GridSpec(5, 4)
		gs.update(hspace=0.4)

		ax0 = plt.subplot(gs[0, :4])
		plt.title("Path Analysis")
		plt.plot(self.tracking_error)
		plt.ylabel('tracking_error[m]')

		ax1 = plt.subplot(gs[1, :4])
		plt.plot(self.velocity, 'b', label="repeat")
		plt.plot(self.velocity_teach, 'g', label="teach")
		plt.plot(self.velocity_should, 'r', label="should")
		plt.ylabel('velocity[km/h]')

		ax2 = plt.subplot(gs[2, :4])
		plt.plot(self.steering, 'b', label="repeat")
		plt.plot(self.steering_should, 'r', label="should")
		plt.ylabel('steering[deg]')

		ax3 = plt.subplot(gs[3:5,:2])
		teach_x, teach_y = getTwoArrays(self.teach_path)
		repeat_x, repeat_y = getTwoArrays(self.repeat_path)
		plt.plot(teach_x, teach_y, 'go', label="teach")
		plt.plot(repeat_x, repeat_y, 'bo', label="repeat")
		plt.ylabel('Teach and Repeat path')
		
		ax4 = plt.subplot(gs[3:4,2:4])
		plt.axis('off')
		frame = plt.gca()
		frame.axes.get_xaxis().set_ticks([])
		frame.axes.get_yaxis().set_ticks([])
		path_vals =[['Distance[m]', round(max(self.distance_start),3),'',''],
		 			['K1_S: '+str(k1_s),'K2_S: '+str(k2_s),'K1_V: '+str(k1_v), 'K2_V: '+str(k2_v)]]
		path_table = plt.table(cellText=path_vals,
		                  	   colWidths = [0.1]*4,
		                       loc='center left')
		path_table.set_fontsize(14)
		path_table.scale(2.2,3)

		ax5 = plt.subplot(gs[4:5,2:4])
		plt.axis('off')
		frame = plt.gca()
		frame.axes.get_xaxis().set_ticks([])
		frame.axes.get_yaxis().set_ticks([])
		mean_labels=['','Mean','Variance','Median']
		mean_vals=[['Track Error',round(np.mean(self.tracking_error),3),round(np.var(self.tracking_error),3),round(np.median(self.tracking_error),3)],
					['Velocity',round(np.mean(self.velocity),3),round(np.var(self.velocity),3),round(np.median(self.velocity),3)]]
		mean_table = plt.table(cellText=mean_vals,
		                  	   colWidths = [0.1]*4,
		                  	   colLabels=mean_labels,
		                  	   loc='center left')
		mean_table.set_fontsize(14)
		mean_table.scale(2.2,3)

		plt.savefig(self.file_name + "_infos.png")
		plt.close()
		print("ARC: Saved info pdf to %s" % self.file_name + "_infos.png")

		#Create bigger paths.
		fig2 = plt.figure(figsize=(20, 14))
		plt.plot(teach_x, teach_y, 'go', label="teach")
		plt.plot(repeat_x, repeat_y, 'bo', label="repeat")
		plt.savefig(self.file_name + "_path.png")
		plt.close()
		print("ARC: Saved path visualisation to %s" % self.file_name + "_path.png")

	def commandsCallback(self, msg):
		if not self.ready: self.ready = bool(msg.data[0])

	def dataCallback(self, msg):
		self.velocity.append(3.6*msg.data[0])
		self.velocity_should.append(3.6*msg.data[1])
		self.velocity_teach.append(3.6*msg.data[9])
		self.steering.append(math.degrees(msg.data[2]))
		self.steering_should.append(math.degrees(msg.data[3]))
		self.tracking_error.append(msg.data[5])
		self.distance_start.append(msg.data[7])

	def repeatCallback(self, msg):
		self.repeat_path = np.zeros((1,2))
		for element in msg.poses:
			path_element = np.array([-element.pose.position.y, element.pose.position.x])
			self.repeat_path = np.vstack([self.repeat_path, path_element])
		
	def teachCallback(self, msg):
		self.teach_path = np.zeros((1,2))
		for element in msg.poses:
			path_element = np.array([-element.pose.position.y, element.pose.position.x])
			self.teach_path = np.vstack([self.teach_path, path_element])
	
if __name__ == '__main__':
	#Path to latex.
	file_path = str(sys.argv[1])
	#Init info class.
	information = PDFTracker(file_path)
	#Init subscribing loop.
	rospy.spin()
	#Create PDF.
	try:
		information.createPDF()
	except Exception, e:
		print("ARC: Error with creating pdf -> %s " % e)
	
