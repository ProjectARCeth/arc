#!/usr/bin/env python
from PyQt4 import QtGui, QtCore
import pyqtgraph as pg
import math
import numpy as np
import os
import signal
import sys

from ros_interface import ROSInterface

class GUI(QtGui.QWidget):
	def __init__(self, name, init_mode, use_obsdet, use_sensors, use_vcu, use_gps):
		super(GUI, self).__init__()
		#Set mode parameter.
		self.init_mode = init_mode
		self.autonomous_mode = False
		self.system_ready = False
		#Set launchable programms.
		self.use_obsdet = use_obsdet
		self.use_sensors = use_sensors
		self.use_vcu = use_vcu
		self.use_gps = use_gps
		#UI parameter.
		self.height = 600
		self.width = 1000
		#Init ROSInterface.
		self.ros_interface = ROSInterface(init_mode, name)
		#Build up UI.
		if(self.init_mode): self.initUI_Repeat()
		else: self.initUI_Teach()
		self.qtConnections()
		#Init running programms.
		self.running = [0 for i in range(0,9)]

	def initUI_Repeat(self):
		self.setWindowTitle('ARC')
		self.setWindowIcon(QtGui.QIcon('/home/sele/catkin_ws/src/arc/resources/logo.jpg'))
		#Define layouts.
		top_left_layout = QtGui.QHBoxLayout()
		top_right_layout = QtGui.QHBoxLayout()
		middle_left_upper_layout = QtGui.QVBoxLayout()
		middle_left_under_left_layout = QtGui.QVBoxLayout()
		middle_left_under_right_layout = QtGui.QVBoxLayout()
		middle_right_layout = QtGui.QVBoxLayout()
		lower_left_up_layout = QtGui.QHBoxLayout()
		lower_left_down_layout = QtGui.QHBoxLayout()
		lower_center_layout = QtGui.QHBoxLayout()
		lower_right_layout = QtGui.QHBoxLayout()
		#Create title label.
		title_label = QtGui.QLabel("Autonomous Racing Car")
		title_label.setStyleSheet("background-color: white; color: black")
		title_label.setFont(QtGui.QFont('SansSerif', 25))
		title_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		title_label.setFixedSize(self.width*0.75-self.height/9, self.height/12)
		top_left_layout.addWidget(title_label)     
		#Create mode label.
		mode_label = QtGui.QLabel("REPEAT")
		mode_label.setStyleSheet("background-color: black; color: white")
		mode_label.setFont(QtGui.QFont('SansSerif', 25))
		mode_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		mode_label.setFixedSize(self.width/4, self.height/12)
		top_right_layout.addWidget(mode_label)
		#Create Velocity ist Display.
		self.velocity_ist_display = QtGui.QLineEdit("0.0")
		self.velocity_ist_display.setStyleSheet("background-color: black; color: white")
		self.velocity_ist_display.setAlignment(QtCore.Qt.AlignCenter)
		self.velocity_ist_display.setFont(QtGui.QFont('SansSerif', 20,weight=QtGui.QFont.Bold))
		self.velocity_ist_display.setFixedSize(self.width/3, self.height/10,)
		middle_left_upper_layout.addWidget(self.velocity_ist_display)
		#Create Velocity should Display.
		velocity_should_label = self.createLabel("Vel SHOULD", middle_left_under_left_layout)
		self.velocity_should_display = self.createDisplay("white", middle_left_under_left_layout)
		#Create Steering Display.
		steering_ist_label = self.createLabel("Steer IST", middle_left_under_left_layout)
		self.steering_ist_display = self.createDisplay("white", middle_left_under_left_layout)
		steering_should_label = self.createLabel("Steer SHOULD", middle_left_under_left_layout)
		self.steering_should_display = self.createDisplay("white", middle_left_under_left_layout)
		#Create Array Index Display.
		array_index_label = self.createLabel("INDEX", middle_left_under_left_layout)
		self.array_index_display = self.createDisplay("white", middle_left_under_left_layout)
		#Create Tracking Error Display.
		tracking_error_label = self.createLabel("Track Error", middle_left_under_right_layout)
		self.tracking_error_display = self.createDisplay("white", middle_left_under_right_layout)
		#Create Obstacle Distance Display.
		obstacle_distance_label = self.createLabel("Obstacle", middle_left_under_right_layout)
		self.obstacle_distance_display = self.createDisplay("white", middle_left_under_right_layout)
		#Create Distance Display.
		distance_start_label = self.createLabel("START", middle_left_under_right_layout)
		self.distance_start_display = self.createDisplay("white", middle_left_under_right_layout)
		distance_end_label = self.createLabel("END", middle_left_under_right_layout)
		self.distance_end_display = self.createDisplay("white", middle_left_under_right_layout)
		#Path plot.
		self.plotwidget = pg.PlotWidget()
		self.plotcurve = pg.ScatterPlotItem()
		self.plotwidget.addItem(self.plotcurve)
		middle_right_layout.addWidget(self.plotwidget)
		#Create programm label.
		if(self.use_obsdet): self.obstacle_detection_label = self.createProgramm("OBS", lower_left_down_layout, lower_left_up_layout)
		self.pure_pursuit_label = self.createProgramm("PPU", lower_left_down_layout, lower_left_up_layout)
		self.rovio_label = self.createProgramm("ROV", lower_left_down_layout, lower_left_up_layout)
		self.rslam_label = self.createProgramm("RSL", lower_left_down_layout, lower_left_up_layout)
		self.state_estimation_label = self.createProgramm("SES", lower_left_down_layout, lower_left_up_layout)
		if(self.use_vcu): self.vcu_label = self.createProgramm("VCU", lower_left_down_layout, lower_left_up_layout)
		if(self.use_gps and self.use_sensors): self.gps_label = self.createProgramm("GPS", lower_left_down_layout, lower_left_up_layout)
		if(self.use_sensors): self.vi_label = self.createProgramm("VIS", lower_left_down_layout, lower_left_up_layout)
		if(self.use_sensors and self.use_obsdet): self.velodyne_label = self.createProgramm("VEL", lower_left_down_layout, lower_left_up_layout)
		#Shutdown button.
		self.shutdown_button = QtGui.QPushButton("Shutdown")
		self.shutdown_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.shutdown_button.setFixedSize(self.width/6,self.height/8)
		lower_center_layout.addWidget(self.shutdown_button)
		#Stop button.
		self.stop_button = QtGui.QPushButton("STOP")
		self.stop_button.setStyleSheet("background-color: white")
		self.stop_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.stop_button.setFixedSize(self.width/6,self.height/8)
		lower_center_layout.addWidget(self.stop_button)
		#Mode button
		self.start_button = QtGui.QPushButton("System Booting")
		self.start_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.start_button.setStyleSheet("background-color: yellow")
		self.start_button.setFixedSize(self.width/4,self.height/8)
		lower_right_layout.addWidget(self.start_button)
		#Set layouts.
		layout = QtGui.QGridLayout()
		layout.addLayout(top_left_layout,0, 0, 1, 4)
		layout.addLayout(top_right_layout,0,4)
		layout.addLayout(middle_left_upper_layout,1,0,1,2)
		layout.addLayout(middle_left_under_left_layout,2,0,1,2)
		layout.addLayout(middle_left_under_right_layout,2,1,1,2)
		layout.addLayout(middle_right_layout,1,2,2,3)
		layout.addLayout(lower_left_up_layout,4,0,1,2)
		layout.addLayout(lower_left_down_layout,5,0,1,2)
		layout.addLayout(lower_center_layout,4,3,2,1)
		layout.addLayout(lower_right_layout,4,4,2,1)
		self.setLayout(layout)
		#Set Window geometry and background color.
		palette = QtGui.QPalette()
		palette.setColor(QtGui.QPalette.Background,QtCore.Qt.black)
		self.setPalette(palette)
		self.setGeometry(100, 100, self.width, self.height)
		self.show()

	def initUI_Teach(self):
		self.setWindowTitle('ARC')
		self.setWindowIcon(QtGui.QIcon('/home/sele/catkin_ws/src/arc/resources/logo.jpg'))
		#Define layouts.
		top_left_layout = QtGui.QHBoxLayout()
		top_right_layout = QtGui.QHBoxLayout()
		middle_left_layout = QtGui.QVBoxLayout()
		middle_right_layout = QtGui.QVBoxLayout()
		lower_left_up_layout = QtGui.QHBoxLayout()
		lower_left_down_layout = QtGui.QHBoxLayout()
		lower_right_layout = QtGui.QHBoxLayout()
		#Create title label.
		title_label = QtGui.QLabel("Autonomous Racing Car")
		title_label.setStyleSheet("background-color: white; color: black")
		title_label.setFont(QtGui.QFont('SansSerif', 25))
		title_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		title_label.setFixedSize(self.width*0.75-self.height/9, self.height/12)
		top_left_layout.addWidget(title_label)     
		#Create mode label.
		mode_label = QtGui.QLabel("TEACH")
		mode_label.setStyleSheet("background-color: black; color: white")
		mode_label.setFont(QtGui.QFont('SansSerif', 25))
		mode_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		mode_label.setFixedSize(self.width/4, self.height/12)
		top_right_layout.addWidget(mode_label)
		#Create Velocity ist Display.
		self.velocity_ist_display = QtGui.QLineEdit("0.0")
		self.velocity_ist_display.setStyleSheet("background-color: black; color: white")
		self.velocity_ist_display.setAlignment(QtCore.Qt.AlignCenter)
		self.velocity_ist_display.setFont(QtGui.QFont('SansSerif', 20,weight=QtGui.QFont.Bold))
		self.velocity_ist_display.setFixedSize(self.width/6, self.height/10,)
		middle_left_layout.addWidget(self.velocity_ist_display)
		#Create Steering Display.
		steering_ist_label = self.createLabel("Steer IST", middle_left_layout)
		self.steering_ist_display = self.createDisplay("white", middle_left_layout)
		#Create Array Index Display.
		array_index_label = self.createLabel("INDEX", middle_left_layout)
		self.array_index_display = self.createDisplay("white", middle_left_layout)
		#Path plot.
		self.plotwidget = pg.PlotWidget()
		self.plotcurve = pg.ScatterPlotItem()
		self.plotwidget.addItem(self.plotcurve)
		middle_right_layout.addWidget(self.plotwidget)
		#Create programm label.
		self.rovio_label = self.createProgramm("ROV", lower_left_down_layout, lower_left_up_layout)
		self.rslam_label = self.createProgramm("RSL", lower_left_down_layout, lower_left_up_layout)
		self.state_estimation_label = self.createProgramm("SES", lower_left_down_layout, lower_left_up_layout)
		if(self.use_vcu): self.vcu_label = self.createProgramm("VCU", lower_left_down_layout, lower_left_up_layout)
		if(self.use_gps and self.use_sensors): self.gps_label = self.createProgramm("GPS", lower_left_down_layout, lower_left_up_layout)
		if(self.use_sensors): self.vi_label = self.createProgramm("VIS", lower_left_down_layout, lower_left_up_layout)
		#Mode button
		self.start_button = QtGui.QPushButton("System Booting")
		self.start_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.start_button.setStyleSheet("background-color: yellow")
		self.start_button.setFixedSize(self.width/4,self.height/8)
		lower_right_layout.addWidget(self.start_button)
		#Set layouts.
		layout = QtGui.QGridLayout()
		layout.addLayout(top_left_layout,0, 0, 1, 4)
		layout.addLayout(top_right_layout,0,4)
		layout.addLayout(middle_left_layout,1,0,1,3)
		layout.addLayout(middle_right_layout,1,1,3,4)
		layout.addLayout(lower_left_up_layout,4,0,1,4)
		layout.addLayout(lower_left_down_layout,5,0,1,4)
		layout.addLayout(lower_right_layout,4,4,2,2)
		self.setLayout(layout)
		#Set Window geometry and background color.
		palette = QtGui.QPalette()
		palette.setColor(QtGui.QPalette.Background,QtCore.Qt.black)
		self.setPalette(palette)
		self.setGeometry(100, 100, self.width, self.height)
		self.show()

	def qtConnections(self):
		#Buttons.
		if(self.init_mode): self.shutdown_button.clicked.connect(self.shutdown)
		self.start_button.clicked.connect(self.changeMode)
		if(self.init_mode): self.stop_button.clicked.connect(self.emergencyStop)
		#Ros Signals.
		self.ros_interface.info_signal.connect(self.updateDisplay)
		self.ros_interface.programm_signal.connect(self.updateProgrammDisplay)
		self.ros_interface.repeat_path_signal.connect(self.updateRepeatPath)
		self.ros_interface.teach_path_signal.connect(self.updateTeachPath)

	def createDisplay(self, color, layout):
		display = QtGui.QLineEdit("0.0")
		display.setStyleSheet("background-color: " + color)
		display.setFixedSize(self.width/10, self.height/20)
		layout.addWidget(display)
		return display

	def createLabel(self, name, layout):
		label = QtGui.QLabel(name)
		label.setStyleSheet("color: white")
		label.setFont(QtGui.QFont("Times",weight=QtGui.QFont.Bold))
		label.setFixedSize(self.width/10, self.height/20)
		layout.addWidget(label)
		return label

	def createProgramm(self, name, text_layout, color_layout):
		text_label = QtGui.QLabel(name)
		text_label.setStyleSheet("color: white")
		text_label.setFont(QtGui.QFont("Times",weight=QtGui.QFont.Bold))
		text_label.setFixedSize(self.width/20, self.height/20)
		text_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		text_layout.addWidget(text_label)
		color_label = QtGui.QLabel("")
		color_label.setStyleSheet("background-color:red")
		color_label.setFixedSize(self.width/20, self.height/20)
		color_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		color_layout.addWidget(color_label)
		return color_label

	def updateDisplay(self, infos):
		self.velocity_ist_display.setText(str('%.2f' % (infos[0]*3.6)))
		if(self.init_mode): self.velocity_should_display.setText('%.2f'%(infos[1]*3.6))
		self.steering_ist_display.setText(str('%.2f'%math.degrees(infos[2])))
		if(self.init_mode): self.steering_should_display.setText(str('%.2f'%math.degrees(infos[3])))
		self.array_index_display.setText(str('%.2f'%infos[4]))
		if(self.init_mode): self.tracking_error_display.setText(str('%.2f'%infos[5]))
		if(self.init_mode): self.obstacle_distance_display.setText(str('%.2f'%infos[6]))
		if(self.init_mode): self.distance_start_display.setText(str('%.2f'%infos[7]))
		if(self.init_mode): self.distance_end_display.setText(str('%.2f'%infos[8]))

	def updateRepeatPath(self, new_pose):
		repeat_x = [new_pose[0]]
		repeat_y = [new_pose[1]]
		self.plotcurve.addPoints(repeat_x, repeat_y, symbol='o', pen=QtGui.QPen(QtGui.QColor(0, 0, 255)))

	def updateTeachPath(self, teach_path):
		teach_x = [item[0] for item in teach_path]
		teach_y = [item[1] for item in teach_path]
		self.plotcurve.addPoints(teach_x, teach_y, symbol='o', pen=QtGui.QPen(QtGui.QColor(0, 255, 0)))

	def updateProgrammDisplay(self, running):
		#Updating programm labels.
		if((running[0] != self.running[0]) and self.use_obsdet and self.init_mode): self.checkProgramm(self.obstacle_detection_label, running[0])
		if(running[1] != self.running[1] and self.init_mode): self.checkProgramm(self.pure_pursuit_label, running[1])
		if(running[2] != self.running[2]): self.checkProgramm(self.rovio_label, running[2])
		if(running[3] != self.running[3]): self.checkProgramm(self.rslam_label, running[3])
		if(running[4] != self.running[4]): self.checkProgramm(self.state_estimation_label, running[4])
		if(running[5] != self.running[5] and self.use_vcu): self.checkProgramm(self.vcu_label, running[5])
		if(running[6] != self.running[6] and self.use_gps and self.use_sensors): self.checkProgramm(self.gps_label, running[6])
		if(running[7] != self.running[7] and self.use_sensors): self.checkProgramm(self.vi_label, running[7])
		if(running[8] != self.running[8] and self.use_sensors and self.use_obsdet and self.init_mode): self.checkProgramm(self.velodyne_label, running[8])
		#Updating programm state.
		self.running = running
		#Check system startable.
		self.system_ready = True
		for element in running: 
			if(element == 0): self.system_ready = False
		if(self.system_ready): 
			self.start_button.setStyleSheet("background-color: cyan")
			self.start_button.setText("Ready") 

	def checkProgramm(self, label, running_state):
		if(running_state == 0): label.setStyleSheet("background-color: red")
		if(running_state == 1): label.setStyleSheet("background-color: green")

	def changeMode(self):
		#Change to manuell iff autonomous and repeat.
		if(self.init_mode and self.autonomous_mode):
			self.autonomous_mode = False
			self.system_ready = True
			self.start_button.setStyleSheet("background-color: yellow")
			self.start_button.setText("Autonomous")
			self.ros_interface.publishInfo([0,0,0])
		#Launch system iff ready only one time and go autonomous.
		elif(self.init_mode and self.system_ready):
			self.system_ready = False
			self.autonomous_mode = True
			self.start_button.setStyleSheet("background-color: blue")
			self.start_button.setText("Manuell")
			self.ros_interface.publishInfo([1,0,0])
		#Launch system iff ready only one time but stay manuelly.
		elif(self.system_ready):
			self.system_ready = False
			self.start_button.setStyleSheet("background-color: cyan")
			self.start_button.setText("System started") 

	def emergencyStop(self):
		self.stop_button.setStyleSheet("background-color: red")
		self.ros_interface.publishInfo([int(self.autonomous_mode),0,1])

	def shutdown(self):
		self.shutdown_button.setText("Slowing down")
		self.ros_interface.publishInfo([int(self.autonomous_mode),1,0])

def main():
	#Starting application.
	app = QtGui.QApplication(sys.argv)
	app.setApplicationName('ARC')
	#Getting Mode.
	name = str(sys.argv[1])
	mode = (sys.argv[2]=="true")
	use_obsdet = (sys.argv[3]=="true")
	use_sensors = (sys.argv[4]=="true")
	use_vcu = (sys.argv[5]=="true")
	use_gps = (sys.argv[6]=="true")
	#Creating GUI.
	ex = GUI(name, mode, use_obsdet, use_sensors, use_vcu, use_gps)
	# Exit.
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
