#!/usr/bin/env python

import rospy
import cv2
import re
import sys
import numpy as np
from std_msgs.msg import String, Int8
from sensor_msgs.msg import CompressedImage, Image
# import hil_servoing.msg as msg
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
import base64
from io import BytesIO
from PIL import Image, ImageDraw, ImageChops
TEXT_WIDTH = 420
TEXT_HEIGHT = 150


class UDPGuiWidget(QWidget):
	#############################################################################################################
	# INITIALIZATION METHODS
	#############################################################################################################
	def __init__(self):
		# print(dir(msg))
		super(UDPGuiWidget, self).__init__()
		self.setWindowTitle("University of Alberta")
		self.setStyleSheet("background-color:rgba(150, 150, 150, 100%)")
		# global variables
		self.cvbridge = CvBridge()
		self.clicked_points = []
		self.step1_complete = False
		# setup subscribers and publishers
		self.sub_response = rospy.Subscriber("/udp/response", String, self.callback_udp_response, queue_size=3)
		self.pub_udp_request = rospy.Publisher("/udp/request", String, queue_size=1) #change to udp/request
		self.initialize_widgets()

	def initialize_widgets(self):
		self.main_layout = QVBoxLayout()
		self.select_widget = QWidget()
		self.select_widget.setLayout(self.select_layout())
		self.main_layout.addWidget(self.select_widget)
		self.setLayout(self.main_layout)
		self.pen = QPen(Qt.magenta)
		self.pen.setCapStyle(Qt.RoundCap)
		self.pen.setJoinStyle(Qt.RoundJoin)
		self.pen.setWidth(10)
#############################################################################################################
# WIDGET METHODS
#############################################################################################################
	def send_command_button(self): ## send fine task specification data, i.e., the rectangle string.
		if len(self.clicked_points) > 0:
			self.pub_udp_request.publish(self.format_string(self.clicked_points))

	def go2coarseTarget(self):  # send command to go to coarse target. coarse_target:1:preset_mark, from 1~ 4 orientation
		orientation_preset_mark = "0"  # 0 means don't change orientation
		if self.cb2.isChecked():
			orientation_preset_mark = "1"
		elif self.cb3.isChecked():
			orientation_preset_mark = "2"
		elif self.cb4.isChecked():
			orientation_preset_mark = "3"
		elif self.cb5.isChecked():
			orientation_preset_mark = "4"
		self.pub_udp_request.publish("coarse_target:1:"+orientation_preset_mark)

	def acquire_image_button(self):
		print "Acquiring an image..."
		self.pub_udp_request.publish("require_image:0:fine")

	def select_layout(self):
		# layouts
		l = QVBoxLayout()
		layout = QHBoxLayout()
		grasp_layout = QHBoxLayout()
		step_layout = QVBoxLayout()
		rhs_layout = QVBoxLayout()
		msgs = []
		for i in range(5):
			m = QLabel()
			m.setWordWrap(True)
			m.setAlignment(Qt.AlignLeft)
			if i == 0 or i == 3:
				m.setStyleSheet("font-size: 32px")
			else:
				m.setStyleSheet("font-size: 22px")
			msgs.append(m)
		# title
		title_msg = QLabel()
		title_msg.setText("Long Range Teleoperation: Operator Side Console")
		title_msg.setWordWrap(True)
		title_msg.setStyleSheet("font-size: 36px")
		title_msg.setAlignment(Qt.AlignCenter)
		title_msg.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		title_msg2 = QLabel()
		title_msg2.setText("University of Alberta")
		title_msg2.setWordWrap(True)
		title_msg2.setStyleSheet("font-size: 28px; color: rgba(0, 102, 0, 100%)")
		title_msg2.setAlignment(Qt.AlignCenter)
		title_msg2.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		# MESSAGES
		msgs[0].setText("Step 1: Coarse Target Selection")
		msgs[1].setText("Click once in the image to select a target")
		msgs[2].setText("Select a grasping pose then press button below to send command")
		msgs[3].setText("Step 2: Visual Task Specification")
		msgs[4].setText("Please specify a task in the new image then send command")
		# CHECKBOXES
		cb0 = QCheckBox("Robot Online")
		cb0.setStyleSheet("color: rgba(0, 255, 0, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		cb0.setChecked(True)

		self.cb1 = QCheckBox("Target selected in telepresence interface")
		self.cb1.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		# icon = QIcon("green_button.png")
		# cb1.setIcon(QIcon("green_button.png"))
		# cb1.setStyleSheet('QCheckBox::indicator:checked:hover {image: url(green_button.png);}'
                   # 'QCheckBox::indicator:checked {image: url(green_button.png);}')
		# cb1.setChecked(True)
		self.cb2 = QCheckBox("Top")
		self.cb2.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		self.cb3 = QCheckBox("Left")
		self.cb3.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		self.cb4 = QCheckBox("Right")
		self.cb4.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		self.cb5 = QCheckBox("Front")
		self.cb5.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")

		grasp_layout.addWidget(self.cb2)
		grasp_layout.addWidget(self.cb3)
		grasp_layout.addWidget(self.cb4)
		grasp_layout.addWidget(self.cb5)

		self.cb6 = QCheckBox("Coarse target reached!")
		self.cb6.setStyleSheet("color: rgba(0, 255, 0, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		self.cb7 = QCheckBox("Visual task specification complete!")
		self.cb7.setStyleSheet("color: rgba(0, 255, 255, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px")
		self.cb8 = QCheckBox("Task complete!")
		self.cb8.setStyleSheet("color: rgba(0, 255, 0, 100%); background-color:rgba(150, 150, 150, 100%); font-size: 22px; font-weight: bold")
		# BUTTONS
		img_button = QPushButton("Acquire an image from remote")
		img_button.clicked.connect(self.acquire_image_button)
		img_button.setStyleSheet("background-color: rgba(220, 220, 220, 50%); \
			selection-background-color: rgba(220, 220, 220, 50%); font-size: 28px")
		img_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

		coarse_command_button = QPushButton("Approaching coarse target")
		coarse_command_button.clicked.connect(self.go2coarseTarget)

		publish_button = QPushButton("Send command to remote robot")
		publish_button.clicked.connect(self.send_command_button)
		publish_button.setStyleSheet("background-color: rgba(220, 220, 220, 50%); \
			selection-background-color: rgba(220, 220, 220, 50%); font-size: 28px")
		publish_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

		# image view
		self._view = QLabel()
		self._view.setAlignment(Qt.AlignCenter)
		self._view.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
		self._view.mousePressEvent = self.mousePressEventCam

		step_layout.addWidget(msgs[0])
		step_layout.addWidget(msgs[1])
		step_layout.addWidget(self.cb1)
		step_layout.addWidget(msgs[2])
		step_layout.addLayout(grasp_layout)

		step_layout.addWidget(self.cb6)
		step_layout.addWidget(coarse_command_button)
		step_layout.addWidget(msgs[3])
		step_layout.addWidget(img_button)
		step_layout.addWidget(msgs[4])
		step_layout.addWidget(self.cb7)
		step_layout.addWidget(self.cb8)
		# step1_layout.addWidget(reset_button)
		layout.addLayout(step_layout)
		rhs_layout.addWidget(cb0)
		rhs_layout.addWidget(self._view)
		layout.addLayout(rhs_layout)
		l.addWidget(title_msg)
		l.addWidget(title_msg2)
		# l.addWidget(cb0)
		l.addLayout(layout)
		l.addWidget(publish_button)
		return l

#############################################################################################################
# CALLBACK METHODS
#############################################################################################################
	def callback_camera(self, data):
		# self.pixmap = self.convert_img(data)
		pmap = self.convert_compressed_img(data)
		self.paint_pixmap(pmap)
		self._view.setPixmap(pmap)

	def format_string(self, pt):
		return str(pt[0]) + " " + str(pt[1])

	def callback_udp_response(self, msg):  #process reponse data
		if msg.data.find("coarse_target:0:done") is not -1:
			self.cb6.setChecked(True)
			self.step1_complete = True
		elif msg.data.find("fine_target:0:done") is not -1:
			self.cb8.setChecked(True)
		elif msg.data.find("target_image") is not -1:
			## decode target image data
			base64ImgData = msg.data.split(":")[2]  # target_image:0:image_data base 64s
			# base64 display in GUI
			rospy.loginfo(rospy.get_caller_id() + "img_str %s", msg.data[0:10])
			if msg.data is "" or msg.data is "h":
				print('img str base64 encode invalid!')
				return
			current_image = Image.open(BytesIO(base64.b64decode(msg.data)))
			p = self.convert_img_pil(current_image)
			self.paint_pixmap(p)
        	self._view.setPixmap(p)  #display the image, self._view is the image viewer





#############################################################################################################
# IMAGE METHODS
#############################################################################################################
	def paint_pixmap(self, pix):
		painter = QPainter(pix)
		if self.clicked_points:
			painter.setPen(self.pen)
			painter.drawPoint(self.clicked_points[0], self.clicked_points[1])

	def convert_img_pil(self, img):
		try:
			frame = np.asarray(img)
		except:
			print ("2")
		try:
			qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_Indexed8)
		except:
			print ("3")
		pixmap = QPixmap.fromImage(qimg)
		return pixmap

	def convert_img(self, data):
		try:
			frame = self.cvbridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		image = QImage(
			frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

	def convert_compressed_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

#############################################################################################################
# EVENT METHODS
#############################################################################################################
	def mousePressEventCam(self, event):
		if event.button() == 1:
			pos = self._view.mapFromGlobal(event.globalPos())
			self.clicked_points = [pos.x(), pos.y()]
			print self.clicked_points
			if self.step1_complete:
				self.cb7.setChecked(True)
			self.cb1.setChecked(True)
		elif event.button() == 2:
			self.clicked_points = []
			self.cb1.setChecked(False)
			self.cb7.setChecked(False)

	def keyPressEvent(self, QKeyEvent):
		key = QKeyEvent.key()
		if int(key) == 82:  # r
			self.reset()
		elif int(key) == 68:  # d
			self.debug()
		elif int(key) == 81:  # q
			sys.exit()
		else:
			print "Unknown key option: ", key
