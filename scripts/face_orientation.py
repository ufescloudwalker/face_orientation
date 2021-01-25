#!/usr/bin/python
import cv2
import dlib
import numpy as np
from imutils import face_utils

from imutils.video import WebcamVideoStream
from imutils.video import FPS
import imutils

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image 				#Added for ros image subscribing
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError	#Added for ros image subscribing

import time # For better benchmarking. 
			# Please, note that time.clock() 
			# is platform dependant and 
			# has been deprecated since python 3.3. 

import serial # To communicate with an external Arduino
			  # it lights up a LED to indicate that a face has been detected


class FaceOrientation():
	def __init__(self):
		self.rospy = rospy
		self.rospy.init_node('FaceOrientation', anonymous = True)
		self.rospy.loginfo("Starting Head Controller")
		self.initParameters()
		self.initPublishers()
		self.initSubscribers() #Added for ros image subscribing
		self.initVariables()
		self.mainControl()

	def initParameters(self):
		self.posHeadTopic = "pos_head_topic"
		self.imageTopic = self.rospy.get_param("~image_topic","/usb_cam_head/image_raw/compressed")
		self.imageCompressed = self.rospy.get_param("~use_compressed", "True")
		self.controlRate = self.rospy.get_param("~control_rate", 100)
		self.face_landmark_path = self.rospy.get_param("~face_landmark_path", "")
		self.angle_min = self.rospy.get_param("~angle_min", 15)
		self.cam_id = self.rospy.get_param("~cam_id",0)
		self.control_rate = self.rospy.get_param("~control_rate","")
		self.D = self.rospy.get_param("~D",[])
		self.object_pts = np.float32(self.rospy.get_param("~object_pts",[]))
		self.reprojectsrc = np.float32(self.rospy.get_param("~reprojectsrc",[]))
		self.line_pairs = self.rospy.get_param("~line_pairs",[])
		return

	def initPublishers(self):
		self.pubPosHead = self.rospy.Publisher(self.posHeadTopic, String, queue_size = 10)
		self.logtime = self.rospy.Publisher("log_times", String, queue_size = 10)
		return

	def initSubscribers(self):
		if self.imageCompressed:
			self.subImage = self.rospy.Subscriber(self.imageTopic, CompressedImage, self.imageCallback)
		else:
			self.subImage = self.rospy.Subscriber(self.imageTopic, Image, self.imageCallback)
		return

	def initVariables(self):
		self.action = 0
		self.bridge = CvBridge() 
		self.dist_coeffs = np.array(self.D).reshape(5, 1).astype(np.float32)

		self.euler_angle = 0
		self.angle = 0
		self.data = ['Start']
		self.change = False
		self.rate = self.rospy.Rate(self.controlRate)

		return

	def get_head_pose(self, shape):
		self.image_pts = np.float32([shape[17], shape[21], shape[22], shape[26], shape[36],
				        shape[39], shape[42], shape[45], shape[31], shape[35],
				        shape[48], shape[54], shape[57], shape[8]])
		_, rotation_vec, translation_vec = cv2.solvePnP(self.object_pts, self.image_pts, self.cam_matrix, self.dist_coeffs)

		self.reprojectdst, _ = cv2.projectPoints(self.reprojectsrc, rotation_vec, translation_vec, self.cam_matrix,
				                    self.dist_coeffs)

		self.reprojectdst = tuple(map(tuple, self.reprojectdst.reshape(8, 2)))

		# calc euler angle
		rotation_mat, _ = cv2.Rodrigues(rotation_vec)
		pose_mat = cv2.hconcat((rotation_mat, translation_vec))
		_, _, _, _, _, _, self.euler_angle = cv2.decomposeProjectionMatrix(pose_mat)
		return

	def makePosMsg(self):
		msg = String()
		msg.data = self.data[-1]
		self.pubPosHead.publish(msg)

		return

	def imageCallback(self, msg):
		try:
			if self.imageCompressed:
				self.frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
				self.change = True
			else:
				self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
				self.change = True
		except CvBridgeError as e:
			print(e)
		return

	def mainControl(self):
		detector = dlib.get_frontal_face_detector()
		#carrega o modelo
		predictor = dlib.shape_predictor(self.face_landmark_path)

		time_msg = String()
		self.fps = FPS().start() # let FPS.start be closer to while loop
		while not self.rospy.is_shutdown():
			if self.change:
				get_time = time.clock() # For benchmarking image processing times

				frame = self.frame
				frame = imutils.resize(frame, width=400)

				#update
				size = frame.shape
				focal_length = size[1]
				center = (size[1]/2, size[0]/2)
				self.cam_matrix = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype = np.float32)

				frame = cv2.flip(frame, 1)
				face_rects = detector(frame, 0)
				
				if len(face_rects) > 0:
					shape = predictor(frame, face_rects[0])
					shape = face_utils.shape_to_np(shape)

					self.get_head_pose(shape)

					time_msg.data = "Got face: " + str(time.clock() - get_time)
					self.logtime.publish(time_msg)

					#comandos
					if (self.euler_angle[1, 0] > 15):
						self.action = -1
						cv2.putText(frame, "Left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (255, 0, 0), thickness=1)
					elif (self.euler_angle[1, 0] < -15):
						self.action = 1
						cv2.putText(frame, "Right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (255, 0, 0), thickness=1)
					else:
						self.action = 0
						cv2.putText(frame, "center", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (255, 0, 0), thickness=1)
					for (x, y) in self.image_pts:
						cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)

					for start, end in self.line_pairs:
						cv2.line(frame, self.reprojectdst[start], self.reprojectdst[end], (0, 0, 255))

					cv2.putText(frame, "Angle: " + "{:3.0f}".format(self.euler_angle[1, 0]), (10, 10), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (0, 0, 0), thickness=1)


					self.angle = self.euler_angle[1, 0]

					self.data.append(str(round(self.angle,3))+":"+str(self.action))

					self.makePosMsg()

				else:

					time_msg.data = "No face: " + str(time.clock() - get_time)
					self.logtime.publish(time_msg)


					self.data = []
					self.data.append('Start')

					self.makePosMsg()

				cv2.imshow("FaceOrientation", frame)
				self.fps.update()
				try:
					time_msg.data = "Current mean FPS: " + str(self.fps.fps())
					self.logtime.publish(time_msg)
				except:
					time_msg.data = "Current mean FPS: " + str(0)
					self.logtime.publish(time_msg)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
				self.change = False
			self.rate.sleep()

		self.fps.stop()
		print("[INFO] time: {:.2f}".format(self.fps.elapsed()))
		print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
		cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		hc = FaceOrientation()
	except rospy.ROSInterruptException:
		pass
