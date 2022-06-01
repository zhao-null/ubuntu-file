#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

import platform
import rospy
import math
import time
import numpy as np
import cv2 as cv
from visualization_msgs.msg import Marker,MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import  LaserScan,Image
from geometry_msgs.msg import Pose,PoseArray,Point,Pose2D


class laserTracker:
	def __init__(self):
		# self.a = [-2.95521480,  2.36945093,  340.096029, -2.09559005, 0.0606777124, 327.716691, -0.00867824383, 0.000171793232, 1]
		# self.a = [-3805.08333,  2749.40011,  276.022013, -2695.78278, -45.1555397,  411.914789, -10.9596554, -0.177056215, 1]
		# self.a = [-3.80070865,  2.74604306,  275.960786, -2.69276354,  -0.0450003636,  411.678747, -0.0109476245, -0.000176599423, 1]
		self.a = [-6.69249150, 4.89812674, 395.065579, -5.34907425, -0.201485137, 602.245746, -0.0186462658, -0.000573126502, 1]
		self.bridge = CvBridge()
		self.cv_image = np.array([])
		self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.cameraSubscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.cameraImage)
		self.imagePublisher = rospy.Publisher("/usb_cam/detect_image", Image, queue_size=1)

	def registerScan(self, scan_data):
		self.header = scan_data.header
		self.ranges = np.array(scan_data.ranges)
		self.angle_increment = scan_data.angle_increment
		self.angle_min = scan_data.angle_min


		# if(not(self.ranges is None) and not(self.cv_image is None)):
		for i in range(1,self.ranges.size-1):
			if(self.ranges[i] != float('inf') and self.ranges[i]!=0):
				angle = self.angle_increment * i + self.angle_min
				if (angle>-45*math.pi/180 and angle<45*math.pi/180):
					u = (int)(1000*self.ranges[i]*math.cos(angle))
					v = (int)(1000*self.ranges[i]*math.sin(angle))
					lamb = u*self.a[6] + v*self.a[7] + self.a[8]
					x = int((u*self.a[0] + v*self.a[1] + self.a[2]) / lamb)
					y = int((u*self.a[3] + v*self.a[4] + self.a[5]) / lamb)
					cv.circle(self.cv_image, (x, y), 1, (0, 255, 255), thickness=1)
					# print(y)
						# if(x > self.center_x-self.w/2 and x < self.center_x+self.w/2):
						# 	self.filterScan.ranges[i] = self.ranges[i]
						# 	self.positionXYI = np.row_stack((self.positionXYI,(x,y,i,angle)))
						# 	cv.circle(self.cv_image, (x, y), 2, (0, 255, 255), thickness=2)
						# else:s
						# 	cv.circle(self.cv_image, (x, y), 2, (255, 255, 0), thickness=-1)
		self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))



	def cameraImage(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)




if __name__ == '__main__':
	#time.sleep(3)
	try:
		rospy.init_node("cv_bridge_test")
		rospy.loginfo("Starting cv_bridge_test node")
		laserTracker()
		rospy.spin()
		if(rospy.is_shutdown()):
			cv.destroyAllWindows()
			print ("Shutting down cv_bridge_test node.")
	except KeyboardInterrupt:
		print ("Shutting down cv_bridge_test node.")
		cv.destroyAllWindows()
