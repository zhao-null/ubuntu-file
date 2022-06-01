#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

import re
import rospy
import os
import csv
import codecs
import math
import time
import cv2
import numpy as np
from sensor_msgs.msg import  LaserScan,Image
from geometry_msgs.msg import PoseArray,Pose,Pose2D,Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32,Int16
from simple_follower.msg import position as PositionMsg
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.pyplot import MultipleLocator
from cv_bridge import CvBridge, CvBridgeError

class dataRecorder:
	def __init__(self):
		# self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.objectErrCallback,queue_size=1)
		# self.filterScanPublisher = rospy.Publisher('slave/filterScan', LaserScan,self.filterScanCallback,queue_size=1)
		self.cameraSubscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.cameraImage)
		# self.objectPoseSubscriber = rospy.Subscriber('/objectErr', Pose2D,self.paraCallback)
		self.bridge = CvBridge()
		self.all = []


	# def paraCallback(self,error):
	# 	self.x = error.x
	# 	self.y = error.y
	# 	self.angle = error.theta
	# 	self.all.append([self.x,self.y,self.angle])

	# def objectErrCallback(self, scan_data):
	# 	self.ranges = np.array(scan_data.ranges)
	# 	self.angle_increment = scan_data.angle_increment
	# 	self.angle_min = scan_data.angle_min
	
	# def filterScanCallback(self,scan_data):
	# 	self.filterRanges = np.array(scan_data.ranges)

	def cameraImage(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

def data_write_csv(datas):#file_name为写入CSV文件的路径，datas为要写入数据列表
    for i in range(100):
        print(('/home/zcy/essay_data/image/' + str(i) + '.csv'))
        if(os.path.isfile('/home/zcy/essay_data/image/' + str(i) + '.csv')):
            pass
        else:
            file_name = '/home/zcy/essay_data/image/' + str(i) + '.csv'
            print(i)
            break
    file_csv = codecs.open(file_name,'w+','utf-8')#追加
    writer = csv.writer(file_csv, delimiter=',', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
    for data in datas:
        writer.writerow(data)
    print("保存文件成功，处理结束")


if __name__ == '__main__':
	rospy.init_node('data_recoder')
	rospy.loginfo("%s starting" % rospy.get_name())
	recoder = dataRecorder()
	rospy.loginfo('It seems to do something!!!')
	# x = []
	# y = []
	# x1=[]
	# y1=[]
	# colors = '#000000'
	# colors1 = '#000000'
	try:
		rospy.spin()
		if(rospy.is_shutdown()):
			# data_write_csv(recoder.all[-10:])
			# plt.figure(figsize=(5,5))
			# for i in range(len(recoder.ranges)):
			# 	if(recoder.ranges[i] != 0):
			# 		x.append(recoder.ranges[i] * math.cos(recoder.angle_min + i*recoder.angle_increment))
			# 		y.append(recoder.ranges[i] * math.sin(recoder.angle_min + i*recoder.angle_increment))
			# 	if(recoder.filterRanges[i]!=0):
			# 		x1.append(recoder.filterRanges[i] * math.cos(recoder.angle_min + i*recoder.angle_increment))
			# 		y1.append(recoder.filterRanges[i] * math.sin(recoder.angle_min + i*recoder.angle_increment))
			# plt.scatter(x,y,c='b',marker='+')
			# plt.scatter(x,y,c=colors)
			# # plt.plot(recoder.timeArray,recoder.angleArray)
			# plt.xlabel('X / m', horizontalalignment='center',verticalalignment='baseline')
			# plt.ylabel('Y / m', horizontalalignment='center',verticalalignment='baseline')
			# ax=plt.gca()
			# plt.ylim(-2.5,2.5)
			# plt.xlim(0,5)
			# x_major_locator=MultipleLocator(10)
			# ax.yaxis.set_major_locator(x_major_locator)
			# plt.savefig('/home/zcy/essay_data/picture/xerror.jpg')

			# plt.figure('横向偏移',figsize=(15,5))
			# plt.subplot(3,1,2)
			# plt.grid()
			# plt.plot(recoder.yerror)
			# # plt.plot(recoder.timeArray,recoder.angleArray)
			# plt.ylabel('y_error/(cm)', horizontalalignment='center',verticalalignment='baseline')
			# ax=plt.gca()
			# plt.ylim(-30,30)
			# x_major_locator=MultipleLocator(10)
			# ax.yaxis.set_major_locator(x_major_locator)
			# # plt.savefig('/home/zcy/essay_data/picture/yerror.jpg')

			# # plt.figure('航向偏移',figsize=(15,5))
			# plt.subplot(3,1,3)
			# plt.grid()
			# plt.plot(recoder.aerror)
			# # plt.plot(recoder.timeArray,recoder.angleArray)
			# plt.xlabel('data', horizontalalignment='center')
			# plt.ylabel('a_error/($^\circ$)', horizontalalignment='center',verticalalignment='baseline')
			# ax=plt.gca()
			# plt.ylim(-10,10)
			# x_major_locator=MultipleLocator(10)
			# ax.yaxis.set_major_locator(x_major_locator)

			
			for i in range(100):
				print(('/home/zcy/essay_data/image/' + str(i) + '.jpg'))
				if(os.path.isfile('/home/zcy/essay_data/image/' + str(i) + '.jpg')):
					pass
				else:
					file_name = '/home/zcy/essay_data/image/' + str(i) + '.jpg'
					pic_name = '/home/zcy/essay_data/image/' + str(i) + '.png'
					print(i)
					break
			# plt.savefig(file_name)
			cv2.imwrite(pic_name,recoder.cv_image)

			# plt.show()
			rospy.loginfo('the data_recorder is shutdown!')
	except rospy.ROSInterruptException:
		rospy.loginfo('exception')


