#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

import rospy
import math
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import  LaserScan
from geometry_msgs.msg import PoseArray,Pose,Pose2D,Twist, Vector3
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32
from simple_follower.msg import position as PositionMsg
		
class laserTracker:
	def __init__(self):
		self.startEndIndex = np.zeros((1,2))
		self.objectPose = PoseArray()
		self.objectDis = Pose2D()
		self.using = 0
		self.disThresh = 2.5
		self.lastDis = 0
		self.lastAngle = 0
		self.minDis = 12.0
		self.angle_cam = [1280]
		self.targetDis = rospy.get_param('~targetDis')
		self.targetAngle = rospy.get_param('~targetAngle')
		self.angleSubscriber = rospy.Subscriber('/usb_cam/object_angle',Float32,self.angleCallback)
		self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.filterScanPublisher = rospy.Publisher('/filterScan', LaserScan,queue_size=1000)
		self.objectPosePublisher = rospy.Publisher('/object_tracker/objectPoseArray', PoseArray,queue_size=1)
		self.objectDisPublisher = rospy.Publisher('/object_tracker/objectPose2D', Pose2D,queue_size=1)
		self.poseParaToZero()

	def angleCallback(self,data):
		self.angle_cam.append(data.data)
		pass

	def poseParaToZero(self):
		self.range_mid = 0
		self.angle_mid = 0
		self.range_sum = 0
		self.angle_sum = 0
		self.count = 0

	def objectXYThetaToZero(self):
		self.objectDis.x = self.targetDis
		self.objectDis.y = 0
		self.objectDis.theta = self.targetAngle
		self.lastAngle = 0
		self.lastDis = 0

	def posePubSet(self,start,end): # 将符合距离长度的直线放入objectPoseArray集合中
		detectLength = (self.ranges[start]**2 + self.ranges[end]**2 - 2*self.ranges[start]*self.ranges[end]*math.cos((end-start)*self.angle_increment)) ** 0.5
		# detectAngle = self.angle_sum / self.count
		if((detectLength>0.1 and detectLength<0.6)):
			self.pose = Pose()
			self.range_mid = self.range_sum / self.count
			self.angle_mid = self.angle_sum / self.count
			self.pose.position.x = math.cos(self.angle_mid) * self.range_mid
			self.pose.position.y = math.sin(self.angle_mid) * self.range_mid
			self.pose.orientation.w = math.cos(self.angle_mid/2)
			self.pose.orientation.z = math.sin(self.angle_mid/2)
			self.objectPose.poses.append(self.pose)

	def polyContourFit(self,start,end,angle_min,angle_delta,Eps): # 提取直线和角点
		x_start = math.cos((angle_delta * start) + angle_min) * self.ranges[start]
		y_start = math.sin((angle_delta * start) + angle_min) * self.ranges[start]
		x_end = math.cos((angle_delta * end) + angle_min) * self.ranges[end]
		y_end = math.sin((angle_delta * end) + angle_min) * self.ranges[end]
		length = math.sqrt((x_start-x_end) ** 2 + (y_start-y_end) ** 2)
		cosTheta = (x_end - x_start) / length
		sinTheta = -(y_end - y_start) / length
		maxDis = 0
		sumDis = 0
		maxDisInd = -1
		for i in range(start+1,end):
			if(self.ranges[i] != float('inf')):
				x = math.cos((angle_delta * i) + angle_min) * self.ranges[i]
				y = math.sin((angle_delta * i) + angle_min) * self.ranges[i]
				dbDis = abs((y - y_start) * cosTheta + (x - x_start)* sinTheta)
				sumDis = sumDis + dbDis
				if(dbDis>maxDis):
					maxDis = dbDis
					maxDisInd = i
		sumDis = sumDis / (end-start)
		# if(maxDis>Eps):
		# 	if(sumDis>0.05):
		# 		self.posePubSet(start,end)
		# 		return maxDisInd
		# else:
		# 	if(sumDis<0.008):
		# 		self.posePubSet(start,end)
		return 0

	def poseSelect(self): # 在检测出的位置集合中选择目标位置
		tempAngle = 3.14
		if(self.lastDis==0 and self.lastAngle==0):# 目标点初始化
			if(self.angle_cam[-1]==1280):
				rospy.logwarn_once('laserTracker.py: Waiting for camera message')
			else:
				for pose in self.objectPose.poses:
					if(2*math.acos(pose.orientation.w)<60.0*math.pi/180.0):
						angle = math.asin(pose.orientation.z)
						if(abs((angle) + (self.angle_cam[-1]/500.0)) < (tempAngle)):
							tempAngle = abs((angle) + (self.angle_cam[-1]/500.0))
							self.objectDis.x = pose.position.x
							self.objectDis.y = pose.position.y
							self.objectDis.theta = math.asin(pose.orientation.z)*2
							self.lastAngle = self.objectDis.theta
							self.lastDis = (pose.position.x*pose.position.x + pose.position.y*pose.position.y) ** 0.5
				if(abs((angle) + (self.angle_cam[-1]/500.0)) < 0.4):
					rospy.loginfo('laserTracker.py: Follow object is initinated OK')
				else:
					rospy.logwarn('laserTracker.py: Laser and camera error is too big. Initinating.....')
					self.objectXYThetaToZero()
		else:
			if(self.angle_cam[-1]==1280):
				self.objectXYThetaToZero()
				rospy.logwarn('laserTracker.py: Camera lost.')
				rospy.logerr('laserTracker.py: PLEASE STOP THE CAR')
			else:
				if(len(self.objectPose.poses)>0):
					for pose in self.objectPose.poses:
						if(2*math.acos(pose.orientation.w)<60.0*math.pi/180.0):
							angle = math.asin(pose.orientation.z)
							if(abs((angle) + (self.angle_cam[-1]/500.0)) < (tempAngle)):
								tempAngle = abs((angle) + (self.angle_cam[-1]/500.0))
								self.objectDis.x = pose.position.x
								self.objectDis.y = pose.position.y
								self.objectDis.theta = math.asin(pose.orientation.z)*2
					if(abs((self.objectDis.x**2 + self.objectDis.y**2) ** 0.5 - self.lastDis) < 0.6 and abs(self.objectDis.theta - self.lastAngle) < math.pi/4):
						self.using = 0
						self.lastAngle = self.objectDis.theta
						self.lastDis = (self.objectDis.x**2 + self.objectDis.y**2) ** 0.5
						rospy.loginfo_once('laserTracker.py: Everything is OK')
					else:
						self.using = self.using + 1
						if(self.using <3):
							self.objectDis.x = self.lastDis * math.cos(self.lastAngle)
							self.objectDis.y = self.lastDis * math.sin(self.lastAngle)
							self.objectDis.theta = self.lastAngle
							rospy.logwarn('laserTracker.py: This time and last time is too big, using the last pose th follow')
						else:
							self.objectXYThetaToZero()
							self.using = 0
							rospy.logerr('laserTracker.py: Laser lost the object. Laser lost. Reset!')
				else:
					rospy.logerr('laserTracker.py: Laser lost all object, maybe has problem!')
					self.objectXYThetaToZero()
		self.objectDisPublisher.publish(self.objectDis)

	def oneLoopClear(self):
		self.angle_cam[:] = [1280]
		self.objectPose.poses[:] = []
		self.startEndIndex =  np.zeros((1,2))

	def registerScan(self, scan_data):
		recordFlag = 0
		recordCount = 0
		startIndex = 0
		endIndex = 0
		self.ranges = np.array(scan_data.ranges)
		self.angle_increment = scan_data.angle_increment
		# for i in range(520,550):
		# 	print(self.ranges[i])
		# if the topic /scan is not null, then find the continue points and put them in a numpy array named self.startEndIndex 
		if(not(self.ranges is None)):
			# new method
			for i in range(1,self.ranges.size-1):
				if(self.ranges[i] != float('inf')):
					if(recordFlag):
						if(abs(self.ranges[i] - self.ranges[lastIndex]) < 0.08 and i-lastIndex < 5):
							endIndex = i
							lastIndex = i
							recordCount = recordCount + 1
						else:
							# 当点集合数量大于10 ，且有效点数大于集合边缘点的索引差值的1/2，保证点云集合有效
							if(endIndex-startIndex>10 and recordCount>int((endIndex-startIndex)*0.6)):
								self.startEndIndex = np.row_stack((self.startEndIndex,(startIndex,endIndex)))
							recordCount = 0
							# 如果点集最后一个点不在这个集合，但是在下一个集合的时候要记录为开始索引，如果不在就直接下一次循环找第一个点
							if(self.ranges[i]<self.disThresh):
								startIndex = i
								lastIndex = i
								recordFlag  = 1
							else:
								recordFlag = 0
					# 记录点集第一个点的索引，距离小于2.5米
					else:
						if(self.ranges[i] <self.disThresh):
							startIndex = i
							lastIndex = i
							recordFlag = 1					
				else:
					pass
		# 将集合中0值清除
		mask = (self.startEndIndex == 0).all(1)
		self.startEndIndex = self.startEndIndex[~mask,:]
		# 发布过滤点集信息表头
		filterScanMsg = LaserScan()
		filterScanMsg.angle_increment = scan_data.angle_increment
		filterScanMsg.angle_max = scan_data.angle_max
		filterScanMsg.angle_min = scan_data.angle_min
		filterScanMsg.header = scan_data.header
		filterScanMsg.intensities = scan_data.intensities
		filterScanMsg.scan_time = scan_data.scan_time
		filterScanMsg.time_increment = scan_data.time_increment
		filterScanMsg.range_min = scan_data.range_min
		filterScanMsg.range_max = scan_data.range_max
		# 建立提取后的点集，全为无效值
		for i in range(self.ranges.size+1):
			filterScanMsg.ranges.append(float('inf'))
		# 将提取点集进行替换，将提取的点可视化，并且进行直线提取
		for index in self.startEndIndex:
			for i in range(int(index[0]),int(index[1]+1)):
				if(self.ranges[i] != float('inf')):
					filterScanMsg.ranges[i] = self.ranges[i] # set new laser scan topic of the detect laser group 
					self.range_sum = self.range_sum + self.ranges[i]
					self.angle_sum = self.angle_sum + self.angle_increment * i + scan_data.angle_min
					self.count = self.count + 1
			# 拟合直线函数
			self.polyContourFit(start=int(index[0]),end=int(index[1]),angle_min=scan_data.angle_min,angle_delta=self.angle_increment,Eps=0.1)
			# 将起始点和终点索引传入函数，确定位置和角度
			self.posePubSet(start=int(index[0]),end=int(index[1]))
			# 信息归零
			self.poseParaToZero()
		# 发布提取的点集话题
		self.filterScanPublisher.publish(filterScanMsg)
		# 发布提取点集位置集合
		self.objectPose.header = scan_data.header
		self.objectPosePublisher.publish(self.objectPose)
		# 选取目标位置
		self.poseSelect()
		# 输出各项参数
		# print(self.startEndIndex)
		# print(time.time())
		# print('***********************************')
		# 一次循环清除标志位
		self.oneLoopClear()
		
if __name__ == '__main__':
	rospy.init_node('laser_tracker')
	rospy.loginfo("%s starting" % rospy.get_name())
	tracker = laserTracker()
	rospy.loginfo('It seems to do something!!!')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo('exception')


