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
		#self.a = [-3.80070865,  2.74604306,  275.960786, -2.69276354,  -0.0450003636,  411.678747, -0.0109476245, -0.000176599423, 1]
		self.a = [-6.69249150, 4.89812674, 395.065579, -5.34907425, -0.201485137, 602.245746, -0.0186462658, -0.000573126502, 1]
		self.bridge = CvBridge()
		self.poseArray = PoseArray()
		self.marker_array = MarkerArray()#解决marker发布不同步问题
		self.time = 0
		self.lasertime = 0
		self.cv_image = []
		self.cv_image = np.array(self.cv_image)
		self.center_x = 0
		self.w = 0
		self.disThresh = 2.5
		self.markerid = 0
		self.publishTime = time.time()
		self.positionXYI = np.zeros((1,4))
		self.startEndIndex = np.zeros((1,2))
		self.filterScan = LaserScan()
		self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.cameraSubscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.cameraImage)
		self.imagePublisher = rospy.Publisher("/usb_cam/detect_image", Image, queue_size=1)
		self.filterScanPublisher = rospy.Publisher('slave/filterScan', LaserScan,queue_size=1000)
		self.objectPublisher = rospy.Publisher('/objectPose', PoseArray,queue_size=1)
		self.errorPublisher = rospy.Publisher('/objectErr', Pose2D,queue_size=1)
		self.bbox = rospy.Publisher('position', MarkerArray, queue_size=10)
		cv.namedWindow('TrackBar')
		init_vec = (36, 84, 12, 180, 255, 255)#red in cau(0, 56, 91, 14, 226, 247)#(0, 58, 110, 180, 210, 255)
		cv.createTrackbar('H_min', 'TrackBar', init_vec[0], 180, self.mycallback)
		cv.createTrackbar('H_max', 'TrackBar', init_vec[3], 180, self.mycallback)
		cv.createTrackbar('S_min', 'TrackBar', init_vec[1], 255, self.mycallback)
		cv.createTrackbar('S_max', 'TrackBar', init_vec[4], 255, self.mycallback)
		cv.createTrackbar('V_min', 'TrackBar', init_vec[2], 255, self.mycallback)
		cv.createTrackbar('V_max', 'TrackBar', init_vec[5], 255, self.mycallback)

	def mycallback(self,object):
		pass

	def registerScan(self, scan_data):
		self.header = scan_data.header
		self.ranges = np.array(scan_data.ranges)
		self.angle_increment = scan_data.angle_increment
		self.angle_min = scan_data.angle_min
		self.filterScan = scan_data
		self.filterScan.ranges=[]
		recordFlag = 0
		# recordCount = 0
		startIndex = 0
		endIndex = 0
		minConfidence = -2
		self.positionXYI = np.zeros((1,4))
		self.startEndIndex =  np.zeros((1,2))
		for i in range(self.ranges.size):
			if(self.ranges.size>0):
				if(self.ranges[i] > self.disThresh):
					self.filterScan.ranges.append(0)
				else:
					# self.filterScan.ranges.append(self.ranges[i])
					self.filterScan.ranges.append(0)
		if(not(self.ranges is None) and not(self.cv_image is None)):
			for i in range(1,self.ranges.size-1):
				if(self.ranges[i] != float('inf') and self.ranges[i]!=0):
					angle = self.angle_increment * i + self.angle_min
					if (angle>-45*math.pi/180 and angle<45*math.pi/180):
						u = (int)(1000*self.ranges[i]*math.cos(angle))
						v = (int)(1000*self.ranges[i]*math.sin(angle))
						lamb = u*self.a[6] + v*self.a[7] + self.a[8]
						x = int((u*self.a[0] + v*self.a[1] + self.a[2]) / lamb)
						y = int((u*self.a[3] + v*self.a[4] + self.a[5]) / lamb)
						if(x > self.center_x-self.w/2 and x < self.center_x+self.w/2):
							self.filterScan.ranges[i] = self.ranges[i]
							self.positionXYI = np.row_stack((self.positionXYI,(x,y,i,angle)))
							cv.circle(self.cv_image, (x, y), 2, (0, 255, 255), thickness=2)
						else:
							cv.circle(self.cv_image, (x, y), 2, (255, 255, 0), thickness=-1)
			mask = (self.positionXYI == 0).all(1)
			self.positionXYI = self.positionXYI[~mask,:]
			num_array = len(self.positionXYI) # 计算数组长度
			num_count = 0 # 从0开始计数
			for pos in self.positionXYI:
				num_count += 1
				if(self.ranges[int(pos[2])] != float('inf') and self.ranges[int(pos[2])] != 0):
					if(recordFlag):
						if(abs(self.ranges[int(pos[2])] - self.ranges[lastIndex]) < 0.08 and int(pos[2])-lastIndex < 5):
							endIndex = int(pos[2])
							lastIndex = int(pos[2])
							# recordCount = recordCount + 1
						else:
							# 点集合数量大于5 ，保证点云集合有效
							if(endIndex-startIndex>5):
								self.startEndIndex = np.row_stack((self.startEndIndex,(startIndex,endIndex)))
							# recordCount = 0
							# 如果点集最后一个点不在这个集合，但是在下一个集合的时候要记录为开始索引，如果不在就直接下一次循环找第一个点
							if(self.ranges[int(pos[2])]<self.disThresh):
								startIndex = int(pos[2])
								lastIndex = int(pos[2])
								recordFlag  = 1
							else:
								recordFlag = 0
					# 记录点集第一个点的索引，距离小于2.5米
					else:
						if(self.ranges[int(pos[2])] <self.disThresh):
							startIndex = int(pos[2])
							lastIndex = int(pos[2])
							recordFlag = 1
				if(num_count == num_array): # 当最后一次循环时，判断是否满足点集条件
					if(endIndex-startIndex>5):
						self.startEndIndex = np.row_stack((self.startEndIndex,(startIndex,endIndex)))
			mask = (self.startEndIndex == 0).all(1)
			self.startEndIndex = self.startEndIndex[~mask,:]
			# print(self.startEndIndex)
			for index in self.startEndIndex:
				conf = self.polyContourFit(int(index[0]),int(index[1]),scan_data.angle_min,scan_data.angle_increment)
				if(conf > minConfidence):
					minConfidence = conf
					minIndex = index
			try:
				# for i in range(int(minIndex[0]),int(minIndex[1]+1)):
					# self.filterScan.ranges[i] = 0
				self.calculateErr(int(minIndex[0]+1),int(minIndex[1]-1),scan_data.angle_min,scan_data.angle_increment)
				self.pubulishPose(int(minIndex[0]+1),int(minIndex[1]-1),scan_data.angle_min,scan_data.angle_increment)
			except:
				self.objectPublisher.publish(self.poseArray)
				rospy.logwarn("there is no follow object,using the last pose!!!")
		# self.pubulishPose(int(minIndex[0]+1),int(minIndex[1]-1),scan_data.angle_min,scan_data.angle_increment)
		self.filterScanPublisher.publish(self.filterScan)
		self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))
		# print("laser")
		# print(rospy.Time.now().to_sec())
		# try:
		# 	period = ('{:.3f}'.format((time.time() - self.lasertime)*1000))
		# 	text_time = 'Laser handle period is : ' + str(period) + 'ms'
		# 	self.cv_image = cv.putText(self.cv_image, text_time, (10,450), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv.LINE_AA)
		# 	# 展示图像结果
		# 	cv.rectangle(self.cv_image, (self.x, self.y), (self.x+self.w, self.y + self.h), (0, 255, 0), 4)
		# 	cv.circle(self.cv_image, (self.center_x, self.center_y), 5, (0, 255, 0), -1)
		# 	text = 'The object center is:(' + str(self.center_x) + ',' + str(self.center_y) + ')'
		# 	self.cv_image = cv.putText(self.cv_image, text, (10,410), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv.LINE_AA)
		# 	# cv.imshow('image',self.cv_image)
		# 	key = cv.waitKey(10)
		# 	self.lasertime = time.time()
		# except:
		# 	print("image in function regesitorScan is unreachable")

	def calculateErr(self,start,end,angle_min,angle_delta):
		poseErr = Pose2D()
		for i in range(end-start):
			if(self.ranges[start]==0):
				start += 1
			else:
				# x_start = math.cos((angle_delta * (start)) + angle_min) * self.ranges[start]
				# y_start = math.sin((angle_delta * (start)) + angle_min) * self.ranges[start]
				break
		for i in range(end-start):
			if(self.ranges[end]==0):
				end -= 1
			else:
				# x_end = math.cos((angle_delta * (end)) + angle_min) * self.ranges[end]
				# y_end = math.sin((angle_delta * (end)) + angle_min) * self.ranges[end]
				break
		# length = ((x_start-x_end) ** 2 + (y_start-y_end) ** 2) ** 0.5

		length = math.sqrt(math.pow(self.ranges[start],2) + math.pow(self.ranges[end],2) - 2*self.ranges[start]*self.ranges[end]*math.cos((end-start)*angle_delta))
		alf1 = (math.pow(self.ranges[start],2) +  math.pow(length,2) - math.pow(self.ranges[end],2)) / (2*length*self.ranges[start]) #右侧角度
		alf2 = (math.pow(self.ranges[end],2) +  math.pow(length,2) - math.pow(self.ranges[start],2)) / (2*length*self.ranges[end]) #左侧角度
		y1 = self.ranges[start] * alf1 - (length/2)
		y2 = (length/2) - self.ranges[end] *alf2
		y = (y1+y2)/2
		x1 = -math.sin(math.acos(alf1))*self.ranges[start]
		x2 = -math.sin(math.acos(alf2))*self.ranges[end]
		x = (x1+x2)/2
		l = math.sqrt(math.pow(x,2)+math.pow(y,2))
		theta = math.asin(length/2/l*math.sin(math.acos(alf1)))
		alf = angle_min + angle_delta * start
		yaw_err = (alf+theta-math.atan(y/x))/math.pi*180
		poseErr.x = x
		poseErr.y = y
		poseErr.theta = yaw_err
		self.errorPublisher.publish(poseErr)

	def pubulishPose(self,start,end,angle_min,angle_delta):
		pose = Pose()
		self.poseArray.poses[:] = []
		range_mid = 0
		angle_mid = 0
		count = 0
		for i in range(start,end+1):
			if(self.ranges[i] != float('inf') and self.ranges[i] != 0):
				range_mid += self.ranges[i]
				angle_mid += angle_min + i*angle_delta
				count += 1
		range_mid /= count
		angle_mid /= count
		pose.position.x = math.cos(angle_mid) * range_mid
		pose.position.y = math.sin(angle_mid) * range_mid
		pose.orientation.w = math.cos(angle_mid/2)
		pose.orientation.z = math.sin(angle_mid/2)
		self.poseArray.poses.append(pose)
		self.poseArray.header = self.header
		self.objectPublisher.publish(self.poseArray)
		# xy = self.findObstacle()
		self.visualBox(pose.position.x,pose.position.y)
		# self.visualObstacle(xy)
		self.bbox.publish(self.marker_array)
		self.marker_array = MarkerArray()
		self.markerid = 0
		# print(time.time()-self.publishTime)
		# self.publishTime = time.time()

	def findObstacle(self,):
		startIndex = 0
		endIndex = 0
		recordFlag = 0
		lastIndex = 0
		xy = []
		startend = []
		if(not(self.filterScan.ranges is None)):
			for i in range(1,self.ranges.size):
				if(self.filterScan.ranges[i] != float('inf') and self.filterScan.ranges[i] != 0):
					if(recordFlag):
						if(abs(self.filterScan.ranges[i]-self.filterScan.ranges[lastIndex])<0.25 and i - lastIndex < 8):
							endIndex = i
							lastIndex = i
						else:
							recordFlag = 0
							i = i - 1
							if(endIndex-startIndex>0):
								startend.append([startIndex,endIndex])
					else:
						startIndex = i
						lastIndex = i
						recordFlag = 1
				if(i == self.ranges.size-1): # 当最后一次循环时，判断是否满足点集条件
					if(endIndex-startIndex>1):
						startend.append([startIndex,endIndex])
		for x in startend:
			xmin = self.disThresh
			ymin = self.disThresh
			xmax= -self.disThresh
			ymax = -self.disThresh
			for i in range(x[0],x[1]):
				if(self.filterScan.ranges[i] != 0):
					if(self.filterScan.ranges[i] * math.cos(self.angle_increment*i+self.angle_min) < xmin):
						xmin = self.filterScan.ranges[i] * math.cos(self.angle_increment*i+self.angle_min)
					if(self.filterScan.ranges[i] * math.cos(self.angle_increment*i+self.angle_min) > xmax):
						xmax = self.filterScan.ranges[i] * math.cos(self.angle_increment*i+self.angle_min)
					if(self.filterScan.ranges[i] * math.sin(self.angle_increment*i+self.angle_min) < ymin):
						ymin = self.filterScan.ranges[i] * math.sin(self.angle_increment*i+self.angle_min)
					if(self.filterScan.ranges[i] * math.sin(self.angle_increment*i+self.angle_min) > ymax):
						ymax = self.filterScan.ranges[i] * math.sin(self.angle_increment*i+self.angle_min)
			xy.append([xmin,xmax,ymin,ymax])
		# print(xy)
		return(xy)
		# print(startIndex,endIndex)
		
	def polyContourFit(self,start,end,angle_min,angle_delta): # 提取直线和角点
		for i in range(end-start):
			if(self.ranges[start]==0):
				start += 1
			else:
				x_start = math.cos((angle_delta * (start)) + angle_min) * self.ranges[start]
				y_start = math.sin((angle_delta * (start)) + angle_min) * self.ranges[start]
				break
		for i in range(end-start):
			if(self.ranges[end]==0):
				end -= 1
			else:
				x_end = math.cos((angle_delta * (end)) + angle_min) * self.ranges[end]
				y_end = math.sin((angle_delta * (end)) + angle_min) * self.ranges[end]
				break
		length = ((x_start-x_end) ** 2 + (y_start-y_end) ** 2) ** 0.5
		sumLen = 0
		x_last = x_start
		y_last = y_start
		for i in range(start+1,end):
			if(self.ranges[i] != float('inf') and self.ranges[i] != 0):
				# 获得每个点的x,y值
				x = math.cos((angle_delta * i) + angle_min) * self.ranges[i]
				y = math.sin((angle_delta * i) + angle_min) * self.ranges[i]
				# 计算出相邻两点的距离和，即弧线长
				sumLen += ((x-x_last)**2+(y-y_last)**2)**0.5
				x_last = x
				y_last = y
		if(length>0.1 and length<0.22):
			confidence = ((sumLen/length - 1.1))
		else:
			confidence = -1
		return confidence

	def visualBox(self,X,Y): # 显示目标位置和障碍物
		minx = X - 0.3
		miny = Y - 0.3
		maxx = X + 0.3
		maxy = Y + 0.3
	
		marker=Marker()
		marker.header.frame_id=self.filterScan.header.frame_id
		marker.header.stamp=rospy.Time.now()

		marker.id=self.markerid#每个marker只能有一个id，有重复的id，只会显示一个
		self.markerid += 1
		marker.action=Marker.ADD#表示添加marker
		marker.lifetime=rospy.Duration()#lifetime表示marker在画面中显示的时长;Duration()函数，不给任何参数时，表示一直存在
		marker.type=Marker.LINE_STRIP#所发布marker的类型

		#设定指示线颜色
		marker.color.r=0.0
		marker.color.g=1.0
		marker.color.b=1.0
		marker.color.a=1.0#透明度，1表示完全不透明
		marker.scale.x=0.03#大小，这里表示线的粗细

		#根据激光点云的坐标系来定义2号相机的视野范围
		marker.points=[]
		marker.points.append(Point(minx,miny,0))#Point,属于ros的资料包里面的定义，所以需要导入
		marker.points.append(Point(minx,maxy,0))
		marker.points.append(Point(maxx,maxy,0))
		marker.points.append(Point(maxx,miny,0))
		marker.points.append(Point(minx,miny,0))
		
		self.marker_array.markers.append(marker)#将指示线marker放到MarkerArray中

		marker=Marker()
		marker.header.frame_id=self.filterScan.header.frame_id
		marker.header.stamp=rospy.Time.now()

		marker.id=self.markerid#每个marker只能有一个id，有重复的id，只会显示一个
		self.markerid += 1
		marker.action=Marker.ADD#表示添加marker
		marker.lifetime=rospy.Duration()#lifetime表示marker在画面中显示的时长;Duration()函数，不给任何参数时，表示一直存在
		marker.type=Marker.CYLINDER#所发布marker的类型

		#设定指示线颜色
		marker.color.r=0.0
		marker.color.g=1.0
		marker.color.b=0.0
		marker.color.a=1.0#透明度，1表示完全不透明
		marker.scale.x=0.15 #大小，这里表示线的粗细
		marker.scale.y=0.15 #大小，这里表示线的粗细
		marker.scale.z=0.3#大小，这里表示线的粗细

		#根据激光点云的坐标系来定义2号相机的视野范围
		marker.pose.position.x = X
		marker.pose.position.y = Y
		marker.pose.position.z = 0.15
		marker.points=[]
		self.marker_array.markers.append(marker)#将指示线marker放到MarkerArray中

		marker=Marker()
		marker.header.frame_id=self.filterScan.header.frame_id
		marker.header.stamp=rospy.Time.now()

		marker.id=self.markerid#每个marker只能有一个id，有重复的id，只会显示一个
		self.markerid += 1
		marker.action=Marker.ADD#表示添加marker
		marker.lifetime=rospy.Duration()#lifetime表示marker在画面中显示的时长;Duration()函数，不给任何参数时，表示一直存在
		marker.type=Marker.CYLINDER#所发布marker的类型

		#设定指示线颜色
		marker.color.r=0.0
		marker.color.g=0.0
		marker.color.b=1.0
		marker.color.a=1.0#透明度，1表示完全不透明
		marker.scale.x=0.2 #大小，这里表示线的粗细
		marker.scale.y=0.2 #大小，这里表示线的粗细
		marker.scale.z=0.3#大小，这里表示线的粗细

		#根据激光点云的坐标系来定义2号相机的视野范围
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 0.15
		marker.points=[]
		self.marker_array.markers.append(marker)#将指示线marker放到MarkerArray中
		

	def visualObstacle(self,list): # 显示目标位置和障碍物
		for i in range(len(list)):
			marker=Marker()
			marker.header.frame_id=self.filterScan.header.frame_id
			marker.header.stamp=rospy.Time.now()

			marker.id=self.markerid#每个marker只能有一个id，有重复的id，只会显示一个
			self.markerid += 1
			marker.action=Marker.ADD#表示添加marker
			marker.lifetime=rospy.Duration()#lifetime表示marker在画面中显示的时长;Duration()函数，不给任何参数时，表示一直存在
			marker.type=Marker.CYLINDER#所发布marker的类型

			#设定指示线颜色
			marker.color.r=1.0
			marker.color.g=0.0
			marker.color.b=1.0
			marker.color.a=1.0#透明度，1表示完全不透明
			marker.scale.x=list[i][1]-list[i][0]+0.05 #大小，这里表示线的粗细
			marker.scale.y=list[i][3]-list[i][2]+0.05 #大小，这里表示线的粗细
			marker.scale.z=0.3#大小，这里表示线的粗细

			#根据激光点云的坐标系来定义2号相机的视野范围
			marker.pose.position.x = (list[i][0]+list[i][1])/2
			marker.pose.position.y = (list[i][2]+list[i][3])/2
			marker.pose.position.z = 0.15
			marker.points=[]
			self.marker_array.markers.append(marker)#将指示线marker放到MarkerArray中


	def cameraImage(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))		# 定义结构元素
		self.height, self.width = self.cv_image.shape[0:2]
		self.getTrackbar()
		hsv_frame = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)
		mask = cv.inRange(hsv_frame, self.lower_b, self.upper_b)
		mask2 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)			# 开运算去噪
		mask3 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)			# 闭运算去噪
		if(platform.platform() == 'Linux-5.4.0-42-generic-x86_64-with-Ubuntu-18.04-bionic'):
			_, contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		else:
			contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		try:
			maxArea = 0
			maxIndex = 0
			for i, c in enumerate(contours):
				area = cv.contourArea(c)
				if area > maxArea:
					maxArea = area
					maxIndex = i
			self.x, self.y, self.w, self.h = cv.boundingRect(contours[maxIndex])
			cv.rectangle(self.cv_image, (self.x, self.y), (self.x+self.w, self.y + self.h), (0, 255, 0), 4)
			# 获取中心像素点
			self.center_x = int(self.x + self.w / 2)
			self.center_y = int(self.y + self.h / 2)
			# print("image")
			# print(rospy.Time.now().to_sec())
			# print(self.cv_image.header.stamp)
			# print(minxlist)
			# print(self.center_x,self.center_y)
			cv.circle(self.cv_image, (self.center_x, self.center_y), 5, (0, 255, 0), -1)
			period = ('{:.3f}'.format((time.time() - self.time)*1000))
			text_time = 'Frames handle period is : ' + str(period) + 'ms'
			self.cv_image = cv.putText(self.cv_image, text_time, (10,410), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv.LINE_AA)
			text = 'The object center is:(' + str(self.center_x) + ',' + str(self.center_y) + ')'
			self.cv_image = cv.putText(self.cv_image, text, (10,370), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv.LINE_AA)
			# mask3 = cv.resize(mask3,None,fx=0.25,fy=0.25)
			cv.imshow("TrackBar", mask3)
			self.time = time.time()
		except:
			rospy.loginfo("the image HSV detect is not received")
		key = cv.waitKey(10)
		if(key == ord('a')):
			self.getTrackbar()
			rospy.logfatal_once('threshold logout start')
			rospy.logfatal_once(self.lower_b+self.upper_b)
			rospy.logfatal_once('threshold logout end')

	def getTrackbar(self):
		self.hMin = cv.getTrackbarPos('H_min', 'TrackBar')
		self.hMax = cv.getTrackbarPos('H_max', 'TrackBar')
		self.sMin = cv.getTrackbarPos('S_min', 'TrackBar')
		self.sMax = cv.getTrackbarPos('S_max', 'TrackBar')
		self.vMin = cv.getTrackbarPos('V_min', 'TrackBar')
		self.vMax = cv.getTrackbarPos('V_max', 'TrackBar')
		self.lower_b = (self.hMin, self.sMin,self.vMin)
		self.upper_b = (self.hMax, self.sMax, self.vMax)


if __name__ == '__main__':
	time.sleep(3)
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
