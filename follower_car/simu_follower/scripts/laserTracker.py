#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

from numpy.lib.function_base import percentile
import rospy
import math
import threading
import time
import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import  LaserScan
from geometry_msgs.msg import PoseArray,Pose,Pose2D,Twist, Vector3,Point,PoseStamped
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32
from simu_follower.msg import position as PositionMsg
from nav_msgs.msg import Odometry,Path
from kalmanfilter import KalmanFilter

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
		self.scanSubscriber = rospy.Subscriber('slave/scan', LaserScan, self.registerScan)
		self.odomSubscriber = rospy.Subscriber('slave/odom', Odometry, self.odomCallback)
		self.filterScanPublisher = rospy.Publisher('slave/filterScan', LaserScan,queue_size=1000)
		self.objectPosePublisher = rospy.Publisher('/object_tracker/objectPoseArray', PoseArray,queue_size=1)
		self.objectDisPublisher = rospy.Publisher('/object_tracker/objectPose2D', Pose2D,queue_size=1)
		self.bbox = rospy.Publisher('position', MarkerArray, queue_size=10)
		self.masterPathPublisher = rospy.Publisher('slave_master_trajectory', Path, queue_size=10)
		self.poseParaToZero()
		self.odomMsg = Odometry()
		self.masterPath = Path()
		self.kf = KalmanFilter()

	def angleCallback(self,data):
		self.angle_cam.append(data.data)
		pass

	def odomCallback(self,odom):
		# self.odomMsg = Odometry()
		self.odomMsg.pose.pose.position = odom.pose.pose.position
		self.odomMsg.pose.pose.orientation = odom.pose.pose.orientation

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

	def posePubSet(self): # ????????????????????????????????????objectPoseArray?????????
		self.pose = Pose()
		self.range_mid = self.range_sum / self.count
		self.angle_mid = self.angle_sum / self.count
		self.pose.position.x = math.cos(self.angle_mid) * self.range_mid
		self.pose.position.y = math.sin(self.angle_mid) * self.range_mid
		self.pose.orientation.w = math.cos(self.angle_mid/2)
		self.pose.orientation.z = math.sin(self.angle_mid/2)
		self.objectPose.poses.append(self.pose)

	def polyContourFit(self,start,end,angle_min,angle_delta,Eps): # ?????????????????????
		# min_x = 12
		# min_y = 12
		# max_x = -12
		# max_y = -12
		x_start = math.cos((angle_delta * start) + angle_min) * self.ranges[start]
		y_start = math.sin((angle_delta * start) + angle_min) * self.ranges[start]
		x_end = math.cos((angle_delta * end) + angle_min) * self.ranges[end]
		y_end = math.sin((angle_delta * end) + angle_min) * self.ranges[end]
		length = math.sqrt((x_start-x_end) ** 2 + (y_start-y_end) ** 2)
		cosTheta = (x_end - x_start) / length
		sinTheta = -(y_end - y_start) / length
		sumDis = 0
		sumLen = 0
		x_last = x_start
		y_last = y_start
		for i in range(start+1,end):
			if(self.ranges[i] != float('inf')):
				# ??????????????????x,y???
				x = math.cos((angle_delta * i) + angle_min) * self.ranges[i]
				y = math.sin((angle_delta * i) + angle_min) * self.ranges[i]
				# ????????????????????????????????????????????????
				sumLen += ((x-x_last)**2+(y-y_last)**2)**0.5
				x_last = x
				y_last = y
				# ??????????????????????????????????????????????????????
				sumDis += abs((y - y_start) * cosTheta + (x - x_start)* sinTheta)
		sumDis = sumDis / length #????????????????????????????????????????????????????????????????????????????????????
		sumLen = sumLen / length #????????????????????????????????????????????????????????????1????????????????????????1???????????????
		if(length>0.05 and length<0.15 and sumLen>0.95 and sumLen<1.55 and sumDis<5 and sumDis>0): # 0.18 1.2 12
			self.posePubSet() # ???????????????????????????

	def poseSelect(self): # ????????????????????????????????????????????????
		tempAngle = 3.14
		if(self.lastDis==0 and self.lastAngle==0):# ??????????????????
				for pose in self.objectPose.poses:
					if(2*math.acos(pose.orientation.w)<60.0*math.pi/180.0):
						angle = math.asin(pose.orientation.z)
						if(abs(angle) < (tempAngle)):
							tempAngle = abs(angle) 
							self.objectDis.x = pose.position.x
							self.objectDis.y = pose.position.y
							self.objectDis.theta = math.asin(pose.orientation.z)*2
							self.lastAngle = self.objectDis.theta
							self.lastDis = (pose.position.x*pose.position.x + pose.position.y*pose.position.y) ** 0.5
							rospy.loginfo('laserTracker.py: Follow object is initinated OK')
		else:
			if(len(self.objectPose.poses)>0):
				for pose in self.objectPose.poses:
					if(2*math.acos(pose.orientation.w)<60.0*math.pi/180.0):
						angle = math.asin(pose.orientation.z)
						if(abs(angle)< (tempAngle)):
							tempAngle = abs(angle)
							self.objectDis.x = pose.position.x
							self.objectDis.y = pose.position.y
							self.objectDis.theta = math.asin(pose.orientation.z)*2
				if(abs((self.objectDis.x**2 + self.objectDis.y**2) ** 0.5 - self.lastDis) < 0.5 and abs(self.objectDis.theta - self.lastAngle) < math.pi/5):
					self.lastAngle = self.objectDis.theta
					self.lastDis = (self.objectDis.x**2 + self.objectDis.y**2) ** 0.5
					rospy.loginfo_once('laserTracker.py: Everything is OK')
				else:
					self.using = self.using + 1
					if(self.using <4):
						self.objectDis.x = self.lastDis * math.cos(self.lastAngle)
						self.objectDis.y = self.lastDis * math.sin(self.lastAngle)
						self.objectDis.theta = self.lastAngle
						rospy.logwarn('laserTracker.py: This time and last time is too big, using the last pose follower')
					else:
						self.objectXYThetaToZero()
						self.using = 0
						rospy.logerr('laserTracker.py: Laser lost the object. Laser lost. Reset!')
			else:
				rospy.logerr('laserTracker.py: Laser lost all object, maybe has problem!')
				self.objectXYThetaToZero()
		p = (self.objectDis.x+0.5 ** 2 + self.objectDis.y ** 2)**0.5 - 0.07
		theta = self.objectDis.theta
		alf = math.asin(self.odomMsg.pose.pose.orientation.z)*2
		y = p * math.sin(alf+theta)
		x = p * math.cos(alf+theta)
		odom_master_x = x+self.odomMsg.pose.pose.position.x
		odom_master_y = y+self.odomMsg.pose.pose.position.y
		predicted = self.kf.predict(odom_master_x,odom_master_y)
		odom_master_x = predicted[0]
		odom_master_y = predicted[1]
		self.visualBox(odom_master_x,odom_master_y,odom_master_x,odom_master_y)
		this_pose_stamped = PoseStamped()
		this_pose_stamped.pose.position.x = odom_master_x
		this_pose_stamped.pose.position.y = odom_master_y
		this_pose_stamped.header.stamp = rospy.Time.now()
		this_pose_stamped.header.frame_id = "slave/odom"
		self.masterPath.poses.append(this_pose_stamped)
		self.masterPath.header.stamp = rospy.Time.now()
		self.masterPath.header.frame_id = "slave/odom"
		self.masterPathPublisher.publish(self.masterPath)
		self.objectDisPublisher.publish(self.objectDis)

	def visualBox(self,minx,miny,maxx,maxy):
		minx = minx - 0.3
		miny = miny - 0.3
		maxx = maxx + 0.3
		maxy = maxy + 0.3
		marker_array=MarkerArray()#??????marker?????????????????????

		marker=Marker()
		marker.header.frame_id="slave/odom"
		marker.header.stamp=rospy.Time.now()

		marker.id=0#??????marker???????????????id???????????????id?????????????????????
		marker.action=Marker.ADD#????????????marker
		marker.lifetime=rospy.Duration()#lifetime??????marker???????????????????????????;Duration()???????????????????????????????????????????????????
		marker.type=Marker.LINE_STRIP#?????????marker?????????

		#?????????????????????
		marker.color.r=0.0
		marker.color.g=1.0
		marker.color.b=1.0
		marker.color.a=1.0#????????????1?????????????????????
		marker.scale.x=0.03#?????????????????????????????????

		#???????????????????????????????????????2????????????????????????
		marker.points=[]
		marker.points.append(Point(minx,miny,0.519))#Point,??????ros????????????????????????????????????????????????
		marker.points.append(Point(minx,maxy,0.519))
		marker.points.append(Point(maxx,maxy,0.519))
		marker.points.append(Point(maxx,miny,0.519))
		marker.points.append(Point(minx,miny,0.519))

		marker_array.markers.append(marker)#????????????marker??????MarkerArray???
		# print(marker_array)

		self.bbox.publish(marker_array)
		marker_array = []

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
							# ????????????????????????10 ?????????????????????????????????????????????????????????1/2???????????????????????????
							if(endIndex-startIndex>5 and recordCount>int((endIndex-startIndex)*0.6)):
								self.startEndIndex = np.row_stack((self.startEndIndex,(startIndex,endIndex)))
							recordCount = 0
							# ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
							if(self.ranges[i]<self.disThresh):
								startIndex = i
								lastIndex = i
								recordFlag  = 1
							else:
								recordFlag = 0
					# ????????????????????????????????????????????????2.5???
					else:
						if(self.ranges[i] <self.disThresh):
							startIndex = i
							lastIndex = i
							recordFlag = 1
				else:
					pass
		# ????????????0?????????
		mask = (self.startEndIndex == 0).all(1)
		self.startEndIndex = self.startEndIndex[~mask,:]
		# ??????????????????????????????
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
		# ??????????????????????????????????????????
		for i in range(self.ranges.size+1):
			filterScanMsg.ranges.append(float('inf'))
		# ???????????????????????????????????????????????????????????????????????????
		for index in self.startEndIndex:
			for i in range(int(index[0]),int(index[1]+1)):
				if(self.ranges[i] != float('inf')):
					filterScanMsg.ranges[i] = self.ranges[i] # set new laser scan topic of the detect laser group 
					self.range_sum = self.range_sum + self.ranges[i]
					self.angle_sum = self.angle_sum + self.angle_increment * i + scan_data.angle_min
					self.count = self.count + 1
			# ????????????
			self.polyContourFit(start=int(index[0]),end=int(index[1]),angle_min=scan_data.angle_min,angle_delta=self.angle_increment,Eps=0.1)
			# # ???????????????????????????????????????????????????????????????
			# self.posePubSet(start=int(index[0]),end=int(index[1]))
			# ????????????
			self.poseParaToZero()
		# ???????????????????????????
		self.filterScanPublisher.publish(filterScanMsg)
		# ??????????????????????????????
		self.objectPose.header = scan_data.header
		self.objectPosePublisher.publish(self.objectPose)
		# ??????????????????
		self.poseSelect()
		# ??????????????????
		# print(self.startEndIndex)
		# print(time.time())
		# print('***********************************')
		# ???????????????????????????
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


