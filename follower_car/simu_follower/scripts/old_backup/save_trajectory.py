#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

import rospy
import os
import math
import time
import numpy as np
from sensor_msgs.msg import  LaserScan
from geometry_msgs.msg import PoseArray,Pose,Pose2D,Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32
from simu_follower.msg import position as PositionMsg
import matplotlib.pyplot as plt

class dataRecorder:
	def __init__(self):
		# self.disArray = [1,2,3,4,5,6]
		# self.angleArray = [6,5,4,3,2,1]
		# self.timeArray = [1,2,3,4,5,6]
		self.disArray = []
		self.angleArray = []
		self.timeArray = []
		self.x = []
		self.y = []
		self.targetDis = rospy.get_param('~targetDis')
		self.targetAngle = rospy.get_param('~targetAngle')
		# self.angleSubscriber = rospy.Subscriber('/usb_cam/object_angle',Float32,self.angleCallback)
		# self.objectPoseSubscriber = rospy.Subscriber('/turtle1/cmd_vel', Twist,self.disAngleCallback)
		self.objectPoseSubscriber = rospy.Subscriber('/object_tracker/objectPose2D', Pose2D,self.disAngleCallback)
		self.objectPoseSubscriber = rospy.Subscriber('/odom', Odometry,self.odomCallback)

	def disAngleCallback(self, dis_angle_data):
		dis = ((dis_angle_data.x**2+dis_angle_data.y**2)**0.5 - self.targetDis)*1000
		angle = (dis_angle_data.theta - self.targetAngle)/math.pi * 180
		self.disArray.append(dis)
		self.angleArray.append(angle)

	def odomCallback(self,odom_data):
		self.x.append(odom_data.pose.pose.position.x)
		self.y.append(odom_data.pose.pose.position.y)
		# if(self.init_flag):
		# 	dis = (dis_angle_data.x**2+dis_angle_data.y**2)**0.5 - 1.0
		# 	angle = dis_angle_data.theta - 0
		# 	self.time = time.time()
		# 	self.timeArray.append(int(time.time()-self.time))
		# 	self.disArray.append(dis)
		# 	self.angleArray.append(angle)
		# 	# self.disArray.append(dis_angle_data.linear.x)
		# 	# self.angleArray.append(dis_angle_data.angular.z)
		# 	self.init_flag = 0
		# else:
		# 	if(time.time()-self.time > self.lastTime):
		# 		dis = (dis_angle_data.x**2+dis_angle_data.y**2)**0.5 - 1.0
		# 		angle = dis_angle_data.theta - 0
		# 		self.timeArray.append(int(time.time()-self.time))
		# 		self.disArray.append(dis)
		# 		self.angleArray.append(angle)
		# 		# self.disArray.append(dis_angle_data.linear.x)
		# 		# self.angleArray.append(dis_angle_data.angular.z)
		# 		self.lastTime = self.lastTime + 1
		
if __name__ == '__main__':
	rospy.init_node('data_recoder')
	rospy.loginfo("%s starting" % rospy.get_name())
	recoder = dataRecorder()
	rospy.loginfo('It seems to do something!!!')
	try:
		rospy.spin()
		if(rospy.is_shutdown()):
			plt.figure('distance and angle',figsize=(20,5))
			plt.subplot(2,1,1)
			plt.grid()
			plt.plot(recoder.disArray)
			# plt.plot(recoder.timeArray,recoder.disArray)
			plt.ylabel('distance/(mm)', horizontalalignment='center',verticalalignment='baseline')
			# plt.xticks(rotation=90)
			# plt.xticks([])
			plt.subplot(2,1,2)
			plt.grid()
			plt.plot(recoder.angleArray)
			# plt.plot(recoder.timeArray,recoder.angleArray)
			plt.xlabel('time/(s)', horizontalalignment='center')
			plt.ylabel('angle/(degree)', horizontalalignment='center',verticalalignment='baseline')
			# plt.xticks(rotation=90)
			# plt.xticks(rotation=90)
			plt.savefig('/home/zcy/smartcar_ws/src/simu_follower/jpg/dis_angle.jpg')
			plt.figure('odom',figsize=(20,4))
			plt.grid()
			plt.xlabel('x/(m)', horizontalalignment='center')
			plt.ylabel('y/(m)',horizontalalignment='left',verticalalignment='bottom')
			plt.plot(recoder.x,recoder.y)
			plt.savefig('/home/zcy/smartcar_ws/src/simu_follower/jpg/odom.jpg')
			plt.show()
			rospy.loginfo('the data_recorder is shutdown!')
	except rospy.ROSInterruptException:
		rospy.loginfo('exception')


