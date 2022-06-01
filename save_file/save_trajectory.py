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
from std_msgs.msg import Float32,Int16
from simple_follower.msg import position as PositionMsg
import matplotlib.pyplot as plt

class dataRecorder:
	def __init__(self):
		self.disArray = []
		self.angleArray = []
		self.timeArray = []
		self.slavex = []
		self.slavey = []
		self.masterx = []
		self.mastery = []
		self.xerror = []
		self.yerror = []
		self.traFlag = 0
		self.angleX = 0
		self.distance =0
		self.targetDis = rospy.get_param('~targetDis')
		self.targetAngle = rospy.get_param('~targetAngle')
		self.objectPoseSubscriber = rospy.Subscriber('/objectPose', PoseArray,self.disAngleCallback)
		self.objectPoseSubscriber = rospy.Subscriber('/odom', Odometry,self.odomCallback)
		self.traFlagSubscriber = rospy.Subscriber('/trajectory_flag', Int16,self.traFlagCallback)

	def disAngleCallback(self, position):
		if(self.traFlag):
			self.x = position.poses[-1].position.x
			self.y = position.poses[-1].position.y
			self.angleX= math.asin(position.poses[-1].orientation.z) * 2
			self.distance = (self.x*self.x+self.y*self.y) ** 0.5
			dis = (self.distance - self.targetDis)*100
			angle = (self.angleX - self.targetAngle)/math.pi * 180
			self.disArray.append(dis)
			self.angleArray.append(angle)
			self.xerror.append((self.x - self.targetDis*math.cos(self.targetAngle))*100)
			self.yerror.append((self.y - self.targetDis*math.sin(self.targetAngle))*100)
		else:
			pass

	def odomCallback(self,odom_data):
		if(self.traFlag):
			self.slavex.append(odom_data.pose.pose.position.x*100)
			self.slavey.append(odom_data.pose.pose.position.y*100)
			alf = math.asin(odom_data.pose.pose.orientation.z) * 2
			y = self.distance * math.sin(alf+self.angleX)*100
			x = self.distance * math.cos(alf+self.angleX)*100
			try:
				self.masterx.append(x + odom_data.pose.pose.position.x*100)
				self.mastery.append(y + odom_data.pose.pose.position.y*100)
			except:
				rospy.logwarn("no slave's position, plz wait")
		else:
			pass
	
	def traFlagCallback(self,data):
		self.traFlag = data.data

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
			plt.ylabel('distance/(cm)', horizontalalignment='center',verticalalignment='baseline')
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
			plt.savefig('/home/zcy/picture/dis_angle.jpg')

			plt.figure('error',figsize=(20,5))
			plt.subplot(2,1,1)
			plt.grid()
			plt.plot(recoder.xerror)
			# plt.plot(recoder.timeArray,recoder.disArray)
			plt.ylabel('x/(cm)', horizontalalignment='center',verticalalignment='baseline')
			# plt.xticks(rotation=90)
			# plt.xticks([])
			plt.subplot(2,1,2)
			plt.grid()
			plt.plot(recoder.yerror)
			# plt.plot(recoder.timeArray,recoder.angleArray)
			plt.xlabel('time/(s)', horizontalalignment='center')
			plt.ylabel('y/(cm)', horizontalalignment='center',verticalalignment='baseline')
			# plt.xticks(rotation=90)
			# plt.xticks(rotation=90)
			plt.savefig('/home/zcy/picture/error.jpg')



			plt.figure('odom',figsize=(20,4))
			plt.grid()
			plt.xlabel('x/(cm)', horizontalalignment='center')
			plt.ylabel('y/(cm)',horizontalalignment='left',verticalalignment='bottom')
			# plt.plot(recoder.xerror,color='blue',label='x_err')
			# plt.plot(recoder.yerror,color='red',label='y_err')
			plt.plot(recoder.slavex,recoder.slavey,color='blue',label='slave')
			#直线
			# recoder.masterx.append(0.8)
			# recoder.masterx.append(5)
			# recoder.mastery.append(0)
			# recoder.mastery.append(0)
			plt.xlim(0,400)
			plt.ylim(-50,400)
			#弧线
			plt.plot(recoder.masterx,recoder.mastery,color='red',label='master')
			plt.legend()
			plt.savefig('/home/zcy/picture/odom.jpg')
			plt.show()
			rospy.loginfo('the data_recorder is shutdown!')
	except rospy.ROSInterruptException:
		rospy.loginfo('exception')


