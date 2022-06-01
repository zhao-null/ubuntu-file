#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

import rospy
import os
import csv
import codecs
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
from matplotlib.font_manager import FontProperties
from matplotlib.pyplot import MultipleLocator

class dataRecorder:
	def __init__(self):
		self.xerror = []
		self.yerror = []
		self.aerror = []
		self.allerror = []
		self.traFlag = 0
		self.angleX = 0
		self.distance =0
		self.targetDis = rospy.get_param('~targetDis')
		# self.targetAngle = rospy.get_param('~targetAngle')
		self.targetDis = 1.0
		self.targetAngle = 0
		self.objectPoseSubscriber = rospy.Subscriber('/objectErr', Pose2D,self.objectErrCallback)
		self.traFlagSubscriber = rospy.Subscriber('/trajectory_flag', Int16,self.traFlagCallback)

	def objectErrCallback(self, error):
		if(self.traFlag):
			self.x = error.x
			self.y = error.y
			self.angle = error.theta
			self.xerror.append((self.x + self.targetDis)*10)
			self.yerror.append(self.y*10)
			self.aerror.append(self.angle)
			self.allerror.append([self.x+self.targetDis,self.y,self.angle])
		else:
			pass
	
	def traFlagCallback(self,data):
		self.traFlag = data.data


def data_write_csv(datas):#file_name为写入CSV文件的路径，datas为要写入数据列表
    for i in range(100):
        print(('/home/zcy/essay_data/data/' + str(i) + '.csv'))
        if(os.path.isfile('/home/zcy/essay_data/data/' + str(i) + '.csv')):
            pass
        else:
            file_name = '/home/zcy/essay_data/data/' + str(i) + '.csv'
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
	try:
		rospy.spin()
		if(rospy.is_shutdown()):
			data_write_csv(recoder.allerror)
			plt.figure(figsize=(20,18))
			plt.subplot(3,1,1)
			plt.grid()
			plt.plot(recoder.xerror)
			# plt.plot(recoder.timeArray,recoder.angleArray)
			plt.ylabel('x_error/(cm)', horizontalalignment='center',verticalalignment='baseline')
			ax=plt.gca()
			plt.ylim(-30,30)
			x_major_locator=MultipleLocator(10)
			ax.yaxis.set_major_locator(x_major_locator)
			# plt.savefig('/home/zcy/essay_data/picture/xerror.jpg')

			# plt.figure('横向偏移',figsize=(15,5))
			plt.subplot(3,1,2)
			plt.grid()
			plt.plot(recoder.yerror)
			# plt.plot(recoder.timeArray,recoder.angleArray)
			plt.ylabel('y_error/(cm)', horizontalalignment='center',verticalalignment='baseline')
			ax=plt.gca()
			plt.ylim(-30,30)
			x_major_locator=MultipleLocator(10)
			ax.yaxis.set_major_locator(x_major_locator)
			# plt.savefig('/home/zcy/essay_data/picture/yerror.jpg')

			# plt.figure('航向偏移',figsize=(15,5))
			plt.subplot(3,1,3)
			plt.grid()
			plt.plot(recoder.aerror)
			# plt.plot(recoder.timeArray,recoder.angleArray)
			plt.xlabel('data', horizontalalignment='center')
			plt.ylabel('a_error/($^\circ$)', horizontalalignment='center',verticalalignment='baseline')
			ax=plt.gca()
			plt.ylim(-10,10)
			x_major_locator=MultipleLocator(10)
			ax.yaxis.set_major_locator(x_major_locator)

			
			for i in range(100):
				print(('/home/zcy/essay_data/picture/' + str(i) + '.jpg'))
				if(os.path.isfile('/home/zcy/essay_data/picture/' + str(i) + '.jpg')):
					pass
				else:
					file_name = '/home/zcy/essay_data/picture/' + str(i) + '.jpg'
					print(i)
					break
			plt.savefig(file_name)

			# plt.figure('error',figsize=(20,5))
			# plt.subplot(2,1,1)
			# plt.grid()
			# plt.plot(recoder.xerror)
			# # plt.plot(recoder.timeArray,recoder.disArray)
			# plt.ylabel('x/(cm)', horizontalalignment='center',verticalalignment='baseline')
			# # plt.xticks(rotation=90)
			# # plt.xticks([])
			# plt.subplot(2,1,2)
			# plt.grid()
			# plt.plot(recoder.yerror)
			# # plt.plot(recoder.timeArray,recoder.angleArray)
			# plt.xlabel('time/(s)', horizontalalignment='center')
			# plt.ylabel('y/(cm)', horizontalalignment='center',verticalalignment='baseline')
			# # plt.xticks(rotation=90)
			# # plt.xticks(rotation=90)
			# plt.savefig('/home/zcy/picture/error.jpg')



			# plt.figure('odom',figsize=(20,4))
			# plt.grid()
			# plt.xlabel('x/(cm)', horizontalalignment='center')
			# plt.ylabel('y/(cm)',horizontalalignment='left',verticalalignment='bottom')
			# # plt.plot(recoder.xerror,color='blue',label='x_err')
			# # plt.plot(recoder.yerror,color='red',label='y_err')
			# plt.plot(recoder.slavex,recoder.slavey,color='blue',label='slave')
			# #直线
			# # recoder.masterx.append(0.8)
			# # recoder.masterx.append(5)
			# # recoder.mastery.append(0)
			# # recoder.mastery.append(0)
			# plt.xlim(0,400)
			# plt.ylim(-50,400)
			# #弧线
			# plt.plot(recoder.masterx,recoder.mastery,color='red',label='master')
			# plt.legend()
			# plt.savefig('/home/zcy/picture/odom.jpg')
			plt.show()
			rospy.loginfo('the data_recorder is shutdown!')
	except rospy.ROSInterruptException:
		rospy.loginfo('exception')


