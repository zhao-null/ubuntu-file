#!/usr/bin/env python
#_*_ encoding:utf-8 _*_

# from _typeshed import Self
from numpy.lib.function_base import angle
import rospy
import threading
import time
import numpy as np
from geometry_msgs.msg import Twist, Vector3,PoseArray
from nav_msgs.msg import Odometry,Path
import math
from std_msgs.msg import Int16

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math


class Follower:
	def __init__(self):
		self.lastSpeed = 0
		self.acclLimit = 0.3
		self.init = 1
		self.startlamb = 0.7
		self.stoplamb = 0.9
		self.Switch = {
			"stop": 0,
			"follow" :1,
			"init": 2
 		}
		self.initflag = 1
		self.status = self.Switch["init"]
		# as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
		self.controllerLossTimer.start()
		self.max_speed = rospy.get_param('~maxSpeed')
		self.max_angle_speed = rospy.get_param('~maxAngleSpeed')

		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)
		self.traFlagPublisher = rospy.Publisher('/trajectory_flag', Int16, queue_size =1)

		# the topic for the tracker that gives us the current position of the object we are following
		self.positionSubscriber = rospy.Subscriber('/objectPose', PoseArray, self.positionUpdateCallback)
		# an info string from that tracker. E.g. telling us if we lost the object
		# self.trackerInfoSubscriber = rospy.Subscriber('/object_tracker/info', StringMsg, self.trackerInfoCallback)

		# PID parameters first is angular, dist
		self.targetDis = rospy.get_param('~targetDis')
		self.targetAngle = rospy.get_param('~targetAngle')
		PID_param = rospy.get_param('~PID_controller')
		# the first parameter is the angular target (0 degrees always) the second is the target distance (say 1 meter)
		self.PID_controller = simplePID([self.targetAngle, self.targetDis], PID_param['P'], PID_param['I'], PID_param['D'])

		# this method gets called when the process is killed with Ctrl+C
		rospy.on_shutdown(self.controllerLoss)

	def trackerInfoCallback(self, info):
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		rospy.logwarn(info.data)
	
	def positionUpdateCallback(self, position):
		# gets called whenever we receive a new position. It will then update the motorcomand

		x = position.poses[-1].position.x
		y = position.poses[-1].position.y
		angleX= math.asin(position.poses[-1].orientation.z) * 2
		distance = (x*x+y*y) ** 0.5
		if(self.status == self.Switch["init"]): # 初始化状态
			rospy.loginfo_once("----follower is initing!!!!!----")
			self.traFlagPublisher.publish(0)
			self.initflag = 1
			linearSpeed = 0
			angularSpeed = 0
			self.lastSpeed = linearSpeed
			if(distance > self.startlamb * self.targetDis):
				self.status = self.Switch["follow"]
		elif(self.status == self.Switch["follow"]): # 跟随状态
			rospy.loginfo_once("----follower is following!!!!!----")
			self.traFlagPublisher.publish(1)
			[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
			angularSpeed = np.clip(-uncliped_ang_speed, -self.max_angle_speed, self.max_angle_speed)
			linearSpeed  = np.clip(uncliped_lin_speed, -self.max_speed, self.max_speed)
			if(angularSpeed>-0.01 and angularSpeed<0.01):
				angularSpeed = 0
			if(linearSpeed>-0.05 and linearSpeed<0.05):
				linearSpeed = 0
			if(linearSpeed - self.lastSpeed > self.acclLimit):
				linearSpeed = self.lastSpeed + self.acclLimit
			if(distance < self.targetDis):
				if(self.initflag):
					linearSpeed = 0.15
			else:
				self.initflag = 0
			self.lastSpeed = linearSpeed
			if(distance < self.stoplamb * self.targetDis and self.initflag == 0 and linearSpeed < 0.3):
				self.status = self.Switch["stop"]
		elif(self.status == self.Switch["stop"]): # 停止状态
			rospy.loginfo_once("----follower is stops!!!!!----")
			self.traFlagPublisher.publish(0)
			linearSpeed = 0
			angularSpeed = 0
			self.lastSpeed = linearSpeed
			
		velocity = Twist()
		velocity.linear = Vector3(linearSpeed,0,0)
		velocity.angular= Vector3(0., 0.,angularSpeed)

		# rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
		self.cmdVelPublisher.publish(velocity)
		
	def stopMoving(self):
		velocity = Twist()
		velocity.linear = Vector3(0.,0.,0.)
		velocity.angular= Vector3(0.,0.,0.)
		self.cmdVelPublisher.publish(velocity)

	def controllerLoss(self):
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		self.active = False
		rospy.loginfo('lost connection')

class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):
		'''Create a discrete PID controller
		each of the parameters may be a vector if they have the same length
		
		Args:
		target (double) -- the target value(s)
		P, I, D (double)-- the PID parameter

		'''
		self.dis = 0.5
		self.dis_err = 0.5
		self.speed = 0.4
		self.x_qual = np.arange(-self.dis, self.dis, 0.1)
		self.x_serv = np.arange(-self.dis_err,self.dis_err, 0.1)
		self.x_output  = np.arange(0, 0.9, 0.1)
		# 定义模糊控制变量
		self.distance = ctrl.Antecedent(self.x_qual, 'distance')
		self.dis_error = ctrl.Antecedent(self.x_serv, 'dis_error')
		self.output = ctrl.Consequent(self.x_output, 'output')
		# 生成模糊隶属函数
		self.distance['L'] = fuzz.trimf(self.x_qual, [-self.dis, -self.dis, 0])  #定义质量差时的三角隶属度函数横坐标
		self.distance['M'] = fuzz.trimf(self.x_qual, [-self.dis, 0, self.dis])
		self.distance['H'] = fuzz.trimf(self.x_qual, [0, self.dis, self.dis])
		self.dis_error['L'] = fuzz.trimf(self.x_serv, [-self.dis_err, -self.dis_err, 0]) #定义服务差时的三角隶属度函数横坐标
		self.dis_error['M'] = fuzz.trimf(self.x_serv, [-self.dis_err, 0, self.dis_err])
		self.dis_error['H'] = fuzz.trimf(self.x_serv, [0, self.dis_err,self.dis_err])
		self.output['L'] = fuzz.trimf(self.x_output, [0, 0, self.speed]) #定义小费的三角隶属度函数横坐标
		self.output['M'] = fuzz.trimf(self.x_output, [0, self.speed, self.speed*2])
		self.output['H'] = fuzz.trimf(self.x_output, [self.speed, self.speed*2, self.speed*2])

		self.output.defuzzify_method='centroid'
		self.rule1=ctrl.Rule(antecedent=((self.distance['L'] & self.dis_error['L'])|(self.distance['L'] & self.dis_error['M'])|(self.distance['M'] & self.dis_error['L'])),consequent=self.output['L'],label='Low')
		self.rule2=ctrl.Rule(antecedent=((self.distance['M']&self.dis_error['M'])|(self.distance['L']&self.dis_error['H'])|(self.distance['H']&self.dis_error['L'])),consequent=self.output['M'],label='Medium')
		self.rule3=ctrl.Rule(antecedent=((self.distance['M']&self.dis_error['H'])|(self.distance['H']&self.dis_error['M'])|(self.distance['H']&self.dis_error['H'])),consequent=self.output['H'],label='High')


		self.outputping_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3])
		self.outputping = ctrl.ControlSystemSimulation(self.outputping_ctrl)
		# check if parameter shapes are compatabile. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=np.zeros(np.size(P))
		self.last_last_error = np.zeros(np.size(P))
		self.integrator = np.zeros(np.size(P))
		self.OutAngle = 0
		self.OutDistance = 0
		# self.OUT = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 

	def ParamsToZero(self,target):
		self.last_error=np.zeros(np.size(target))
		self.last_last_error = np.zeros(np.size(target))
		self.integrator = np.zeros(np.size(target))
		self.OutAngle = 0
		self.OutDistance = 0
		# self.OUT = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 

	def update(self, current_value):
		'''Updates the PID controller. 

		Args:
			current_value (double): vector/number of same legth as the target given in the constructor

		Returns:
			controll signal (double): vector of same length as the target

		'''
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))

		error = self.setPoint - current_value

		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator[0] = self.integrator[0] + (error[0]*deltaT)
		Iangle = self.integrator[0]
		
		# derivative is difference in error / time since last update
		Dangle = (error[0]-self.last_error[0])/deltaT

		self.OutAngle = self.Kp[0]*error[0]+self.Ki[0]*Iangle+self.Kd[0]*Dangle

		# errory = math.sin(error[0])*error[1]
		# self.OutAngle = errory * 2 / (error[1]**2) * self.OutDistance

		# self.OutDistance += self.Kp[1]*(error[1]-self.last_error[1]) + self.Ki[1]*error[1]*deltaT + self.Kd[1]*(error[1] - 2*self.last_error[1] + self.last_last_error[1])/deltaT

		self.outputping.input['distance'] = -error[1]
		print('error')
		print(-error[1])
		self.outputping.input['dis_error'] = -error[1] + self.last_error[1]
		print(-error[1] + self.last_error[1])
		self.outputping.compute()
		self.OutDistance = self.outputping.output['output']
		print(self.outputping.output['output'])

		OUT=[self.OutAngle, self.OutDistance]
		# self.OUT += self.Kp*(error-self.last_error) + self.Ki*error*deltaT + self.Kd*(error - 2*self.last_error + self.last_last_error)/deltaT
		
		# self.last_last_error = self.last_error
		self.last_error = error
		self.timeOfLastCall = currentTime
		# return controll signal
		return OUT
		

if __name__ == '__main__':
	print('starting')
	rospy.init_node('follower')
	follower = Follower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


