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
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl

class Follower:
	def __init__(self):
		self.lastSpeed = 0
		self.acclLimit = 0.3
		self.init = 1
		self.startlamb = 0.7
		self.stoplamb = 0.95
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
			linearSpeed  = np.clip(-uncliped_lin_speed, 0, self.max_speed)
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

		angle_x = np.arange(-0.5, 0.5, 0.001)
		angle_x_e = np.arange(-0.5, 0.5, 0.001)
		angle_output  = np.arange(-2, 2, 0.001)
		# 定义模糊控制变量
		angle = ctrl.Antecedent(angle_x, 'angle')
		ang_error = ctrl.Antecedent(angle_x_e, 'ang_error')
		a_output = ctrl.Consequent(angle_output, 'a_output')
		# 生成模糊隶属函数
		angle['L'] = fuzz.trimf(angle_x, [-0.5, -0.5, 0])  #定义质量差时的三角隶属度函数横坐标
		angle['M'] = fuzz.trimf(angle_x, [-0.5, 0, 0.5])
		angle['H'] = fuzz.trimf(angle_x, [0, 0.5, 0.5])

		ang_error['L'] = fuzz.trimf(angle_x_e, [-0.5, -0.5, 0]) #定义服务差时的三角隶属度函数横坐标
		ang_error['M'] = fuzz.trimf(angle_x_e, [-0.5, 0, 0.5])
		ang_error['H'] = fuzz.trimf(angle_x_e, [0, 0.5, 0.5])

		a_output['L'] = fuzz.trimf(angle_output, [-2, -2, 0]) #定义小费的三角隶属度函数横坐标
		a_output['M'] = fuzz.trimf(angle_output, [-2, 0, 2])
		a_output['H'] = fuzz.trimf(angle_output, [0, 2, 2])

		a_output.defuzzify_method='centroid'
		#可视化这些输入输出和隶属函数

		#规则
		k_rule1=ctrl.Rule(antecedent=((angle['L'] & ang_error['L'])|(angle['L'] & ang_error['M'])|(angle['M'] & ang_error['L'])),consequent=a_output['L'],label='Low')
		k_rule2=ctrl.Rule(antecedent=((angle['M']&ang_error['M'])|(angle['L']&ang_error['H'])|(angle['H']&ang_error['L'])),consequent=a_output['M'],label='Medium')
		k_rule3=ctrl.Rule(antecedent=((angle['M']&ang_error['H'])|(angle['H']&ang_error['M'])|(angle['H']&ang_error['H'])),consequent=a_output['H'],label='High')

		# rule2.view()
		k_outputping_ctrl = ctrl.ControlSystem([k_rule1, k_rule2, k_rule3])
		self.k_outputping = ctrl.ControlSystemSimulation(k_outputping_ctrl)

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
		print(deltaT)

		# integral of the error is current error * time since last update
		self.integrator[0] = self.integrator[0] + (error[0]*deltaT)
		Iangle = self.integrator[0]
		
		# derivative is difference in error / time since last update
		Dangle = (error[0]-self.last_error[0])/deltaT


		# errory = math.sin(error[0])*error[1]
		# self.OutAngle = errory * 2 / (error[1]**2) * self.OutDistance

		self.OutDistance += self.Kp[1]*(error[1]-self.last_error[1]) + self.Ki[1]*error[1]*deltaT + self.Kd[1]*(error[1] - 2*self.last_error[1] + self.last_last_error[1])/deltaT

		self.k_outputping.input['angle'] = error[0]
		self.k_outputping.input['ang_error'] = error[0] - self.last_error[0]
		self.k_outputping.compute()
		self.apc = self.Kp[0]+abs(self.k_outputping.output['a_output'])

		self.OutAngle = self.apc*error[0]+self.Ki[0]*Iangle+self.Kd[0]*Dangle
		OUT=[self.OutAngle, self.OutDistance]
		# self.OUT += self.Kp*(error-self.last_error) + self.Ki*error*deltaT + self.Kd*(error - 2*self.last_error + self.last_last_error)/deltaT
		
		self.last_last_error = self.last_error
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


