#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import threading
import time
import numpy as np
from sensor_msgs.msg import  LaserScan
from geometry_msgs.msg import Twist, Vector3,Pose2D
from std_msgs.msg import String as StringMsg

class Follower:
	def __init__(self):
		# as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
		self.controllerLossTimer.start()
		self.max_speed = rospy.get_param('~maxSpeed') 
		self.max_angle_speed = rospy.get_param('~maxAngleSpeed') 
		self.flag = 1

		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)

		# the topic for the tracker that gives us the current position of the object we are following
		self.positionSubscriber = rospy.Subscriber('/object_tracker/objectPose2D', Pose2D, self.positionUpdateCallback)
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
		
		angleX= position.theta
		distance = (position.x*position.x+position.y*position.y) ** 0.5
		if(self.flag):
			self.lastdistance = distance
			self.time = time.time()
			self.speed = 0
			self.flag = 0
		else:
			self.relativeSpeed = (distance - self.lastdistance)/(time.time()-self.time)
			self.absoluteSpeed = self.relativeSpeed + self.speed
			self.lastdistance = distance

			# rospy.loginfo('Angle: {}, Distance: {}, '.format(angleX, distance))
			# call the PID controller to update it and get new speeds
			[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
			# clip these speeds to be less then the maximal speed specified above
			angularSpeed = np.clip(-uncliped_ang_speed, -self.max_angle_speed, self.max_angle_speed)
			if(angularSpeed>-0.05 and angularSpeed<0.05):
				angularSpeed = 0
			linearSpeed  = np.clip(-uncliped_lin_speed + self.absoluteSpeed, -self.max_speed, self.max_speed)
			if(linearSpeed>-0.05 and linearSpeed<0.05):
				linearSpeed = 0
			# create the Twist message to send to the cmd_vel topic
			velocity = Twist()
			if(angleX == self.targetAngle and distance == self.targetDis):
				velocity.linear = Vector3(0,0,0)
				velocity.angular = Vector3(0,0,0)
			else:
				velocity.linear = Vector3(linearSpeed,0,0)
				velocity.angular= Vector3(0., 0.,angularSpeed)
			# rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
			self.time = time.time()
			self.speed = linearSpeed
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
		
		self.last_error=0
		self.integrator = 0
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
		P =  error
		
		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D
		
if __name__ == '__main__':
	print('starting')
	rospy.init_node('follower')
	follower = Follower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


