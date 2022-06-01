#!/usr/bin/env python
# _*_ encoding:utf-8 _*_
import platform
import rospy
import cv2 as cv
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
class image_converter:
    def __init__(self):
        self.init_flag = 1
        self.lastX = 0
        self.lastY = 0
        self.time = 0
        self.scale = 2
        self.bridge = CvBridge()
        self.imageSubscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)		# 订阅摄像头信息
        self.anglePublisher = rospy.Publisher("/usb_cam/object_angle",Float32,queue_size=1)
        self.imagePublisher = rospy.Publisher("/usb_cam/detect_image", Image, queue_size=1)
        cv.namedWindow('TrackBar')
        init_vec = (47, 155, 0, 107, 225, 255)
        cv.createTrackbar('H_min', 'TrackBar', init_vec[0], 180, self.mycallback)
        cv.createTrackbar('H_max', 'TrackBar', init_vec[3], 180, self.mycallback)
        cv.createTrackbar('S_min', 'TrackBar', init_vec[1], 255, self.mycallback)
        cv.createTrackbar('S_max', 'TrackBar', init_vec[4], 255, self.mycallback)
        cv.createTrackbar('V_min', 'TrackBar', init_vec[2], 255, self.mycallback)
        cv.createTrackbar('V_max', 'TrackBar', init_vec[5], 255, self.mycallback)
        
    def mycallback(self,object):
        pass

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")			# 获取订阅的摄像头图像
        except CvBridgeError as e:
            print(e)
        # 对图像进行处理
        # self.time = time.time()
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))		# 定义结构元素
        self.height, self.width = cv_image.shape[0:2]
        self.center_x = self.width / 2
        self.center_y = self.height / 2
        if(self.init_flag):
            self.hMin = cv.getTrackbarPos('H_min', 'TrackBar')
            self.hMax = cv.getTrackbarPos('H_max', 'TrackBar')
            self.sMin = cv.getTrackbarPos('S_min', 'TrackBar')
            self.sMax = cv.getTrackbarPos('S_max', 'TrackBar')
            self.vMin = cv.getTrackbarPos('V_min', 'TrackBar')
            self.vMax = cv.getTrackbarPos('V_max', 'TrackBar')
        lower_b = (self.hMin, self.sMin,self.vMin)
        upper_b = (self.hMax, self.sMax, self.vMax)
        # [self.newYmin:self.newYmax, self.newXmax :self.newXmax]
        if(self.init_flag):
            hsv_frame = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)				# 转成HSV颜色空间
        else:
            cv_image_clone = cv_image[self.newYmin:self.newYmax, self.newXmin :self.newXmax]
            hsv_frame = cv.cvtColor(cv_image_clone, cv.COLOR_BGR2HSV)				# 转成HSV颜色空间
        mask = cv.inRange(hsv_frame, lower_b, upper_b)
        mask2 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)			# 开运算去噪
        mask3 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)			# 闭运算去噪
        # 找出面积最大的区域
        if(platform.platform() == 'Linux-5.4.0-42-generic-x86_64-with-Ubuntu-18.04-bionic'):
            _, contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        maxArea = 0
        maxIndex = 0
        for i, c in enumerate(contours):
            area = cv.contourArea(c)
            if area > maxArea:
                maxArea = area
                maxIndex = i
        # 绘制轮廓
        # cv.drawContours(mask3, contours, maxIndex, (255, 255, 0), 2)
        # 获取外切矩形
        try :
            x, y, self.w, self.h = cv.boundingRect(contours[maxIndex])
            cv.rectangle(cv_image, (x + self.newXmin, y + self.newYmin), (x + self.w + self.newXmin, y + self.h + self.newYmin), (0, 255, 0), 4)
            # 获取中心像素点
            self.center_x = int(x + self.w / 2 + self.newXmin)
            self.center_y = int(y + self.h / 2 + self.newYmin)
            cv.circle(cv_image, (self.center_x, self.center_y), 5, (0, 0, 255), -1)
            if(self.init_flag):
                self.lastX = 0
                self.lastY = 0
            else:
                self.lastX = x
                self.lastY = y
                self.setNewWindow()
        except:
            self.w = 0
            self.h = 0
            pass
        if(self.w == 0):
            self.anglePublisher.publish(self.width*2)
        else:
            self.anglePublisher.publish(self.center_x - self.width/2)
        period = ('{:.3f}'.format((time.time() - self.time)*1000))
        text_time = 'Frames handle period is : ' + str(period) + 'ms'
        cv_image = cv.putText(cv_image, text_time, (10,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA) 
        text = 'The object center is:(' + str(self.center_x) + ',' + str(self.center_y) + ')'
        cv_image = cv.putText(cv_image, text, (10,30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA)
		# 显示图像
        mask3 = cv.resize(mask3,None,fx=0.25,fy=0.25)
        cv.imshow("TrackBar", mask3)
        # cv.imshow("image", cv_image)
        self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(cv_image,'bgr8'))
        self.time = time.time()
        key = cv.waitKey(3)
        if(key == ord('a')):
            self.hMin = cv.getTrackbarPos('H_min', 'TrackBar')
            self.hMax = cv.getTrackbarPos('H_max', 'TrackBar')
            self.sMin = cv.getTrackbarPos('S_min', 'TrackBar')
            self.sMax = cv.getTrackbarPos('S_max', 'TrackBar')
            self.vMin = cv.getTrackbarPos('V_min', 'TrackBar')
            self.vMax = cv.getTrackbarPos('V_max', 'TrackBar')
            self.setNewWindow()
            self.init_flag = 0
            rospy.logfatal_once('threshold logout start')
            rospy.logfatal_once(lower_b+upper_b)
            rospy.logfatal_once('threshold logout end')
            # cv.destroyWindow("TrackBar")

    def setNewWindow(self):
        self.newXmin = int(self.center_x - self.w * self.scale)
        if(self.newXmin<0):
            self.newXmin = 0
        self.newXmax = int(self.center_x + self.w * self.scale)
        if(self.newXmax >self.width):
            self.newXmax = self.width -1
        self.newYmin = int(self.center_y - self.h * self.scale)
        if(self.newYmin < 0):
            self.newYmin = 0
        self.newYmax = int(self.center_y + self.h * self.scale)
        if(self.newYmax > self.height):
            self.newYmax = self.height - 1
        if((self.newXmax - self.newYmin < 50) or (self.newXmax - self.newXmin < 50)):
            self.newYmin = 0
            self.newXmin = 0
            self.newXmax = self.width
            self.newYmax = self.height

            

if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv.destroyAllWindows()
