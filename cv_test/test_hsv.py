# _*_ encoding:utf-8 _*_
import rospy
import cv2 as cv
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
        [self.newYmin,self.newYmax, self.newXmin ,self.newXmax] = [0,480,0,640]
        # self.cmd_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)			# 发布运动控制信息
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)		# 订阅摄像头信息
        cv.namedWindow('TrackBar')
        init_vec = (0, 41, 110, 18, 155, 183)
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
        global hMax,hMin,sMax,sMin,vMax,vMin
        hMin = cv.getTrackbarPos('H_min', 'TrackBar')
        hMax = cv.getTrackbarPos('H_max', 'TrackBar')
        sMin = cv.getTrackbarPos('S_min', 'TrackBar')
        sMax = cv.getTrackbarPos('S_max', 'TrackBar')
        vMin = cv.getTrackbarPos('V_min', 'TrackBar')
        vMax = cv.getTrackbarPos('V_max', 'TrackBar')
        lower_b = (hMin, sMin,vMin)
        upper_b = (hMax, sMax, vMax)
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
        _, contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
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
            cv.rectangle(cv_image, (x + self.newXmin, y + self.newYmin), (x + self.w + self.newXmin, y + self.h + self.newYmin), (255, 0, 0), 2)
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
            pass
        period = ('{:.3f}'.format((time.time() - self.time)*1000))
        text_time = 'Frames handle period is : ' + str(period) + 'ms'
        cv_image = cv.putText(cv_image, text_time, (10,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA) 
        text = 'The object center is:(' + str(self.center_x) + ',' + str(self.center_y) + ')'
        cv_image = cv.putText(cv_image, text, (10,30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA)
		# 显示图像
        cv.imshow("mask", mask3)
        cv.imshow("TrackBar", cv_image)
        self.time = time.time()
        key = cv.waitKey(3)
        if(key == ord('a')):
            self.setNewWindow()
            self.init_flag = 0
            print(lower_b+upper_b)

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
