# _*_ encoding:utf-8 _*_
import rospy
import cv2 as cv
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
class image_converter:
    def __init__(self):
        self.time = 0
        # self.cmd_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)			# 发布运动控制信息
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)		# 订阅摄像头信息
        cv.namedWindow('TrackBar')
        init_vec = (0, 44, 83, 84, 112, 167)
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
        self.time = time.time()
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))		# 定义结构元素
        self.height, self.width = cv_image.shape[0:2]
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
        hsv_frame = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)				# 转成HSV颜色空间
        mask = cv.inRange(hsv_frame, lower_b, upper_b)
        mask2 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)			# 开运算去噪
        mask3 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)			# 闭运算去噪
        cv.imshow("mask", mask3)
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
        # cv.drawContours(cv_image, contours, maxIndex, (255, 255, 0), 2)
        # 获取外切矩形
        try :
            x, y, self.w, self.h = cv.boundingRect(contours[maxIndex])
            cv.rectangle(cv_image, (x , y), (x + self.w, y + self.h), (255, 0, 0), 2)
            # 获取中心像素点
            self.center_x = int(x + self.w / 2)
            self.center_y = int(y + self.h / 2)
            cv.circle(cv_image, (self.center_x, self.center_y), 5, (0, 0, 255), -1)
            text = 'The object center is:(' + str(self.center_x) + ',' + str(self.center_y) + ')'
            cv_image = cv.putText(cv_image, text, (10,30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA) 
        except:
            pass
        period = ('{:.3f}'.format((time.time() - self.time)*1000))
        text_time = 'Frames handle period is : ' + str(period) + 'ms'
        cv_image = cv.putText(cv_image, text_time, (10,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA) 
		# 显示图像
        cv.imshow("TrackBar", cv_image)
        key = cv.waitKey(3)
        if(key == ord('a')):
            print(lower_b+upper_b)


if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv.destroyAllWindows()