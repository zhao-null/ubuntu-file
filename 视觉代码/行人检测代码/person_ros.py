# _*_ coding:utf-8 _*_

# 引入所需要的库
from __future__ import print_function  #确保代码同时在Python2.7和Python3上兼容
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils   #安装库pip install imutils ；pip install --upgrade imutils更新版本大于v0.3.1
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time


class person_detect:
    def __init__(self):
        self.bridge = CvBridge()
        # 初始化我们的行人检测器
        #初始化方向梯度直方图描述子
        self.hog = cv2.HOGDescriptor()   
        #设置支持向量机(Support Vector Machine)使得它成为一个预先训练好了的行人检测器
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.imageSubscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback,queue_size=1,buff_size=2**24)
        pass

    def callback(self,data):
        a= time.time()
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        (rects, weights) = self.hog.detectMultiScale(cv_img, winStride=(4, 4), padding=(8, 8), scale=1.2)
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        try:
            # for i in range(1):
            i=0
            cv2.rectangle(cv_img, (pick[i][0], pick[i][1]), (pick[i][2],pick[i][3]), (255, 0, 255), 2)
        except:
            print('none person')
        cv2.imshow("people detection", cv_img)
        print(time.time()-a)
        if cv2.waitKey(1) &0xFF == ord('q'):
            pass

if __name__ == '__main__':
    
    rospy.init_node('img_process_node', anonymous=True)
    rospy.loginfo("%s starting" % rospy.get_name())
    detect = person_detect()
    rospy.loginfo('It seems to do something!!!')
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')

'''
    构造了一个尺度scale=1.05的图像金字塔，以及一个分别在x方向和y方向步长为(4,4)像素大小的滑窗
    scale的尺度设置得越大，在图像金字塔中层的数目就越少，相应的检测速度就越快，但是尺度太大会导致行人出现漏检；
    同样的，如果scale设置得太小，将会急剧的增加图像金字塔的层数，这样不仅耗费计算资源，而且还会急剧地增加检测过程
    中出现的假阳数目(也就是不是行人的被检测成行人)。这表明，scale是在行人检测过程中它是一个重要的参数，
    需要对scale进行调参。我会在后面的文章中对detectMultiScale中的每个参数做些调研。
'''
# detect people in the image：

 #应用非极大抑制方法，通过设置一个阈值来抑制那些重叠的边框
