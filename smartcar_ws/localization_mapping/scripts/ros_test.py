#!/usr/bin/env python
#!coding=utf-8
 
#right code !
#write by Zhao Chengyue at 2021.09.22 11:03
#function: 
#display the frame from another node and detect the trees.
 
from time import time
import rospy
import numpy as np
from localization_mapping.msg import point_range_image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

class DetectObject:
    def __init__(self):
        self.count = 0
        self.bridge = CvBridge()

        self.flag = 0
        self.detectTreeBox =  np.zeros((1,6),dtype=np.int)  # 算法检测树干信息的矩阵
        self.detectTreeBoxTemp = np.zeros((1,6),dtype=np.int) # 中间存储每棵树干信息的矩阵
        self.detectTreeBoxFinal = np.zeros((1,6),dtype=np.int) # 最终获得的所有树干信息的矩阵并按照X坐标排序
        self.detectTreeBoxHeight = np.zeros((1,6),dtype=np.int) # 最终获得的所有树干信息的矩阵并按照Y坐标排序
        self.cv_point_range = ()
        self.index = 0
        self.sum = 0

    def rangeImageCallback(self,data):
        startTime = time.time()
        self.count = self.count + 1
        if self.count == 1:
            self.count = 0
            self.cv_img = self.bridge.imgmsg_to_cv2(data.image, "rgb8")
            self.paraClear()
            self.detectObject()
            self.drawObject()
            self.cv_point_range = data.point_range.data
            print('the length of point msg is: %d' % len(self.cv_point_range))
            print('the height of image is: %d' % self.cv_img.shape[0])
            print('the width of image is: %d' % self.cv_img.shape[1])
            # print(self.index)
            # print(self.cv_point_range[self.index])
            # print(self.cv_point_range[self.index+1])
            # print(self.cv_point_range[self.index+2])
            # print(self.cv_point_range[self.index+3])
            # self.sum = self.sum + 1
            # cv2.imwrite('/home/zcy/pictures/img3/'+ str(self.sum) +'.png',self.cv_img)
            with open("/home/zcy/pictures/new.txt", "a") as f:
                f.write(str(self.sum))
                # fwrite(str(self.detectTreeBoxFinal))
                # f.write('_______').
                f.write(str(self.detectTreeBoxHeight))
                f.write('********')
            cv2.imshow("frame" , self.cv_img)
            cv2.waitKey(2)
        else:
            pass
        print('cost time is:%f ms'  % ((time.time()-startTime)*1000))
        print('***********')
    
    def displayRangeImage(self):
        rospy.init_node('webcam_display', anonymous=True)
        # make a video_object and init the video object
        self.count = 0
        self.bridge = CvBridge()
        rospy.Subscriber('point_with_image', point_range_image, self.rangeImageCallback)
        rospy.spin()
    
    def detectObject(self):
        self.gray = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(self.gray, 160,170, cv2.THRESH_BINARY_INV)
        image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for C in contours:
            x, y, w, h = cv2.boundingRect(C)  #计算边界框坐标
            if(x<40 or x+w>self.cv_img.shape[1]-40 or y<self.cv_img.shape[0]/2.5):
                continue
            else:
                if(w>4 and w<30):
                    self.detectTreeBox = np.row_stack((self.detectTreeBox,[int(x),int(y),int(w),int(h),int(x+w/2),int(y+h/2)]))
                    rect = cv2.minAreaRect(C) #计算包围目标的最小矩形区域
                    box = cv2.boxPoints(rect) #计算最小矩形的坐标
                    box = np.int0(box) #坐标变为整数
                    cv2.drawContours(self.cv_img, [box], 0, (255,0,255),2)
        # 根据树的X轴坐标进行排序，获取所有树的树干点信息
        self.detectTreeBox = self.detectTreeBox[np.argsort(self.detectTreeBox[:,4])]
        for i  in range(1,self.detectTreeBox.shape[0]):
                if(self.detectTreeBox[i][4] - self.detectTreeBox[i-1][4]<6 and self.detectTreeBox[i][4] - self.detectTreeBox[i-1][4]>-6):
                        self.detectTreeBoxTemp = np.row_stack((np.int0(self.detectTreeBoxTemp),np.int0(self.detectTreeBox[i-1])))
                        self.flag = self.flag + 1
                else:
                        if(self.flag>2):
                                self.detectTreeBoxTemp = np.row_stack((np.int0(self.detectTreeBoxTemp),np.int0(self.detectTreeBox[i-1])))
                                self.detectTreeBoxTemp = self.detectTreeBoxTemp[np.argsort(self.detectTreeBoxTemp[:,5])]
                                self.detectTreeBoxFinal = np.row_stack((np.int0(self.detectTreeBoxFinal),np.int0(self.detectTreeBoxTemp)))
                                self.flag = 0
                        self.detectTreeBoxTemp = np.empty([1,6])
        # 移除全为0的行

        mask = (self.detectTreeBoxFinal == 0).all(1)
        self.detectTreeBoxFinal = self.detectTreeBoxFinal[~mask,:]
        self.detectTreeBoxHeight = self.detectTreeBoxFinal[np.argsort(self.detectTreeBoxFinal[:,1])]

    def drawObject(self):
        # 画出中心，四分，分界线
        cv2.line(self.cv_img, (self.cv_img.shape[1]/2,0), (self.cv_img.shape[1]/2,self.cv_img.shape[0]), (0,0,255), 3)
        cv2.line(self.cv_img, (self.cv_img.shape[1]/4,0), (self.cv_img.shape[1]/4,self.cv_img.shape[0]), (0,0,255), 3)
        cv2.line(self.cv_img, (self.cv_img.shape[1]/4*3,0), (self.cv_img.shape[1]/4*3,self.cv_img.shape[0]), (0,0,255), 3)
        # 画出树的边缘线

        self.detectTreeBoxFinal = np.row_stack((self.detectTreeBoxFinal,[0,0,0,0,0,0]))
        for i in range(self.detectTreeBoxFinal.shape[0]-1):
            if(self.detectTreeBoxFinal[i+1][0]-self.detectTreeBoxFinal[i][0]<20 and self.detectTreeBoxFinal[i+1][0]-self.detectTreeBoxFinal[i][0]>-20):
                # 画树的左边线
                ptStart = (int(self.detectTreeBoxFinal[i][0]),int(self.detectTreeBoxFinal[i][1]+self.detectTreeBoxFinal[i][3]))
                ptEnd = (int(self.detectTreeBoxFinal[i+1][0]),int(self.detectTreeBoxFinal[i+1][1]))
                cv2.line(self.cv_img,ptStart,ptEnd,(255,0,255),5)
                # 画树的右边线
                ptStart = (int(self.detectTreeBoxFinal[i][0]+self.detectTreeBoxFinal[i][2]),int(self.detectTreeBoxFinal[i][1]+self.detectTreeBoxFinal[i][3]))
                ptEnd = (int(self.detectTreeBoxFinal[i+1][0]+self.detectTreeBoxFinal[i+1][2]),int(self.detectTreeBoxFinal[i+1][1]))
                cv2.line(self.cv_img,ptStart,ptEnd,(255,0,255),5)

    def paraClear(self):
        self.detectTreeBox =  np.zeros((1,6),dtype=np.int)  # 算法检测树干信息的矩阵
        self.detectTreeBoxTemp = np.zeros((1,6),dtype=np.int) # 中间存储每棵树干信息的矩阵
        self.detectTreeBoxFinal = np.zeros((1,6),dtype=np.int) # 最终获得的所有树干信息的矩阵

if __name__ == '__main__':
    
    my_detecter = DetectObject()
    my_detecter.displayRangeImage() 