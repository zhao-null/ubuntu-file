#-*- coding:utf-8 -*-
import cv2
import numpy as np
import time
'''
        created on Tues jan 09:36:30 2018
        @author:ren_dong
        cv2.boundingRect()          边界框即直边界矩形
        cv2.minAreaRect()           最小矩形区域即旋转的边界矩形
        cv2.minEnclosingCircle()    最小闭圆
'''
# 参数设置
flag = 0
detectTreeBox =  np.zeros((1,6),dtype=np.int)  # 算法检测树干信息的矩阵
detectTreeBoxTemp = np.zeros((1,6),dtype=np.int) # 中间存储每棵树干信息的矩阵
detectTreeBoxFinal = np.zeros((1,6),dtype=np.int) # 最终获得的所有树干信息的矩阵
#载入图像img
img = cv2.imread('/home/zcy/pictures/img1/1.png')
startTime = time.clock()
#灰度化
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#二值
ret, thresh = cv2.threshold(gray, 160,170, cv2.THRESH_BINARY_INV)
#寻找轮廓
image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#遍历每一个轮廓
for C in contours:
    #计算边界框坐标
    x, y, w, h = cv2.boundingRect(C)
    if(w>5 and w<30):
        detectTreeBox = np.row_stack((detectTreeBox,[x,y,w,h,(x+w/2),(y+h/2)]))
        #计算包围目标的最小矩形区域
        rect = cv2.minAreaRect(C)
        #计算最小矩形的坐标
        box = cv2.boxPoints(rect)
        #坐标变为整数
        box = np.int0(box)
        #绘制矩形框轮廓  颜色为green  线宽为2
        # cv2.drawContours(img, [box], 0, (255,0,255),2)

# 根据树的X轴坐标进行排序，获取所有树的树干点信息
detectTreeBox = detectTreeBox[np.argsort(detectTreeBox[:,4])]
for i  in range(1,detectTreeBox.shape[0]):
        if(detectTreeBox[i][4] - detectTreeBox[i-1][4]<6):
                detectTreeBoxTemp = np.row_stack((detectTreeBoxTemp,detectTreeBox[i-1]))
                flag = flag + 1
        else:
                if(flag>2):
                        detectTreeBoxTemp = np.row_stack((detectTreeBoxTemp,detectTreeBox[i-1]))
                        detectTreeBoxTemp = detectTreeBoxTemp[np.argsort(detectTreeBoxTemp[:,5])]
                        detectTreeBoxFinal = np.row_stack((detectTreeBoxFinal,detectTreeBoxTemp))
                        detectTreeBoxTemp = np.empty([1,6])
                        flag = 0
# 移除全为0的行
mask = (detectTreeBoxFinal == 0).all(1)
detectTreeBoxFinal = detectTreeBoxFinal[~mask,:]

# 画出中心，四分，分界线
cv2.line(img, (img.shape[1]/2,0), (img.shape[1]/2,img.shape[0]), (0,0,255), 3)
cv2.line(img, (img.shape[1]/4,0), (img.shape[1]/4,img.shape[0]), (0,0,255), 3)
cv2.line(img, (img.shape[1]/4*3,0), (img.shape[1]/4*3,img.shape[0]), (0,0,255), 3)
# 画出树的边缘线

detectTreeBoxFinal = np.row_stack((detectTreeBoxFinal,[0,0,0,0,0,0]))
for i in range(detectTreeBoxFinal.shape[0]-1):
        if(detectTreeBoxFinal[i+1][4]-detectTreeBoxFinal[i][4]<10 and detectTreeBoxFinal[i+1][4]-detectTreeBoxFinal[i][4]>-10):
                # 画树的左边线
                ptStart = (int(detectTreeBoxFinal[i][0]),int(detectTreeBoxFinal[i][1]+detectTreeBoxFinal[i][3]))
                ptEnd = (int(detectTreeBoxFinal[i+1][0]),int(detectTreeBoxFinal[i+1][1]))
                cv2.line(img,ptStart,ptEnd,(255,0,255),5)
                # 画树的右边线
                ptStart = (int(detectTreeBoxFinal[i][0]+detectTreeBoxFinal[i][2]),int(detectTreeBoxFinal[i][1]+detectTreeBoxFinal[i][3]))
                ptEnd = (int(detectTreeBoxFinal[i+1][0]+detectTreeBoxFinal[i+1][2]),int(detectTreeBoxFinal[i+1][1]))
                cv2.line(img,ptStart,ptEnd,(255,0,255),5)
endTime = time.clock()
# 输出运行时间
print(endTime-startTime)
cv2.imshow('thresh',thresh)
cv2.imshow('contours',img)
cv2.waitKey()
cv2.destroyAllWindows()
