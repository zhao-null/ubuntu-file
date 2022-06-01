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

img = cv2.imread('/home/zcy/pictures/img1/46.png')
img2 = cv2.imread('/home/zcy/pictures/img2/46.png')

startTime = time.clock()

gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)

endTime = time.clock()
# 输出运行时间
print(endTime-startTime)
# #灰度化
# # #二值化
# ret, thresh = cv2.threshold(gray, 160,190, cv2.THRESH_BINARY_INV)
# min_i = 0
# max_i = 0
trunk_section_point_count=0
mid_list = []
all_mid_list=[]
ground_points =  np.zeros((1,2),dtype=np.int)
tree_points =  np.zeros((1,4),dtype=np.int)

for i in range(1,gray.shape[1]-1):
        if(gray[7,i] == 179):
                if(trunk_section_point_count==0):section_count_start = i
                trunk_section_point_count = trunk_section_point_count+ 1
        else:
                if(trunk_section_point_count>5):
                        mid_list.append((section_count_start+i)/2)
                trunk_section_point_count=0
left_value = 0
right_value = 0
for mid_point in mid_list:
        mid_value = mid_point
        for i in range(7,17):
                last_left_value = left_value
                last_right_value = right_value
                left_value = 0
                right_value = 0
                for j in range(mid_value,mid_value-15,-1):
                        if(gray[i,j]!=179):
                                left_value = j
                                break
                for k in range(mid_value,mid_value+15):
                        if(gray[i,k]!=179):
                                right_value = k
                                break
                mid_value = (left_value + right_value)/2
                if(right_value-left_value<8):
                        ground_points = np.row_stack((ground_points,[i+15,mid_value]))
                        ground_points = np.row_stack((ground_points,[i+14,last_left_value]))
                        ground_points = np.row_stack((ground_points,[i+14,last_right_value]))
                        break
                else:
                        if(i == 16):
                                ground_points = np.row_stack((ground_points,[i+15,left_value]))
                                ground_points = np.row_stack((ground_points,[i+14,last_left_value]))
                                ground_points = np.row_stack((ground_points,[i+14,last_right_value]))
ground_points = np.delete(ground_points,0,0)
print(ground_points)
for a in ground_points:
        cv2.circle(img2, (a[1],a[0]), 1, (0,0,255), 4)

for mid_point in mid_list:
        mid_value = mid_point
        for i in range(22,16,-2):
                left_value = 0
                right_value = 0
                for j in range(mid_value,mid_value-15,-1):
                        if(abs(int(gray2[i,j]) - int(gray2[i,mid_value])) > 1):
                                left_value = j+1
                                cv2.circle(img2, (left_value,i), 1, (255,0,0), 0)
                                break
                for k in range(mid_value,mid_value+15):
                        if(abs(int(gray2[i,k]) - int(gray2[i,mid_value])) > 1):
                                right_value = k-1
                                cv2.circle(img2, (right_value,i), 1, (255,0,0), 0)
                                break
                if(left_value == 0 or right_value == 0):
                        break
                else:
                        tree_points = np.row_stack((tree_points,[i,mid_value,(left_value+right_value)/2,right_value]))
                        mid_value = (left_value + right_value)/2
        for i in range(24,31):
                left_value = 0
                right_value = 0
                mid_value = mid_point
                for j in range(mid_value,mid_value-20,-1):
                        if(abs(int(gray2[i,j]) - int(gray2[i,mid_value])) > 2):
                                left_value = j+1
                                cv2.circle(img2, (left_value,i), 1, (255,0,0), 0)
                                break
                for k in range(mid_value,mid_value+20):
                        if(abs(int(gray2[i,k]) - int(gray2[i,mid_value])) > 2):
                                right_value = k-1
                                cv2.circle(img2, (right_value,i), 1, (255,0,0), 0)
                                break
                if(left_value == 0 or right_value == 0):
                        break
                else:
                        tree_points = np.row_stack((tree_points,[i,left_value,(left_value+right_value)/2,right_value]))
                        mid_value = (left_value + right_value)/2


tree_points = np.delete(tree_points,0,0)
print(tree_points)


# cv2.imshow('thresh',gray)
cv2.imshow('img2',img2)
cv2.imshow('img',img)
cv2.waitKey()
cv2.destroyAllWindows()
