#!/usr/bin/env python
# coding:utf-8

import numpy as np
import math

def sin_change(a):
    return -(math.sin(a/180*math.pi))

def cos_change(a):
    return abs(math.cos(a/180*math.pi))

dis = [463.25,728.25,1249.25,1570.25,1085.25,1326.25]
angle=[341.547,14.6406,12.4531,342.328,324.422,36.3281]

y=[dis[0]*sin_change(angle[0]),dis[1]*sin_change(angle[1]),dis[2]*sin_change(angle[2]),dis[3]*sin_change(angle[3]),dis[4]*sin_change(angle[4]),dis[5]*sin_change(angle[5])];                    # 雷达坐标x轴
x=[dis[0]*cos_change(angle[0]),dis[1]*cos_change(angle[1]),dis[2]*cos_change(angle[2]),dis[3]*cos_change(angle[3]),dis[4]*cos_change(angle[4]),dis[5]*cos_change(angle[5])];                    # 雷达坐标y轴
print(x)
print(y)
u=[251,434,421,268,153,574];                    # 图像坐标u轴
v=[245,260,272,276,268,268];                    # 图像坐标v轴
z=np.array([u[0],v[0],u[1],v[1],u[2],v[2],u[3],v[3],u[4],v[4],u[5],v[5]])                   # 所有u v依次排下去，即（1.4）式等号右边的向量
# X为公式（1.4）中的左边第一个矩阵
X=np.array([
    [x[0],y[0],1,0,0,0,-u[0]*x[0],-u[0]*y[0]],
    [0,0,0,x[0],y[0],1,-v[0]*x[0],-v[0]*y[0]],
    [x[1],y[1],1,0,0,0,-u[1]*x[1],-u[1]*y[1]],
    [0,0,0,x[1],y[1],1,-v[1]*x[1],-v[1]*y[1]],
    [x[2],y[2],1,0,0,0,-u[2]*x[2],-u[2]*y[2]],
    [0,0,0,x[2],y[2],1,-v[2]*x[2],-v[2]*y[2]],
    [x[3],y[3],1,0,0,0,-u[3]*x[3],-u[3]*y[3]],
    [0,0,0,x[3],y[3],1,-v[3]*x[3],-v[3]*y[3]],
    [x[4],y[4],1,0,0,0,-u[4]*x[4],-u[4]*y[4]],
    [0,0,0,x[4],y[4],1,-v[4]*x[4],-v[4]*y[4]],
    [x[5],y[5],1,0,0,0,-u[5]*x[5],-u[5]*y[5]],
    [0,0,0,x[5],y[5],1,-v[5]*x[5],-v[5]*y[5]],
    ])

# print(X)
print(np.linalg.pinv(X))
print(z)
n=np.dot(np.linalg.pinv(X),z)              # 向量n = X^(-1) * z  即X矩阵的逆乘以z向量
print(n)

