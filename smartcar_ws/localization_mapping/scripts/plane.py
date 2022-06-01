# _*_ coding:utf-8 _*_
import csv
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


filename = './1.csv'
with open(filename, 'U') as csvfile:
    reader = csv.reader(csvfile)
    Data = [row for row in reader]
x = []
y = []
z = []
for i in range(len(Data)):
    temp = 0
    Data[i][0] = float(Data[i][0])		# 读取csv文件的内容一般是字符，需转化为浮点数
    x.append(Data[i][0])
    Data[i][1] = float(Data[i][1])
    y.append(Data[i][1])
    Data[i][2] = float(Data[i][2])
    z.append(Data[i][2])
 
 # 绘制3D图  
ax = plt.axes(projection='3d')
ax.plot3D(x, y, z, 'b.')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

## 最小二乘法求解方程
#   创建系数矩阵A和矩阵b
A = np.ones((len(Data), 3))
b = np.zeros((len(Data), 1))
for j in range(len(Data)):
    A[j, 0] = x[j]
    A[j, 1] = y[j]
    b[j, 0] = z[j]

# 通过X=(AT*A)^(-1)*AT*b直接求解
A_T = A.T
A1 = np.dot(A_T, A)
A2 = np.linalg.inv(A1)
A3 = np.dot(A2, A_T)
X = np.dot(A3, b)
print('平面拟合结果为：z = %.3f * x + %.3f * y + %.3f' % (X[0, 0], X[1, 0], X[2, 0]))

