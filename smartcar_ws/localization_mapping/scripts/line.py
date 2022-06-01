# _*_ coding:utf-8 _*_
# 含噪声曲线拟合图
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


##  由空间3维点拟合出一条直线
def linear_fitting_3D_points(points):
    '''
    用直线拟合三维空间数据点。
    
    参考https://www.doc88.com/p-8189740853644.html  
    《三维空间点中基于最小二乘法的分段直线拟合方法》 薛丽红，2015年7月，齐齐哈尔学报，第31卷第4期
    注意; 文中的公式推导有误，k1,b1,k2,b2中的系数2， 应该为n，n表示数据点的个数。
    
    直线方程可以转化成如下形式（具体见上面的文献）：
    x = k1 * z + b1
    y = k2 * z + b2
    
    Input:
        points    ---   List， 三维空间数据点，例如：
                        [[2,3,48],[4,5,50],[5,7,51]]
                    
    返回值是公式系数 k1, b1, k2, b2
    '''
 
    #表示矩阵中的值
    Sum_X=0.0
    Sum_Y=0.0
    Sum_Z=0.0
    Sum_XZ=0.0
    Sum_YZ=0.0
    Sum_Z2=0.0
 
    for i in range(0,len(points)):
        xi=points[i][0]
        yi=points[i][1]
        zi=points[i][2]
 
        Sum_X = Sum_X + xi
        Sum_Y = Sum_Y + yi
        Sum_Z = Sum_Z + zi
        Sum_XZ = Sum_XZ + xi*zi
        Sum_YZ = Sum_YZ + yi*zi
        Sum_Z2 = Sum_Z2 + zi**2
 
    n = len(points) # 点数
    den = n*Sum_Z2 - Sum_Z * Sum_Z # 公式分母
    k1 = (n*Sum_XZ - Sum_X * Sum_Z)/ den
    b1 = (Sum_X - k1 * Sum_Z)/n
    k2 = (n*Sum_YZ - Sum_Y * Sum_Z)/ den
    b2 = (Sum_Y - k2 * Sum_Z)/n
    
    return k1, b1, k2, b2


# 设置图例字号
mpl.rcParams['legend.fontsize'] = 10

# 方式2：设置三维图形模式
fig = plt.figure()
ax = fig.gca(projection='3d')

# 测试数据
# x = np.linspace(-4 * np.pi, 4 * np.pi, 30) 
# y = x + np.random.randn(x.shape[-1]) * 0.7
# z = x * x 
points = [[30,40,0],[30.1,40.2,30],[30.2,40.1,60],[30.3,39.9,90]]
print(linear_fitting_3D_points(points=points))
x = np.array([30,30.1,30.2,30.3])
y = np.array([40,40.2,40.1,39.9])
z = np.array([0,30,60,90])

# 绘制图形
ax.plot(x, y, z, label='parametric1 curve')

# p_xy = np.polyfit(x,y,2);
# y_out = np.polyval(p_xy, x);
# p_xz = np.polyfit(x,z,2);
# z_out = np.polyval(p_xz, x);
# ax.plot(x, y_out, z_out, label='parametric2 curve') 

# p_yx = np.polyfit(y,x,2);
# x_out = np.polyval(p_yx, y);
# p_yz = np.polyfit(y,z,2);
# z_out = np.polyval(p_yz, y);
# ax.plot(x_out, y, z_out, label='parametric3 curve') 

p_zx = np.polyfit(z,x,1);
x_out = np.polyval(p_zx, z);
p_zy = np.polyfit(z,y,1);
y_out = np.polyval(p_zy, z);
ax.plot(x_out, y_out, z, label='parametric4 curve') 

# 显示图例
ax.legend()

# 显示图形
plt.show()

# 拟合是拟合出一个误差小的曲线，这里并不包括光滑，当噪音大时，拟合的曲线不光滑。
