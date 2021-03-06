# coding: utf-8
import numpy as np

x=[731.187, 986.965726, 769.7128, 532.8932, 850.62129, 573.36727]
y=[365.04923, 23.690051, -372.8199, 262.978876, -147.11893, -352.86758]
u=[181, 333, 491, 171, 394, 542]
v=[223, 231, 228, 217, 227, 222]
z=[181, 223, 333, 231, 491, 228, 171, 217, 394, 227, 542, 222]

X=[
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
    ]
X=np.array(X)
Y=np.linalg.pinv(X)
z=np.array(z)
print(np.dot(Y,z))
#[-8.60864535e-01 -2.82860510e-02  4.41052891e+02 -4.78533787e-01 -6.74168494e-02  2.37008276e+02 -2.04469923e-03 -3.04210398e-04]
#[-0.861 -0.028286 441.05 -0.4785 -0.0674 237 -0.002 -0.0003 1]
