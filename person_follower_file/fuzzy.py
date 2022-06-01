# _*_ encoding:utf-8 _*_
import time
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
import math

#  小费范围为[0，25]

angle_x = np.arange(-0.5, 0.5, 0.001)
angle_x_e = np.arange(-0.5, 0.5, 0.001)
angle_output  = np.arange(-2, 2, 0.001)
# 定义模糊控制变量
angle = ctrl.Antecedent(angle_x, 'angle')
ang_error = ctrl.Antecedent(angle_x_e, 'ang_error')
a_output = ctrl.Consequent(angle_output, 'a_output')
# 生成模糊隶属函数
angle['L'] = fuzz.trimf(angle_x, [-0.5, -0.5, 0])  #定义质量差时的三角隶属度函数横坐标
angle['M'] = fuzz.trimf(angle_x, [-0.5, 0, 0.5])
angle['H'] = fuzz.trimf(angle_x, [0, 0.5, 0.5])

ang_error['L'] = fuzz.trimf(angle_x_e, [-0.5, -0.5, 0]) #定义服务差时的三角隶属度函数横坐标
ang_error['M'] = fuzz.trimf(angle_x_e, [-0.5, 0, 0.5])
ang_error['H'] = fuzz.trimf(angle_x_e, [0, 0.5, 0.5])

a_output['L'] = fuzz.trimf(angle_output, [-2, -2, 0]) #定义小费的三角隶属度函数横坐标
a_output['M'] = fuzz.trimf(angle_output, [-2, 0, 2])
a_output['H'] = fuzz.trimf(angle_output, [0, 2, 2])

a_output.defuzzify_method='centroid'
#可视化这些输入输出和隶属函数

#规则
k_rule1=ctrl.Rule(antecedent=((angle['L'] & ang_error['L'])|(angle['L'] & ang_error['M'])|(angle['M'] & ang_error['L'])),consequent=a_output['L'],label='Low')
k_rule2=ctrl.Rule(antecedent=((angle['M']&ang_error['M'])|(angle['L']&ang_error['H'])|(angle['H']&ang_error['L'])),consequent=a_output['M'],label='Medium')
k_rule3=ctrl.Rule(antecedent=((angle['M']&ang_error['H'])|(angle['H']&ang_error['M'])|(angle['H']&ang_error['H'])),consequent=a_output['H'],label='High')

# rule2.view()
k_outputping_ctrl = ctrl.ControlSystem([k_rule1, k_rule2, k_rule3])
k_outputping = ctrl.ControlSystemSimulation(k_outputping_ctrl)

k_outputping.input['angle'] = -0.3
k_outputping.input['ang_error'] = -0.1
a = time.clock()
k_outputping.compute()
print(time.clock()-a)
print(k_outputping.output['a_output'])
print(abs(k_outputping.output['a_output']))




# a_output.view(sim=ang_outputping)
# plt.show()


#----------------------------------------------------#
#距离模糊控制

# #  小费范围为[0，25]
# x_qual = np.arange(-0.5, 0.5, 0.1)
# x_serv = np.arange(-0.5, 0.5, 0.1)
# x_output  = np.arange(0, 0.9, 0.1)
# # 定义模糊控制变量
# angle = ctrl.Antecedent(x_qual, 'distance')
# dis_error = ctrl.Antecedent(x_serv, 'dis_error')
# output = ctrl.Consequent(x_output, 'output')
# # 生成模糊隶属函数
# distance['L'] = fuzz.trimf(x_qual, [-0.5, -0.5, 0])  #定义质量差时的三角隶属度函数横坐标
# distance['M'] = fuzz.trimf(x_qual, [-0.5, 0, 0.5])
# distance['H'] = fuzz.trimf(x_qual, [0, 0.5, 0.5])
# dis_error['L'] = fuzz.trimf(x_serv, [-0.5, -0.5, 0]) #定义服务差时的三角隶属度函数横坐标
# dis_error['M'] = fuzz.trimf(x_serv, [-0.5, 0, 0.5])
# dis_error['H'] = fuzz.trimf(x_serv, [0, 0.5, 0.5])
# output['L'] = fuzz.trimf(x_output, [0, 0, 0.4]) #定义小费的三角隶属度函数横坐标
# output['M'] = fuzz.trimf(x_output, [0, 0.4, 0.8])
# output['H'] = fuzz.trimf(x_output, [0.4, 0.8, 0.8])

# output.defuzzify_method='centroid'
# #可视化这些输入输出和隶属函数

# #规则
# rule1=ctrl.Rule(antecedent=((distance['L'] & dis_error['L'])|(distance['L'] & dis_error['M'])|(distance['M'] & dis_error['L'])),consequent=output['L'],label='Low')
# rule2=ctrl.Rule(antecedent=((distance['M']&dis_error['M'])|(distance['L']&dis_error['H'])|(distance['H']&dis_error['L'])),consequent=output['M'],label='Medium')
# rule3=ctrl.Rule(antecedent=((distance['M']&dis_error['H'])|(distance['H']&dis_error['M'])|(distance['H']&dis_error['H'])),consequent=output['H'],label='High')

# # rule2.view()
# outputping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
# outputping = ctrl.ControlSystemSimulation(outputping_ctrl)


# outputping.input['distance'] = 0
# outputping.input['dis_error'] = 0
# outputping.compute()
# # output.view(sim=outputping)
# # plt.show()
