# -*- coding: utf-8 -*-

import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import logging as LOG

#对比1，2阶滤波器的代码
# unit func implement:
def unit(t):
    r = np.where(t > 1, 1.0, 0.0)
    return r

# step func
def step(t):
    chooseFirst = (t > 1 and t < 5)
    return np.where(chooseFirst, 1.0, 0.0)

# 1 order highpass filter
# high pass filter implemet, 从xn / yn = s / (s+w), 推出一阶高通的表达式为: yn = (2(xn) - 2(xn-1) + 2wT(yn-1))/(2+wT)
inputSrc = [0, 0]
outputSrc = [0, 0]


def high_pass_filter(w_cutoff=1, w_src=0, get_sample_interval=10.0 / 1000):
    wT = w_cutoff * get_sample_interval
    inputSrc[0] = w_src
    outputSrc[0] = (2 * inputSrc[0] - 2 * inputSrc[1] + (2 - wT) * outputSrc[1]) / (2 + wT)

    inputSrc[1] = inputSrc[0]
    outputSrc[1] = outputSrc[0]
    return outputSrc[0]


# 2阶的源表达式为： xn / yn = s^2 / (s^2 + 2wbs + w^2*T^2), w为截至频率，b为阻尼系数
# ref：https://wenku.baidu.com/view/f6020856a76e58fafbb00347.html
inputSrc_2order = [0, 0, 0]
outputSrc_2order = [0, 0, 0]


def high_pass_filter_2order(w_cutoff=1.0, damping_ratio=1.0, w_src=0.0, get_sample_interval=10.0 / 1000):
    A = w_cutoff ** 2 * get_sample_interval ** 2
    B = 4 * w_cutoff * damping_ratio * get_sample_interval
    inputSrc_2order[0] = w_src
    outputSrc_2order[0] = (4 * (inputSrc_2order[0] - 2 * inputSrc_2order[1] + inputSrc_2order[2]) -
                           (2 * A - 8) * outputSrc_2order[1] -
                           (A - B + 4) * outputSrc_2order[2]) / (A + B + 4)

    inputSrc_2order[2] = inputSrc_2order[1]
    inputSrc_2order[1] = inputSrc_2order[0]
    outputSrc_2order[2] = outputSrc_2order[1]
    outputSrc_2order[1] = outputSrc_2order[0]

    return outputSrc_2order[0]

lp_inputSrc_2order = [0,0,0]
lp_outputSrc_2order = [0,0,0]
def low_pass_filter_2order(w_cutoff=1.0, damping_ratio=1.0, w_src=0.0, get_sample_interval=10.0 / 1000):
    A = w_cutoff ** 2 * get_sample_interval ** 2
    B = 4 * w_cutoff * damping_ratio * get_sample_interval
    lp_inputSrc_2order[0] = w_src
    lp_outputSrc_2order[0] = (A * (lp_inputSrc_2order[0] + 2 * lp_inputSrc_2order[1] + lp_inputSrc_2order[2]) -
                           (2 * A - 8) * lp_outputSrc_2order[1] -
                           (A - B + 4) * lp_outputSrc_2order[2]) / (A + B + 4)

    lp_inputSrc_2order[2] = lp_inputSrc_2order[1]
    lp_inputSrc_2order[1] = lp_inputSrc_2order[0]
    lp_outputSrc_2order[2] = lp_outputSrc_2order[1]
    lp_outputSrc_2order[1] = lp_outputSrc_2order[0]

    return lp_outputSrc_2order[0]


t = np.linspace(0, 10.0, 1000)  # 10s分成1000份
inputList = list(map(unit, t))
outputList_1order = []
outputList_2order = []
# calculate with 1 order highpass filter
for i in inputList:
    outputList_1order.append(high_pass_filter(1, i, 0.01))
# calculate with 2 order highpass filter
for i in inputList:
    outputList_2order.append(high_pass_filter_2order(1, 1, i, 0.01))
inputSrc_2order = [0, 0, 0]
outputSrc_2order = [0, 0, 0]


fig, plts = plt.subplots(2,1)
plt1 = plts[0]
plt2 = plts[1]
plt.ylim(-0.2, 1.3)
plt.xlim(0, 10)
plt1.set_title('contrast between 1order and 2order highpass filter', fontsize=15)
# plt1.set_xlabel('time (s)', fontsize=18, fontfamily='sans-serif', fontstyle='italic')
plt1.set_xlabel('time (s)', fontsize='x-large',  fontstyle='italic')
plt1.set_ylabel('w (rad/s)', fontsize='x-large', fontstyle='oblique')

hp_1order, = plt1.plot(t, outputList_1order, linestyle=':')
hp_2order, = plt1.plot(t, outputList_2order, linestyle='--')
unit_signal, = plt1.plot(t, inputList)
plt1.legend([unit_signal, hp_1order, hp_2order], ["unit_signal", "hp_1order", "hp_2order"], loc="center right")

plt1.grid(which='major', axis='both', linestyle="--")

# 图2画的是低通2阶滤波器
impulse_signal = list(map(step,t))
impulse_lp_2order_res = []
for i in impulse_signal:
    impulse_lp_2order_res.append(low_pass_filter_2order(5, 1, i, 0.01))

lp_impulse, = plt2.plot(t, impulse_signal)
lp_impulse_res, = plt2.plot(t, impulse_lp_2order_res, linestyle=':')
plt2.legend([lp_impulse, lp_impulse_res], ["impulse_signal", "impulse_lp_2order_res"], loc="upper right")
plt2.set_xlabel('time (s)', fontsize='x-large',  fontstyle='italic')
plt2.set_ylabel('a (m/s^2)', fontsize='x-large', fontstyle='oblique')
plt2.grid(which='major', axis='both', linestyle="--")
# plt1.plot(inputList, high_pass_filter(1,1,inputList[0],0.01))
plt.show()



