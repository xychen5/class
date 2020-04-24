# -*- coding: utf-8 -*-

import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import logging as LOG


class BaseFilter:
    def __init__(self, getSampleInterval):
        """
        :param getSampleInterval: 获取样本的间隔时间：单位为s
        """
        if getSampleInterval <= 0:
            LOG.error("get sample interval is not a positive number")
        else:
            self.__getSampleInterval = getSampleInterval
        self.__cutoffFreq = 0
        self.__damping = 0

    def setCutoffFreq(self, cutoffFreq):
        self.__cutoffFreq = cutoffFreq

    def getCutoffFreq(self):
        return self.__cutoffFreq

    def setGetSampleInterval(self, getSampleInterval):
        self.__getSampleInterval = getSampleInterval

    def getGetSampleInterval(self):
        return self.__getSampleInterval

    def setDamping(self, damping):
        self.__damping = damping

    def getDamping(self):
        return self.__damping


# 二阶低通实现持续平动加速模拟
class TransLPFilter(BaseFilter):
    def __init__(self, getSampleInterval):
        super().__init__(getSampleInterval)
        self.__inputSrc_2order = [0, 0, 0]
        self.__outputSrc_2order = [0, 0, 0]

    def filter(self, trans_input):
        """
        低通滤波：
        :param trans_input:输入的平动数据
        :return: 返回滤波后的值
        """
        A = self.getCutoffFreq() ** 2 * self.getGetSampleInterval() ** 2
        B = 4 * self.getCutoffFreq() * self.getDamping() * self.getGetSampleInterval()
        self.__inputSrc_2order[0] = trans_input
        self.__outputSrc_2order[0] = (A * (
                    self.__inputSrc_2order[0] + 2 * self.__inputSrc_2order[1] + self.__inputSrc_2order[2]) -
                                      (2 * A - 8) * self.__outputSrc_2order[1] -
                                      (A - B + 4) * self.__outputSrc_2order[2]) / (A + B + 4)

        self.__inputSrc_2order[2] = self.__inputSrc_2order[1]
        self.__inputSrc_2order[1] = self.__inputSrc_2order[0]
        self.__outputSrc_2order[2] = self.__outputSrc_2order[1]
        self.__outputSrc_2order[1] = self.__outputSrc_2order[0]

        return self.__outputSrc_2order[0]


# 二阶高通实现突发线性加速度模拟
class TransHPFilter(BaseFilter):
    def __init__(self, getSampleInterval):
        super().__init__(getSampleInterval)
        self.__inputSrc_2order = [0, 0, 0]
        self.__outputSrc_2order = [0, 0, 0]

    def filter(self, trans_input):
        """
        低通滤波：
        :param trans_input:输入的平动数据
        :return: 返回滤波后的值
        """
        A = self.getCutoffFreq() ** 2 * self.getGetSampleInterval() ** 2
        B = 4 * self.getCutoffFreq() * self.getDamping() * self.getGetSampleInterval()
        self.__inputSrc_2order[0] = trans_input
        self.__outputSrc_2order[0] = (4 * (
                    self.__inputSrc_2order[0] - 2 * self.__inputSrc_2order[1] + self.__inputSrc_2order[2]) -
                                      (2 * A - 8) * self.__outputSrc_2order[1] -
                                      (A - B + 4) * self.__outputSrc_2order[2]) / (A + B + 4)

        self.__inputSrc_2order[2] = self.__inputSrc_2order[1]
        self.__inputSrc_2order[1] = self.__inputSrc_2order[0]
        self.__outputSrc_2order[2] = self.__outputSrc_2order[1]
        self.__outputSrc_2order[1] = self.__outputSrc_2order[0]

        return self.__outputSrc_2order[0]


# 一阶高通实现旋转的角加速度模拟
# 经典洗出算法使用的是一阶，改成2阶可以回到原点
class RotatHPFilter(BaseFilter):
    def __init__(self, getSampleInterval):
        super().__init__(getSampleInterval)
        self.__inputSrc_1order = [0, 0]
        self.__outputSrc_1order = [0, 0]

    def filter(self, wInput):
        wT = self.getCutoffFreq() * self.getGetSampleInterval()
        self.__inputSrc_1order[0] = wInput
        self.__outputSrc_1order[0] = (2 * self.__inputSrc_1order[0]
                                      - 2 * self.__inputSrc_1order[1]
                                      + (2 - wT) * self.__outputSrc_1order[1]) / (2 + wT)

        self.__inputSrc_1order[1] = self.__inputSrc_1order[0]
        self.__outputSrc_1order[1] = self.__outputSrc_1order[0]
        return self.__outputSrc_1order[0]

# class RotatHPFilter(BaseFilter):
#     def __init__(self, getSampleInterval):
#         super().__init__(getSampleInterval)
#         self.__inputSrc_2order = [0, 0, 0]
#         self.__outputSrc_2order = [0, 0, 0]
#
#     def filter(self, trans_input):
#         """
#         低通滤波：
#         :param trans_input:输入的平动数据
#         :return: 返回滤波后的值
#         """
#         A = self.getCutoffFreq() ** 2 * self.getGetSampleInterval() ** 2
#         B = 4 * self.getCutoffFreq() * self.getDamping() * self.getGetSampleInterval()
#         self.__inputSrc_2order[0] = trans_input
#         self.__outputSrc_2order[0] = (4 * (
#                     self.__inputSrc_2order[0] - 2 * self.__inputSrc_2order[1] + self.__inputSrc_2order[2]) -
#                                       (2 * A - 8) * self.__outputSrc_2order[1] -
#                                       (A - B + 4) * self.__outputSrc_2order[2]) / (A + B + 4)
#
#         self.__inputSrc_2order[2] = self.__inputSrc_2order[1]
#         self.__inputSrc_2order[1] = self.__inputSrc_2order[0]
#         self.__outputSrc_2order[2] = self.__outputSrc_2order[1]
#         self.__outputSrc_2order[1] = self.__outputSrc_2order[0]
#
#         return self.__outputSrc_2order[0]

# 可以看出，每个实例的私有变量是互相不影响的
b = BaseFilter(0.01)
c = BaseFilter(0.03)
print(b.getGetSampleInterval(), "  ", c.getGetSampleInterval())
t = TransLPFilter(0.02)
print(t.getGetSampleInterval())
print(b.getGetSampleInterval(), "  ", c.getGetSampleInterval())
print(b.getCutoffFreq(), "  ", b.getDamping())
b.setGetSampleInterval(0.07)
print(t.getGetSampleInterval())
print(b.getGetSampleInterval(), "  ", c.getGetSampleInterval())
print(b.__dict__)




# # unit func implement:
# def unit(t):
#     r = np.where(t > 1, 1.0, 0.0)
#     return r
#
#
# # 1 order highpass filter
# # high pass filter implemet, 从xn / yn = s / (s+w), 推出一阶高通的表达式为: yn = (2(xn) - 2(xn-1) + 2wT(yn-1))/(2+wT)
# inputSrc = [0, 0]
# outputSrc = [0, 0]
#
#
# def high_pass_filter(w_cutoff=1, w_src=0, get_sample_interval=10.0 / 1000):
#     wT = w_cutoff * get_sample_interval
#     inputSrc[0] = w_src
#     outputSrc[0] = (2 * inputSrc[0] - 2 * inputSrc[1] + (2 - wT) * outputSrc[1]) / (2 + wT)
#
#     inputSrc[1] = inputSrc[0]
#     outputSrc[1] = outputSrc[0]
#     return outputSrc[0]
#
#
# # 2阶的源表达式为： xn / yn = s^2 / (s^2 + 2wbs + w^2*T^2), w为截至频率，b为阻尼系数
# # ref：https://wenku.baidu.com/view/f6020856a76e58fafbb00347.html
# inputSrc_2order = [0, 0, 0]
# outputSrc_2order = [0, 0, 0]
#
#
# def high_pass_filter_2order(w_cutoff=1.0, damping_ratio=1.0, w_src=0.0, get_sample_interval=10.0 / 1000):
#     A = w_cutoff ** 2 * get_sample_interval ** 2
#     B = 4 * w_cutoff * damping_ratio * get_sample_interval
#     inputSrc_2order[0] = w_src
#     outputSrc_2order[0] = (4 * (inputSrc_2order[0] - 2 * inputSrc_2order[1] + inputSrc_2order[2]) -
#                            (2 * A - 8) * outputSrc_2order[1] -
#                            (A - B + 4) * outputSrc_2order[2]) / (A + B + 4)
#
#     inputSrc_2order[2] = inputSrc_2order[1]
#     inputSrc_2order[1] = inputSrc_2order[0]
#     outputSrc_2order[2] = outputSrc_2order[1]
#     outputSrc_2order[1] = outputSrc_2order[0]
#
#     return outputSrc_2order[0]
#
#
# t = np.linspace(0, 10.0, 1000)  # 10s分成1000份
# inputList = list(map(unit, t))
# outputList_1order = []
# outputList_2order = []
# # calculate with 1 order highpass filter
# for i in inputList:
#     outputList_1order.append(high_pass_filter(1, i, 0.01))
# # calculate with 2 order highpass filter
# for i in inputList:
#     outputList_2order.append(high_pass_filter_2order(1, 1, i, 0.01))
#
# fig, plt1 = plt.subplots(figsize=(10, 7))
# plt.ylim(-0.2, 1.3)
# plt.xlim(0, 10)
# plt1.set_title('contrast between 1order and 2order highpass filter', fontsize=18)
# plt1.set_xlabel('time (s)', fontsize=18, fontfamily='sans-serif', fontstyle='italic')
# plt1.set_ylabel('w (rad/s)', fontsize='x-large', fontstyle='oblique')
#
# hp_1order, = plt1.plot(t, outputList_1order)
# hp_2order, = plt1.plot(t, outputList_2order)
# step_signal, = plt1.plot(t, inputList)
# plt1.legend([step_signal, hp_1order, hp_2order], ["step_signal", "hp_1order", "hp_2order"], loc="upper right")
#
# plt.grid(which='major', axis='both', linestyle="--")
# # plt1.plot(inputList, high_pass_filter(1,1,inputList[0],0.01))
# plt.show()
