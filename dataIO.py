from washout import WashOut
from motionPlatformArgs import *
import math
import logging as LOG
import numpy as np

class DataInput:
    # raw data
    oriData = {}

    # data input func
    def DataInput(self, filePath):
        # readfile into inData from filePath

        return self.oriData


class DataOutput:
    # data need to output
    inverseKinematicsData = {}
    forwardKinematicsData = {}

    def DataOutput(self):
        # 由于运动平台限制，从洗出算法中获得的有效数据只有，updown，pitch，roll三种
        washOutData = WashOut.GetWashOutData()

        # 为washOutData赋值作测试, 正式的运行时需要删除
        washOutData.pitch = 1.2
        washOutData.roll = 2.1
        washOutData.updown = 6.3

        # -->> kinematics solution：运动学正解
        if math.fabs(washOutData.pitch) > MAX_PITCH_ANGLE:
            LOG.error("the pitch angle is beyond the simulation ability!")
        if math.fabs(washOutData.roll) > MAX_ROLL_ANGLE:
            LOG.error("the roll angle is beyond the simulation ability!")
        if math.fabs(washOutData.updown) > MAX_CYLIDER_LENGTH:
            LOG.error("the updown translation is beyond the simulation ability!")

        # 将角度转为弧度
        washOutData.pitch = washOutData.pitch * math.pi / 180.0
        washOutData.roll = washOutData.roll * math.pi / 180.0

        # 定义三电缸的上端点坐标，初始值a1(280,0,0) a2(-280,280,0) a3(-280,-280,0)
        a1 = np.array([
            [FB_LENGTH / 2.0],
            [0],
            [0],
            [1]
        ])
        a2 = np.array([
            [- FB_LENGTH / 2.0],
            [LR_LENGTH / 2.0],
            [0],
            [1]
        ])
        a3 = np.array([
            [- FB_LENGTH / 2.0],
            [- LR_LENGTH / 2.0],
            [0],
            [1]
        ])
        # 定义三电缸的下端点坐标，初始值b1, b2, b3
        b1 = np.array([
            [FB_LENGTH / 2.0],
            [0],
            [- (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)],
            [1]
        ])
        b2 = np.array([
            [- FB_LENGTH / 2.0],
            [LR_LENGTH / 2.0],
            [- (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)],
            [1]
        ])
        b3 = np.array([
            [- FB_LENGTH / 2.0],
            [- LR_LENGTH / 2.0],
            [- (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)],
            [1]
        ])

        # 设某一状态下，动平台经过了 T (4*4)的矩阵的变化，包括绕x轴的pitch的a角和绕y轴的roll的b角，绕z轴是不旋转的，但是有沿着z轴的平移距离z
        # 这些旋转平移变化参考 https://blog.csdn.net/csxiaoshui/article/details/65446125
        sina = math.sin(washOutData.pitch)
        cosa = math.cos(washOutData.pitch)
        sinb = math.sin(washOutData.roll)
        cosb = math.cos(washOutData.roll)
        z = washOutData.updown

        T = np.array([
            [cosb , sina*sinb, sinb*cosa, 0],
            [0    , cosa     , -sina    , 0],
            [-sinb, sina*cosb, cosa*cosb, z],
            [0    , 0        , 0        , 1]
        ])

        # 经过状态T后的a1',a2',a3'分别如下
        a1_T = np.matmul(T, a1)
        a2_T = np.matmul(T, a2)
        a3_T = np.matmul(T, a3)

        # 根据在状态T的a1,a2,a3的坐标获和定平台的b1,b2,b3的坐标获得电动缸的进动
        h1 = math.sqrt((a1_T[0] - b1[0]) ** 2 + (a1_T[1] - b1[1]) ** 2 + (a1_T[2] - b1[2]) ** 2) - \
             (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)
        h2 = math.sqrt((a2_T[0] - b2[0]) ** 2 + (a2_T[1] - b2[1]) ** 2 + (a2_T[2] - b2[2]) ** 2) - \
             (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)
        h3 = math.sqrt((a3_T[0] - b3[0]) ** 2 + (a3_T[1] - b3[1]) ** 2 + (a3_T[2] - b3[2]) ** 2) - \
             (CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH)

        if math.fabs(h1) > MAX_CYLIDER_LENGTH:
            LOG.error("the forward cylinder is beyond the limit!")
        if math.fabs(h2) > MAX_CYLIDER_LENGTH:
            LOG.error("the left cylinder is beyond the limit!")
        if math.fabs(h3) > MAX_CYLIDER_LENGTH:
            LOG.error("the right cylinder is beyond the limit!")




        return self.inverseKinematicsData
