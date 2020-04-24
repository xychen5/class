from washout import WashOut
from motionPlatformArgs import *
import math
import logging as LOG
import numpy as np


def deg2rad(deg):
    return deg / 180.0 * math.pi


class DataItem:
    """
    输入数据单元的结构体,取右手系，x轴向前，y轴向作，z轴向上
    pitch 是绕y轴的运动， roll 是绕x轴的运动
    """

    def __init__(self, pitch_ang_val, roll_ang_val, xAcc, yAcc, zAcc):
        self.wPitch = deg2rad(pitch_ang_val)
        self.wRoll = deg2rad(roll_ang_val)
        self.xAcc = xAcc
        self.yAcc = yAcc
        self.zAcc = zAcc


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

    # 牛顿迭代法求解方程的最大迭代步数和误差定义
    MAX_ITERATION_NUM = 100
    MIN_ERROR = 0.0001

    def DataOutput(self):
        # 由于 3 dof 运动平台限制，从洗出算法中获得的有效数据只有，updown，pitch，roll三种
        washOutData = WashOut.getWashOutData()

        # 为washOutData赋值作测试, 正式的运行时需要删除
        washOutData.pitch = 1.2
        washOutData.roll = 2.1
        washOutData.updown = 6.3

        # -->> inverse kinematics：运动学反解
        # 原理： 通过动平台的位置姿态，求出动平台的3个顶点的坐标，然后计算出3个电动缸的进动
        if math.fabs(washOutData.pitch) > MAX_PITCH_ANGLE:
            LOG.error("the pitch angle is beyond the simulation ability!")
        if math.fabs(washOutData.roll) > MAX_ROLL_ANGLE:
            LOG.error("the roll angle is beyond the simulation ability!")
        if math.fabs(washOutData.updown) > MAX_CYLIDER_LENGTH:
            LOG.error("the updown translation is beyond the simulation ability!")

        # 将角度转为弧度
        washOutData.pitch = washOutData.pitch * math.pi / 180.0
        washOutData.roll = washOutData.roll * math.pi / 180.0

        # 建立坐标系，其原点位于等腰三角形的底部的高的一半，x轴垂直于三角形的底
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
            [cosb, sina * sinb, sinb * cosa, 0],
            [0, cosa, -sina, 0],
            [-sinb, sina * cosb, cosa * cosb, z],
            [0, 0, 0, 1]
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

        # -->> forward kinematics：运动学正解，目标：根据电动缸的进动求出动平台的坐标a1,a2,a3
        # 原理：首先3个电动缸位于的平面是不会改变的，那么要求的坐标包含的9个变量变为6个，
        # a1为（xa1, 0, za1），a2为（xa2, -xa2, za2）,a3为（xa3, xa3, za3）
        # 然后根据|a1a2|, |a2a3|, |a1a3|均为已知定值，为3个方程，然后是|a1b1|,|a2b2|,|a3b3|的值均为已知，又可以列3个方程
        # 方程组如下：            dfb = FB_LENGTH, dlr = LR_LENGTH, ct = CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH
        # |a1b1| = (xa1 - dfb/2)**2 + (za1 + ct)**2 - (h1 + ct)**2 = 0
        # |a2b2| = (xa2 + dfb/2)**2 + (xa2 + dlr/2)**2 + (za2 + ct)**2 - (h2 + ct)**2 = 0
        # |a3b3| = (xa3 + dfb/2)**2 + (xa3 + dlr/2)**2 + (za3 + ct)**2 - (h3 + ct)**2 = 0
        # |a1a2| = (xa1 - xa2)**2 + xa2**2 + (za1 - za2)**2 - [(dfb/2)**2 + dfb**2] = 0
        # |a1a3| = (xa1 - xa3)**2 + xa3**2 + (za1 - za3)**2 - [(dfb/2)**2 + dfb**2] = 0
        # |a2a3| = (xa2 - xa3)**2 + (xa2 + xa3)**2 + (za2 - za3)**2 - dlr**2 = 0
        # 使用牛顿迭代法求解该方程组
        dfb = FB_LENGTH
        dlr = LR_LENGTH
        ct = CYLIDER_HEIGHT + MAX_CYLIDER_LENGTH
        d1 = h1 + ct
        d2 = h2 + ct
        d3 = h3 + ct

        # 为初始的x(k)赋值，x为[xa1, za1, xa2, za2, xa3, za3]的转置的列向量，为以上方程的解,k为迭代次数，一开始为0
        xk = np.array([
            [FB_LENGTH / 2.0],
            [0],
            [- FB_LENGTH / 2.0],
            [0],
            [- FB_LENGTH / 2.0],
            [0]
        ])
        # xkp1 为 x(k+1)， 一开始初始化成和x(k)一样的即可
        xkp1 = xk

        # 开始迭代求解：
        calculate_error = 0
        for i in range(self.MAX_ITERATION_NUM):
            if calculate_error < 6 * self.MAX_ITERATION_NUM:
                break

            # xk时F(X)的值为：
            Fxk = np.array([
                [(xk[0] - dfb / 2) ** 2 + (xk[1] + ct) ** 2 - d1 ** 2],
                [(xk[2] + dfb / 2) ** 2 + (xk[2] + dlr / 2) ** 2 + (xk[3] + ct) ** 2 - d2 ** 2],
                [(xk[4] + dfb / 2) ** 2 + (xk[4] + dlr / 2) ** 2 + (xk[5] + ct) ** 2 - d3 ** 2],
                [(xk[0] - xk[2]) ** 2 + xk[2] ** 2 + (xk[1] - xk[3]) ** 2 - (dfb / 2) ** 2 - dfb ** 2],
                [(xk[0] - xk[4]) ** 2 + xk[4] ** 2 + (xk[1] - xk[5]) ** 2 - (dfb / 2) ** 2 - dfb ** 2],
                [(xk[2] - xk[4]) ** 2 + (xk[2] + xk[4]) ** 2 + (xk[3] - xk[5]) ** 2 - dlr ** 2]
            ])

            # 构造jacobii矩阵， 方程组计为： F(X) = 0; 那么，jacobii矩阵 = F‘(X) = D(F(X))
            DFX = np.array([
                [2 * xk[0] - dfb, 2 * (xk[1] + ct), 0, 0, 0, 0],
                [0, 0, 4 * xk[2] + dfb + dlr, 2 * (xk[3] + ct), 0, 0],
                [0, 0, 0, 0, 4 * xk[4] + dfb + dlr, 2 * (xk[5] + ct)],
                [2 * (xk[0] - xk[2]), 2 * (xk[1] - xk[5]), 4 * xk[2] - 2 * xk[0], 2 * (xk[3] - xk[1]), 0, 0],
                [2 * (xk[0] - xk[4]), 2 * (xk[1] - xk[5]), 0, 0, 4 * xk[4] - 2 * xk[0], 2 * (xk[5] - xk[1])],
                [0, 0, 4 * xk[2], 2 * (xk[3] - xk[5]), 4 * xk[4], 2 * (xk[5] - xk[3])]
            ])

            DFX_INV = np.linalg.inv(DFX)
            # 牛顿迭代公式： x(k+1) = x(k) - [F'[x(k)]]^(-1) * F(x(k))
            xkp1 = xk - np.matmul(DFX_INV, Fxk)
            # 计算误差
            for i in range(6):
                calculate_error = math.fabs(calculate_error + xkp1[i] - xk[i])
            # 更新当前的 xk
            xk = xkp1

        # 求解后，xk即为新的三个顶点的值
        # 需要注意改进的地方： 反解里的三个顶点的坐标值会每次都变，注意修改，把他们加入到类的属性之后注意他们值的变化
        # 正解里的x(0)的值的确定

        return self.inverseKinematicsData
