from motionPlatformArgs import *
import math
import logging as LOG
import numpy as np

class InverseKinematics:
    # data need to output
    inverseKinematicsData = {}
    forwardKinematicsData = {}

    # 牛顿迭代法求解方程的最大迭代步数和误差定义
    MAX_ITERATION_NUM = 100
    MIN_ERROR = 0.0001

    def DataOutput(self, washOutData):
        """
        将洗出算法的输出，进行运动学的正解反解开
        :param washOutData: 洗出算法的输出数据，包含3个方向的旋转角度，和3个方向的平移
        :return:
        """
        # 由于 3 dof 运动平台限制，从洗出算法中获得的有效数据只有，updown，pitch，roll三种
        # ----------------------------------------------
        # -->> inverse kinematics：运动学反解
        # ----------------------------------------------
        # 原理： 通过动平台的位置姿态，求出动平台的3个顶点的坐标，然后计算出3个电动缸的进动
        if math.fabs(washOutData["pitch"]) > MAX_PITCH_ANGLE:
            LOG.error("the pitch angle is beyond the simulation ability!")
        if math.fabs(washOutData["roll"]) > MAX_ROLL_ANGLE:
            LOG.error("the roll angle is beyond the simulation ability!")
        if math.fabs(washOutData["updown"]) > MAX_CYLIDER_LENGTH:
            LOG.error("the updown translation is beyond the simulation ability!")

        # 将角度转为弧度
        washOutData["pitch"] = washOutData["pitch"] * math.pi / 180.0
        washOutData["roll"] = washOutData["roll"] * math.pi / 180.0

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

        # 设某一状态下，动平台经过了 T (4*4)的矩阵的变化，包括绕x轴的roll的a角和绕y轴的pitch的b角，绕z轴是不旋转的，但是有沿着z轴的平移距离z
        # 这些旋转平移变化参考 https://blog.csdn.net/csxiaoshui/article/details/65446125
        sinb = math.sin(washOutData["pitch"])
        cosb = math.cos(washOutData["pitch"])
        sina = math.sin(washOutData["roll"])
        cosa = math.cos(washOutData["roll"])
        z = washOutData["updown"]

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

        # print ("h1: %f\n h2: %f\n h3: %f\n"%(h1, h2, h3))

        if math.fabs(h1) > MAX_CYLIDER_LENGTH:
            LOG.error("the forward cylinder is beyond the limit!")
        if math.fabs(h2) > MAX_CYLIDER_LENGTH:
            LOG.error("the left cylinder is beyond the limit!")
        if math.fabs(h3) > MAX_CYLIDER_LENGTH:
            LOG.error("the right cylinder is beyond the limit!")

        # ----------------------------------------------
        # -->> forward kinematics：运动学正解，目标：根据电动缸的进动求出动平台的坐标a1,a2,a3
        # ----------------------------------------------
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
        calculate_error = 1  # 初始误差值大于 6 * self.MIN_ERROR 即可
        for i in range(self.MAX_ITERATION_NUM):
            if calculate_error < 6 * self.MIN_ERROR:
                break

            calculate_error = 0
            # xk时F(X)的值为：
            Fxk = np.array([
                [(xk[0][0] - dfb / 2) ** 2 + (xk[1][0] + ct) ** 2 - d1 ** 2],
                [(xk[2][0] + dfb / 2) ** 2 + (xk[2][0] + dlr / 2) ** 2 + (xk[3][0] + ct) ** 2 - d2 ** 2],
                [(xk[4][0] + dfb / 2) ** 2 + (xk[4][0] + dlr / 2) ** 2 + (xk[5][0] + ct) ** 2 - d3 ** 2],
                [(xk[0][0] - xk[2][0]) ** 2 + xk[2][0] ** 2 + (xk[1][0] - xk[3][0]) ** 2 - (dlr / 2) ** 2 - dfb ** 2],
                [(xk[0][0] - xk[4][0]) ** 2 + xk[4][0] ** 2 + (xk[1][0] - xk[5][0]) ** 2 - (dlr / 2) ** 2 - dfb ** 2],
                [(xk[2][0] - xk[4][0]) ** 2 + (xk[2][0] + xk[4][0]) ** 2 + (xk[3][0] - xk[5][0]) ** 2 - dlr ** 2]
            ])
            # print(Fxk)

            # 构造jacobii矩阵， 方程组计为： F(X) = 0; 那么，jacobii矩阵 = F‘(X) = D(F(X))
            DFX = np.array([
                [2 * xk[0][0] - dfb, 2 * (xk[1][0] + ct), 0, 0, 0, 0],
                [0, 0, 4 * xk[2][0] + dfb + dlr, 2 * (xk[3][0] + ct), 0, 0],
                [0, 0, 0, 0, 4 * xk[4][0] + dfb + dlr, 2 * (xk[5][0] + ct)],
                [2 * (xk[0][0] - xk[2][0]), 2 * (xk[1][0] - xk[5][0]), 4 * xk[2][0] - 2 * xk[0][0], 2 * (xk[3][0] - xk[1][0]), 0, 0],
                [2 * (xk[0][0] - xk[4][0]), 2 * (xk[1][0] - xk[5][0]), 0, 0, 4 * xk[4][0] - 2 * xk[0][0], 2 * (xk[5][0] - xk[1][0])],
                [0, 0, 4 * xk[2][0], 2 * (xk[3][0] - xk[5][0]), 4 * xk[4][0], 2 * (xk[5][0] - xk[3][0])]
            ])

            # DFX = DFX.astype(np.float64), 注意任何array都是2维的
            DFX_INV = np.linalg.inv(DFX)

            # 牛顿迭代公式： x(k+1) = x(k) - [F'[x(k)]]^(-1) * F(x(k))
            xkp1 = xk - np.matmul(DFX_INV, Fxk)
            # 计算误差
            for i in range(6):
                calculate_error += math.fabs(xkp1[i][0] - xk[i][0])

            # 更新当前的 xk
            xk = xkp1

        # 求解后，xk即为新的三个顶点的值
        # 需要注意改进的地方： 反解里的三个顶点的坐标值会每次都变，注意修改，把他们加入到类的属性之后注意他们值的变化
        # 正解里的x(0)的值的确定,毕竟x(0)不总是处于动平台平行定平台的位置

        # # for test：
        # rollNew = math.atan((xk[3][0] - xk[5][0]) / LR_LENGTH) * 180 / math.pi
        # pitchNew = math.atan(((xk[3][0] + xk[5][0]) / 2 - xk[1][0]) / FB_LENGTH) * 180 / math.pi
        # lastZ = 0
        # updownNew = ((xk[3][0] + xk[5][0]) / 2 + xk[1][0] ) / 2 - lastZ # 当前的动平台的中心的纵坐标 减去 上一次的纵坐标
        #
        # print ("rollNew : %f\n pitchNew : %f\n updownNew : %f\n"%(rollNew, pitchNew, updownNew))


        return self.inverseKinematicsData


# to test the func: DataOutput
# 为washOutData赋值作测试, 正式的运行时需要删除 @ line: 53
#         washOutData["pitch"] = 1.2
#         washOutData["roll"] = 2.1
#         washOutData["updown"] = 6.3
tmp = {
    "pitch": 1.2,  # 俯仰
    "roll": 2.1,  # 翻滚
    "yaw": 0,  # 艏摇
    "updown": 6.3,  # 上下平移
    "leftright": 0,  # 左右平移
    "forwardback": 0  # 前后平移
}
ik = InverseKinematics()
test = ik.DataOutput(tmp)
