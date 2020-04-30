from filter import *
from dataInput import *
import math


class WashOut:
    def __init__(self, getSampleInterval):
        """
        初始化洗出流程类，包括各个方向的滤波器
        :param getSampleInterval:获取样本的时间间隔
        """
        # 洗出算法的结果，由于三自由度运动平台的限制，得到的数据只有，pitch，roll，updown为有效值
        self.__washOutData = {
            "pitch": 0,  # 俯仰
            "roll": 0,  # 翻滚
            "yaw": 0,  # 艏摇
            "updown": 0,  # 上下平移
            "leftright": 0,  # 左右平移
            "forwardback": 0  # 前后平移
        }
        # x low pass filter settings
        self.xLP = TransLPFilter(getSampleInterval)
        self.xLP.setCutoffFreq(5.0)
        self.xLP.setDamping(1.0)
        # y low pass filter settings
        self.yLP = TransLPFilter(getSampleInterval)
        self.yLP.setCutoffFreq(8.0)
        self.yLP.setDamping(1.0)
        # z high pass filter settings
        self.zHP = TransHPFilter(getSampleInterval)
        self.zHP.setCutoffFreq(4.0)
        self.zHP.setDamping(1.0)
        # pitch high pass filter settings
        self.pitchHP = RotatHPFilter(getSampleInterval)
        self.pitchHP.setCutoffFreq(1.0)
        # roll high pass filter settings
        self.rollHP = RotatHPFilter(getSampleInterval)
        self.rollHP.setCutoffFreq(1.0)
        self.__XYScale = 1
        self.__ZScale = 1
        self.__RScale = 1
        self.__lastTiltPitchX = 0  # 上一个倾斜坐标系洗出的pitch的角度, 单位 rad
        self.__lastTiltRollY = 0  # 上一个倾斜坐标系洗出的roll的角度
        self.__lastRotPitchX = 0  # 上一个角速度通道得到的pitch的角度
        self.__lastRotRollY = 0  # 上一个角速度通道得到的roll的角度
        self.__lastTransZVelocity = 0  # 上一个平动加速度通道的Z方向的速度
        self.__getSampleInterval = getSampleInterval  # 单位 s

    def setXYScale(self, scale):
        self.__XYScale = scale

    def setZScale(self, scale):
        self.__ZScale = scale

    def setRScale(self, scale):
        self.__RScale = scale

    def restrictTiltAngleInThreshold(self, curRadAngle, preRadAngle):
        """
        将倾斜坐标系通道的角限制在合理范围内
        :param curRadAngle: 滤波器当前输出的角度
        :param preRadAngle: 滤波器上一次输出的角度
        :return: 限制后的角度
        """
        # 判断角度是否位于[-pi/2, pi/2],超出范围则将其置为pi/2 or -pi/2
        if not ((curRadAngle >= -deg2rad(90)) and (curRadAngle <= deg2rad(90))):
            if curRadAngle > 0:
                curRadAngle = deg2rad(90)
            else:
                curRadAngle = -deg2rad(90)

        # 计算得到动平台在x or y方向的旋转的角速度为 (curRadAngle - preRadAngle)/getSampleInterval，其值需要小于人体感受阈值2.6 rad/s
        if math.fabs(curRadAngle - preRadAngle) / self.__getSampleInterval > deg2rad(2.6):
            maxPitchVal = deg2rad(2.6) * self.__getSampleInterval
            if curRadAngle - preRadAngle > 0:
                # 相比于上一次增加的角度,也就是增加幅度超过阈值
                curRadAngle = preRadAngle + maxPitchVal
            else:
                # 相比于上一次减小的角度，减小幅度超过阈值
                curRadAngle = preRadAngle - maxPitchVal
        return curRadAngle

    def washOutAlgorithm(self, inputDataItem):
        """
        根据输入的数据执行洗出算法
        :param inputDataItem: 详情见 DataItem 类
        :return: 洗出的动平台的位置姿态，以字典数据返回
        """

        # -----------------------------------------------
        # -->> 缩放
        # -----------------------------------------------
        scaledDataItem = inputDataItem
        scaledDataItem.xAcc *= self.__XYScale
        scaledDataItem.yAcc *= self.__XYScale
        scaledDataItem.zAcc *= self.__ZScale
        scaledDataItem.wPitch *= self.__RScale
        scaledDataItem.wRoll *= self.__RScale

        # -----------------------------------------------
        # -->> tilt-coordination, 倾斜坐标系，包括x,y两个方向
        # -----------------------------------------------
        # 由于平台限制，x,y方向的突发线性加速度无法模拟
        # tilt simulate x
        fTiltX = self.xLP.filter(scaledDataItem.xAcc)
        tiltPitchX = math.asin(fTiltX)
        self.__lastTiltPitchX = tiltPitchX = self.restrictTiltAngleInThreshold(tiltPitchX, self.__lastTiltPitchX)
        # print ("pitch_1: ", tiltPitchX)

        # tilt simulate y
        fTiltY = self.yLP.filter(scaledDataItem.yAcc)
        tiltRollY = math.asin(fTiltY)  # 返回值为弧度
        self.__lastTiltRollY = tiltRollY = self.restrictTiltAngleInThreshold(tiltRollY, self.__lastTiltRollY)

        # -----------------------------------------------
        # -->> Rotation,旋转通道,将角速度的高频率的部分拿出来模拟
        # -----------------------------------------------
        # rotation pitch
        rotPitchW = self.pitchHP.filter(scaledDataItem.wPitch)  # 获得pitch方向的角速度的高频部分
        rotPitchX = self.__lastRotPitchX + rotPitchW * self.__getSampleInterval  # 积分获得当前角速度带来的角度变化，然后获得当前的角度
        self.__lastRotPitchX = rotPitchX
        # print ("pitch_2: ", rotPitchX)

        # rotation roll
        rotRollW = self.rollHP.filter(scaledDataItem.wRoll)
        rotRollY = self.__lastRotRollY + rotRollW * self.__getSampleInterval
        self.__lastRotRollY = rotRollY

        # -----------------------------------------------
        # -->> translation,平动加速通道，主要模拟突发线性加速度
        # -----------------------------------------------
        # 由于运动平台限制，仅能模拟z方向的突发线性加速度，也就是滤出高频部分然后2次积分得到位移
        zAccPlusG = scaledDataItem.zAcc  # 这里不再加入g的原因是，输入的就是人体的线性绝对加速度
        fTransZAcc = self.zHP.filter(zAccPlusG)
        # 计算出移动的位移， y = (v0 + a * T) * T
        transZ = (self.__lastTransZVelocity + fTransZAcc * self.__getSampleInterval) * self.__getSampleInterval
        self.__lastTransZVelocity = self.__lastTransZVelocity + fTransZAcc * self.__getSampleInterval

        # 返回的值
        self.__washOutData["pitch"] = rad2deg(tiltPitchX + rotPitchX)
        self.__washOutData["roll"] = rad2deg(tiltRollY + rotRollY)
        self.__washOutData["updown"] = transZ

        return self.__washOutData

    def getWashOutData(self):
        return self.__washOutData

# 使用以下代码测试
wo = WashOut(1)
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(1, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(1, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(1, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(1, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
t1 = DataItem(0, 0, 0, 0, 0)
tmp = wo.washOutAlgorithm(t1)
print (">>>pitch: %f roll: %f z: %f" % (tmp["pitch"], tmp["roll"], tmp["updown"]))
