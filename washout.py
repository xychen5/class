from filter import *
from dataIO import *
import math

class WashOut:
    # 由于三自由度运动平台的限制，得到的数据只有，pitch，roll，updown为有效值
    washOutData = {
        "pitch": 0, #俯仰
        "roll": 0, #翻滚
        "yaw": 0, #艏摇
        "updown": 0, #上下平移
        "leftright": 0, #左右平移
        "forwardback": 0  #前后平移
    }


    def __init__(self, getSampleInterval):
        """
        初始化洗出流程类，包括各个方向的滤波器
        :param getSampleInterval:获取样本的时间间隔
        """
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
        self.__lastPitchX = 0 #上一个洗出的pitch的角度

    def setXYScale(self, scale):
        self.__XYScale = scale

    def setZScale(self, scale):
        self.__ZScale = scale

    def setRScale(self, scale):
        self.__RScale = scale

    def washOutAlgorithm(self, inputDataItem):
        # -->> 缩放
        scaledDataItem = inputDataItem
        scaledDataItem.xAcc *= self.__XYScale
        scaledDataItem.yAcc *= self.__XYScale
        scaledDataItem.zAcc *= self.__ZScale
        scaledDataItem.wPitch *= self.__RScale
        scaledDataItem.wRoll *= self.__RScale

        # -->> tilt-coordination, 倾斜坐标系
        fTiltX = self.xLP.filter(scaledDataItem.xAcc)
        pitchX = math.asin(fTiltX)

        # 判断picthX是否位于[-pi/2, pi/2],超出范围则将其置为pi/2 or -pi/2
        if not ((pitchX >= -deg2rad(90)) and (pitchX <= deg2rad(90))):
            if pitchX > 0:
                pitchX = deg2rad(90)
            else:
                pitchX = -deg2rad(90)

        # 计算得到x方向的旋转的角加速度为 (pitchX - 上一个pitchX)/getSampleInterval，其值需要小于人体感受阈值2.6度



    def getWashOutData(self):
        return self.washOutData