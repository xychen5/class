import math
import logging as LOG


def deg2rad(deg):
    return deg / 180.0 * math.pi

def rad2deg(rad):
    return rad * 180.0 / math.pi

class DataItem:
    def __init__(self, pitchVelocity, rollVelocity, xAcc, yAcc, zAcc):
        """
        输入数据单元的结构体,取右手系，x轴向前，y轴向左，z轴向上,这也是传给洗出算法的基本数据单元
        :param pitchVelocity: 俯仰角速度, 单位为：degree/s
        :param rollVelocity: 翻滚角速度
        :param xAcc: x方向的平动加速度,单位为：m/s^2
        :param yAcc: y方向的平动加速度
        :param zAcc: z方向的平动加速度
        """
        self.wPitch = deg2rad(pitchVelocity)  # 由于洗出算法里使用的均为弧度，这里作转换
        self.wRoll = deg2rad(rollVelocity)
        self.xAcc = xAcc
        self.yAcc = yAcc
        self.zAcc = zAcc

class DataInput:
    # 将数据一次性从文件里读取到内存里
    def LoadFromFile(self, filePath):
        """
        从对应的filePath中读取数据
        :param filePath: 运动数据文件，据格式定义为：每一行5个数据，由逗号分割，分别为：
                         pitchVelocity: 俯仰角速度, 单位为：degree/s
                         rollVelocity: 翻滚角速度
                         xAcc: x方向的平动加速度,单位为：m/s^2
                         yAcc: y方向的平动加速度
                         zAcc: z方向的平动加速度
        :return: DataItem 的一个list
        """

        # readfile into inData from filePath
        formatedData = []
        with open(filePath) as f:
            rawData = f.readlines()

            for oneLine in rawData:
                items = oneLine.strip('\n').split(',')
                tmp = DataItem(
                    float(items[0]),
                    float(items[1]),
                    float(items[2]),
                    float(items[3]),
                    float(items[4])
                )
                formatedData.append(tmp)
        return formatedData

# 测试读取数据，读取内容为项目里的data目录下的文件
input = DataInput()
data = input.LoadFromFile('/home/nash5/Desktop/class/data/pitchOnly.csv')
print (data[0].zAcc)