from flask import Flask, jsonify
from flask_cors import CORS
import dataInput as DI
import dataOutput as DO
from washout import *
import threading
import time


# configurations
DEBUG = False

# instantiate the app
app = Flask(__name__)
app.config.from_object(__name__)

# enable CORS
CORS(app, resources={r'/*': {'origins': '*'}})

# 读取的数据文件的路径
DATA_FILE_PATH = "/home/nash5/Desktop/class/data/pitchOnly.csv"

# 存放读取的数据和洗出,反解正解后的数据，这些数据供前端读取
srcData = []
resData = []
sumIOData = {
    'wPitch': [],
    'wRoll': [],
    'xAcc': [],
    'yAcc': [],
    'zAcc': [],
    'h1': [],
    'h2': [],
    'h3': []
}

sumResData = {
    'xa1': [],
    'ya1': [],
    'za1': [],
    'xa2': [],
    'ya2': [],
    'za2': [],
    'xa3': [],
    'ya3': [],
    'za3': [],
    'h1': [],
    'h2': [],
    'h3': [],
    'pitch': [],
    'roll': [],
    'updown': []
}

# 用于srcData和resData的锁
lock = threading.Lock()

@app.route('/')
def hello_world():
    return 'Hello World!'


@app.route('/io', methods=['GET'])
def io():
    # print (sumIOData)
    return jsonify(sumIOData)


@app.route('/visualize', methods=['GET'])
def visualize():
    # print (sumResData)
    return jsonify(sumResData)


class DaemonThread(threading.Thread):
    def __init__(self, threadID, name, getSampleInterval):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.getSampleInterval = getSampleInterval

    def run(self):
        """
        通过覆盖threading.Thread 父类的run方法来执行自己的函数，执行的逻辑如下：
            1，从文件中读取运动数据
            2，每间隔一个自定义周期，执行洗出和反解正解
            3，一般会一直循环在 2 里面，这是为了模拟真实的数据采集
        :return: none
        """
        dataReader = DI.DataInput()
        dataAll = dataReader.LoadFromFile('/home/nash5/Desktop/class/data/xAccOnly.csv')
        redoTime = 1  #  重复上面的数据的次数
        forEachItemTimeUsage = 0  # 单位为s
        while redoTime > 0:
            redoTime -= 1
            wo = WashOut(self.getSampleInterval)
            ik = DO.InverseKinematics()
            sumEachItemTimeUsage = 0
            for item in dataAll:
                # 计算性能现在开始计时：
                st_time = time.time()
                # 对于一个数据项，要执行的处理流程如下：
                print (item.wPitch)
                # 将读取的数据加入srcData，由于io()函数会读取srcData,写入的时候需要加锁
                lock.acquire()
                srcData.append(item)
                sumIOData['wPitch'].append(item.wPitch)
                sumIOData['wRoll'].append(item.wRoll)
                sumIOData['xAcc'].append(item.xAcc)
                sumIOData['yAcc'].append(item.yAcc)
                sumIOData['zAcc'].append(item.zAcc)
                lock.release()

                # 执行洗出算法, 将洗出的数据加入sumResData，注意，洗出的为角度，这里我们将其化为弧度
                washOutData = wo.washOutAlgorithm(item)
                lock.acquire()
                sumResData['pitch'].append(deg2rad(washOutData['pitch']))
                sumResData['roll'].append(deg2rad(washOutData['roll']))
                sumResData['updown'].append(deg2rad(washOutData['updown']))
                lock.release()

                # 执行反解
                inverseKinematicsData = ik.DataOutput(washOutData)

                # 将反解的数据加入resData
                lock.acquire()
                resData.append(inverseKinematicsData)
                sumResData['xa1'].append(inverseKinematicsData["xa1"])
                sumResData['ya1'].append(inverseKinematicsData["ya1"])
                sumResData['za1'].append(inverseKinematicsData["za1"])
                sumResData['xa2'].append(inverseKinematicsData["xa2"])
                sumResData['ya2'].append(inverseKinematicsData["ya2"])
                sumResData['za2'].append(inverseKinematicsData["za2"])
                sumResData['xa3'].append(inverseKinematicsData["xa3"])
                sumResData['ya3'].append(inverseKinematicsData["ya3"])
                sumResData['za3'].append(inverseKinematicsData["za3"])
                sumResData['h1'].append(inverseKinematicsData["h1"])
                sumResData['h2'].append(inverseKinematicsData["h2"])
                sumResData['h3'].append(inverseKinematicsData["h3"])
                sumIOData['h1'].append(inverseKinematicsData["h1"])
                sumIOData['h2'].append(inverseKinematicsData["h2"])
                sumIOData['h3'].append(inverseKinematicsData["h3"])
                lock.release()

                end_time = time.time()
                sumEachItemTimeUsage += end_time - st_time
                print("time usage is: ", end_time - st_time, "s")

                time.sleep(self.getSampleInterval)

                # print(srcData[-1], resData[-1])
            # print (sumResData, "\n\n ->> ", sumIOData['wPitch'])
            print ("all the redo times have completed!", " ---> ", redoTime)
            forEachItemTimeUsage = sumEachItemTimeUsage / len(dataAll)
            print ("forEachItemTimeUsage is ", forEachItemTimeUsage, "s")
if __name__ == '__main__':
    # 首先开启后台线程，该线程的运行方式： 每隔一个采样周期，则运行洗出算法和反解正解过程，同时更新全局变量
    daemonThread = DaemonThread(1, "daemon", 1)
    daemonThread.setDaemon(True)
    daemonThread.start()
    # 运行flask app
    app.run()

# 主要问题，如何实现定时获取，我们这里由于无法实时获得数据，便采用模拟的方式，每隔一个getSampleInterval，
# 便从读入的数据里获取一条记录，然后前端便调用获得数据接口和获得可视化接口获取对应的数据，
# 主要实现方式：
# 在main中执行定时采样并且调用洗出，反解的计算的后台线程，前段便也按照相关的时间间隔获取对应的共有数据
#   - 读取公共数据的时候需要上锁
# 然后便是输入数据显示的需求，看看是不是可以调用开源头的数据显示，初步考虑采用grafana,或者echarts的timeline也可以

# 简单实现： 仅仅读取数据然后开始模拟，前端的行为是根据
