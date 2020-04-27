from flask import Flask, jsonify
from flask_cors import CORS
from dataIO import *
from washout import *

# configurations
DEBUG = True

# instantiate the app
app = Flask(__name__)
app.config.from_object(__name__)

# enable CORS
CORS(app, resources={r'/*': {'origins': '*'}})


@app.route('/')
def hello_world():
    return 'Hello World!'


@app.route('/io', methods=['GET'])
def io():
    return jsonify('return your input and output data here')


@app.route('/visualize', methods=['GET'])
def visualize():
    return 'return your visualize here'


if __name__ == '__main__':
    app.run()

# 主要问题，如何实现定时获取，我们这里由于无法实时获得数据，便采用模拟的方式，每隔一个getSampleInterval，
# 便从读入的数据里获取一条记录，然后前端便调用获得数据接口和获得可视化接口获取对应的数据，
# 主要实现方式：
# 在main中执行定时采样并且调用洗出，反解的计算的后台线程，前段便也按照相关的时间间隔获取对应的共有数据
#   - 读取公共数据的时候需要上锁
# 然后便是输入数据显示的需求，看看是不是可以调用开源头的数据显示，初步考虑采用grafana,或者echarts的timeline也可以