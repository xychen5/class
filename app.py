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
