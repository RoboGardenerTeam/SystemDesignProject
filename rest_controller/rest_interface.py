"""rest_controller controller"""

import numpy as np
from robot_driver import Driver
from auto_controller import AutoController
from flask import Flask, jsonify
# from flask_restful import Resource, Api
from threading import Thread

app = Flask(__name__)
driver = Driver()
control_thread = None
control = None



@app.route('/control/run')
def run_auto_control():
    global control
    control = AutoController(driver)
    control_thread = Thread(target=control.run())
    control_thread.start()
    return jsonify({'message' : "Started Controller"})

@app.route('/control/stop')
def stop_control():
    control.terminate()
    return jsonify({'message' : "Stopped Controller"})

if __name__ == "__main__":
    app.run()

