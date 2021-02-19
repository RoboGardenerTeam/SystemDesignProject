import numpy as np
from robot_driver import Driver
from random_controller import RandomController
from flask import Flask, jsonify
from threading import Thread

app = Flask(__name__)
driver = Driver()
control = None

@app.route('/control/run')
def run_auto_control():
    global control
    # instantiate the random controller class
    control = RandomController(driver)
    # start the control algorithm in a new thread
    Thread(target=control.run()).start()
    return jsonify({'message' : "Started Controller"})

@app.route('/control/stop')
def stop_control():
    # send signal to controller to terminate
    control.terminate()
    return jsonify({'message' : "Stopped Controller"})

if __name__ == "__main__":
    app.run()
