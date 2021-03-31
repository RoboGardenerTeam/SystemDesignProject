import numpy as np
from robot_driver import Driver
from random_controller import RandomController
from flask import Flask, jsonify
import random

app = Flask(__name__)
driver = Driver()
control = None

@app.route('/start')
def run_auto_control():
    global control

    if control is None:
        control = RandomController(driver)
        message = control.call_start()
    else:
        message = control.call_continue()

    return jsonify({'message' : message})

@app.route('/pause')
def pause_control():
    return jsonify({'message' : control.call_pause()})

@app.route('/gohome')
def stop_control():
    return jsonify({'message' : control.call_return_to_base()})

@app.route('/continue')
def continue_control():
    return jsonify({'message' : control.call_continue()})

@app.route('/battery')
def get_battery():
    return jsonify({'message' : driver.get_battery_value()})

@app.route('/status')
def get_status():
    global control

    if control is None:
        print('CONTROL NONE')
        return jsonify({
                'status': "AT_BASE",
                'statusMessage': "Waiting at base station"
            })
    else:
        print('CONTROL NOT NONE')
        return jsonify(control.get_status())

if __name__ == "__main__":
    random.seed(2)
    app.run(port=5001)

    # To start manually without using flask API:
    # control = RandomController(driver)
    # control.call_start()
