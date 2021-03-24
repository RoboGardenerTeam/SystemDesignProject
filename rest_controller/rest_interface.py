import numpy as np
from robot_driver import Driver
from random_controller import RandomController
from flask import Flask, jsonify

app = Flask(__name__)
driver = Driver()
control = None

@app.route('/start')
def run_auto_control():
    global control
    # instantiate the random controller class
    control = RandomController(driver)
    # start the control algorithm
    control.start()
    return jsonify({'message' : "Started Controller"})

@app.route('/pause')
def pause_control():
    # send signal to controller to terminate
    control.terminate()
    return jsonify({'message' : "Stopped Controller"})

@app.route('/stop')
def stop_control():
    # send signal to controller to return to base
    # TODO: implement this
    return jsonify({'message' : 'Returning to base'})

@app.route('/battery')
def get_battery():
    # TODO: implement battery getter
    return jsonify({'battery' : driver.get_battery()})

@app.route('/baseFill')
def get_base_fill():
    # TODO: implement actual base fill getter
    return jsonify({'base_fill' : 0.43})

@app.route('/status')
def get_status():
    # TODO: implement status getter 
    return jsonify({'status' : 'implement status getter'})
    

if __name__ == "__main__":
    app.run()
