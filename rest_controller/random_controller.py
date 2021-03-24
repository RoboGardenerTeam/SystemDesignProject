import numpy as np
from robot_driver import Driver
from threading import Lock, Thread

class RandomController():
    def __init__(self, driver):
        self.driver = driver
        self._running = True
        self.lock = Lock()
        self.cntrl_thread = None

    def start(self):
        self.cntrl_thread = Thread(target=self.run_loop)
        self.cntrl_thread.start()
        
    # start a while loop that calls the random navigation
    def run_loop(self):
        while self._running and self.driver.step_one() > -1:
            self.lock.acquire()
            self.random_navigation()
            self.lock.release()
    
    # set the stopping condition of the control loop
    def terminate(self):
        self.lock.acquire()
        self._running = False
        self.lock.release()

    # random navigation algorithm
    def random_navigation(self):
        # get sensor values for left and right sensors into arrays
        left_sensors_value = np.array([
            sensor.getValue() 
            for sensor 
            in [self.driver.sensorL1, self.driver.sensorL2, self.driver.sensorL3]])
        right_sensors_value = np.array([
            sensor.getValue()
            for sensor
            in [self.driver.sensorR1, self.driver.sensorR2, self.driver.sensorR3]])

        # if close to an object on the left, turn right
        if (np.any(left_sensors_value < 1000)):
            self.driver.turn('right')
        # if close to an object on the right, turn left
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left')
        # else drive forward
        else:
            self.driver.move('forward')
