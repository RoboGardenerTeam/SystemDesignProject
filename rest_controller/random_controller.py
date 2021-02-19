import numpy as np
from robot_driver import Driver

class RandomController():
    def __init__(self, driver):
        self.driver = driver
        self._running = True
        
    # star a while loop that calls the random navigation
    def run(self):
        while self._running and self.driver.step_one() > -1:
            self.random_navigation()
    
    # set the stopping condition of the control loop
    def terminate(self):
        self._running = False

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
