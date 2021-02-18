import numpy as np
from robot_driver import Driver
from threading import Thread


class AutoController():
    def __init__(self, driver):
        self.driver = driver
        self._running = True
        

    def run(self):
        while self._running and self.driver.step_one() > -1:
            self.automatic_navigation()
    
    def terminate(self):
        self._running = False


    def automatic_navigation(self):
        left_sensors_value = np.array([
            sensor.getValue() 
            for sensor 
            in [self.driver.sensorL1, self.driver.sensorL2, self.driver.sensorL3]])
        right_sensors_value = np.array([
            sensor.getValue()
            for sensor
            in [self.driver.sensorR1, self.driver.sensorR2, self.driver.sensorR3]])

        if (np.any(left_sensors_value < 1000)):
            self.driver.turn('right')
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left')
        else:
            self.driver.move('forward')

