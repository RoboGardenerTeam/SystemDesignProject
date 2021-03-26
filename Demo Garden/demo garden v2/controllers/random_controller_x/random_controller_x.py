from robot_driver import Driver
from random import randrange
import numpy as np

class RandomController():

    def __init__(self, driver):
        self.driver = driver
        self._RUNNING = True
        self.TIME_COUNT = 0
        self.DUMP_TIME = 0
        self.STOP_FRONT_BAR = False
        self.AT_BASE_STATION = False
        self.START_NAVIGATION = False
        
    # start the controller
    def run(self):
        while self._RUNNING and self.driver.state_of_robot() != -1:
            self.TIME_COUNT = self.TIME_COUNT + 1
            self.full_pipeline_demo(self.TIME_COUNT)
    
    # terminate the controller
    def terminate(self):
        self._RUNNING = True
        self.TIME_COUNT = 0
        self.DUMP_TIME = 0
        self.STOP_FRONT_BAR = False
        self.AT_BASE_STATION = False
        self.START_NAVIGATION = False

    # move to base station   
    def move_to_base_station(self):
        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()
            if model_name == "charger":
                position = i.get_position()
                deviation = float(position[0])
                if deviation <= 0.03 and deviation >= -0.03:
                    self.driver.move('forward', randrange(1,4), 0.2)
                    if self.AT_BASE_STATION == False:
                        self.AT_BASE_STATION = True
                elif deviation < -0.03:
                    self.driver.turn('left', randrange(1,4))
                elif deviation > 0.03:
                    self.driver.turn('right', randrange(1,4))
            else:
                self.driver.turn('left', randrange(1,4))  

    # back to base station       
    def back_to_base_station(self):
        if self.STOP_FRONT_BAR == False:
            self.random_navigation(0)
            for i in self.driver.camera.getRecognitionObjects():
                model_name = i.get_model()
                if model_name == "base station bar":
                    position = i.get_position()
                    depth = float(position[2])
                    if depth >= -0.8:
                        self.driver.move('stop', randrange(1,4))
                        if self.STOP_FRONT_BAR == False:
                            self.STOP_FRONT_BAR = True
                    break       
        if self.STOP_FRONT_BAR == True:
            self.move_to_base_station()
        
    # random navigation
    def random_navigation(self, control=1.0):
        left_sensors_value = np.array([
            sensor.getValue() for sensor in 
            [self.driver.sensors[0], self.driver.sensors[1], self.driver.sensors[2]]])
        right_sensors_value = np.array([
            sensor.getValue() for sensor in 
            [self.driver.sensors[3], self.driver.sensors[4], self.driver.sensors[5]]])
        if (np.any(left_sensors_value < 1000)):
            self.driver.turn('right', randrange(1,4), control)
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left', randrange(1,4), control)
        else:
            self.driver.move('forward', randrange(1,4))
            
    # start robot navigation      
    def start_robot_navigation(self):
        battery_value = self.driver.get_battery_value()
        if battery_value >= 400000 and self.AT_BASE_STATION == False:
            self.random_navigation()
        elif self.TIME_COUNT <= self.DUMP_TIME + 150:
            self.driver.front_motor.setPosition(2.5)
            self.driver.front_motor.setVelocity(1.0)
            self.driver.bottom_motor.setPosition(1.5)
            self.driver.bottom_motor.setVelocity(0.5)
        elif self.TIME_COUNT <= self.DUMP_TIME + 300:
            self.driver.front_motor.setPosition(0.0)
            self.driver.front_motor.setVelocity(1.0)
            self.driver.bottom_motor.setPosition(0.0)
            self.driver.bottom_motor.setVelocity(0.5)
        elif self.TIME_COUNT <= self.DUMP_TIME + 350:
            self.driver.move('backward', randrange(1,4))
        elif self.TIME_COUNT <= self.DUMP_TIME + 410:
            self.driver.turn('right', randrange(1,4))
            if self.START_NAVIGATION == False:
                self.START_NAVIGATION = True
        elif self.START_NAVIGATION == True:
            self.DUMP_TIME = 0
            self.STOP_FRONT_BAR = False
            self.AT_BASE_STATION = False
            self.START_NAVIGATION = False
            self.start_robot_navigation()
        elif battery_value >= 400000 and self.AT_BASE_STATION == True:
            self.driver.move('stop', randrange(1,4))
            if self.DUMP_TIME == 0:
                self.DUMP_TIME = self.TIME_COUNT
        else:
            self.back_to_base_station()
            
    # full pipeline demo  
    def full_pipeline_demo(self, time_count):
        if time_count <= 50:
            self.driver.move('backward', randrange(1,4))
        elif time_count <= 110:
            self.driver.turn('right', randrange(1,4))
        else:
            self.start_robot_navigation()  
            
if __name__ == "__main__":
    driver = Driver()
    random_controller = RandomController(driver)
    random_controller.run()
