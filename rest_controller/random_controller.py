import numpy as np
from robot_driver import Driver
from threading import Lock, Thread
from random import randrange
import logging
logging.basicConfig(filename='example.log', level=logging.DEBUG)
# logging.debug('This message should go to the log file')


LOW_BATT_VAL = 400000

class RandomController():
    def __init__(self, driver):
        self.driver = driver
        self._RUNNING = True
        self.lock = Lock()
        self.cntrl_thread = None
        self.TIME_COUNT = 0
        self.DUMP_TIME = 0
        self.STOP_FRONT_BAR = False
        self.AT_BASE_STATION = False
        self.START_NAVIGATION = False

        self.RETURN_TO_BASE = False
        self.MOVE_OFF = True   # set to true so the demo starts on charger

    def call_start(self):
        if self.cntrl_thread is None:
            self.cntrl_thread = Thread(target=self.loop)
            self.cntrl_thread.start()
        return "Thread started"

    def call_return_to_base(self):
        self.lock.acquire()
        self.RETURN_TO_BASE = True
        self.lock.release()


    # # set the stopping condition of the control loop
    # def terminate(self):
    #     self.lock.acquire()
    #     self._RUNNING = False
    #     self.TIME_COUNT = 0
    #     self.DUMP_TIME = 0
    #     self.STOP_FRONT_BAR = False
    #     self.AT_BASE_STATION = False
    #     self.START_NAVIGATION = False
    #     self.lock.release()

    # drive up onto base station   
    def move_to_base_station(self):
        print("move to base station")
        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()
            if model_name == b'charger':
                position = i.get_position()
                deviation = float(position[0])
                depth = float(position[2])
                if deviation <= 0.03 and deviation >= -0.03:
                    if depth <= -0.1:
                        self.driver.move('forward')
                    elif self.AT_BASE_STATION == False:
                        self.AT_BASE_STATION = True
                elif deviation < -0.03:
                    self.driver.turn('left')
                elif deviation > 0.03:
                    self.driver.turn('right')
            else:
                self.driver.turn('left')

    # drive down from base station and turn right
    def move_off_base_station(self):
        print("move off base station")
        found_charger = False
        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()
            if model_name == b'charger':
                found_charger = True
                position = i.get_position()
                deviation = float(position[0])
                depth = float(position[2])
                if depth >= -5:
                    self.driver.move('backward')
                
                elif deviation > -100:
                    self.driver.turn('right')

        if not found_charger:
            self.MOVE_OFF = False

    # back to base station       
    def back_to_base_station(self):
        if self.STOP_FRONT_BAR == False:
            self.random_navigation()
            for i in self.driver.camera.getRecognitionObjects():
                model_name = i.get_model()
                print(str(model_name))
                if model_name == b'base station bar':
                    position = i.get_position()
                    depth = float(position[2])
                    print(depth)
                    if depth >= -0.8:
                        self.driver.move('stop')
                        if self.STOP_FRONT_BAR == False:
                            self.STOP_FRONT_BAR = True

                    break       
        if self.STOP_FRONT_BAR == True:
            self.move_to_base_station()
        
    # random navigation
    def random_navigation(self):
        left_sensors_value = np.array([
            sensor.getValue() for sensor in 
            [self.driver.sensors[0], self.driver.sensors[1], self.driver.sensors[2]]])
        right_sensors_value = np.array([
            sensor.getValue() for sensor in 
            [self.driver.sensors[3], self.driver.sensors[4], self.driver.sensors[5]]])
        if (np.any(left_sensors_value < 1000)):
            self.driver.turn('right')
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left')
        else:
            self.driver.move('forward')

        # if close to an object on the left, turn right
        if (np.any(left_sensors_value < 1000)):
            self.driver.turn('right')
        # if close to an object on the right, turn left
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left')
        # else drive forward
        else:
            self.driver.move('forward')

    def loop(self):
        while self._RUNNING and self.driver.state_of_robot() != -1:
            self.lock.acquire()
            self.TIME_COUNT = self.TIME_COUNT + 1
            battery_value = self.driver.get_battery_value()
            
            if battery_value < LOW_BATT_VAL: self.RETURN_TO_BASE = True
            
            if self.AT_BASE_STATION:
                self.driver.move('stop')
                if self.DUMP_TIME == 0:
                    self.DUMP_TIME = self.TIME_COUNT
                self.dump()
                print(battery_value)

            elif self.MOVE_OFF:
                self.driver.move('stop')
                self.DUMP_TIME = 0
                self.AT_BASE_STATION = False
                self.move_off_base_station()

            elif self.RETURN_TO_BASE:
                # print("returning to base")
                self.back_to_base_station()
            else:
                print("random nav")
                self.random_navigation()
            self.lock.release()

    def dump(self):
        if self.TIME_COUNT <= self.DUMP_TIME + 150:
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
            self.AT_BASE_STATION = False
            self.MOVE_OFF = True #TODO remove this
            self.RETURN_TO_BASE = False
            self.DUMP_TIME = 0

    # full pipeline demo  
    def full_pipeline_demo(self, time_count):
        print(time_count)
        if time_count <= 50:
            self.driver.move('backward')
        elif time_count <= 110:
            self.driver.turn('right')
        else:
            self.start_robot_navigation()  