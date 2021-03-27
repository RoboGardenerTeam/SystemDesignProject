import numpy as np
import logging
from robot_driver import Driver
from threading import Lock, Thread
from random import randrange
from enum import Enum

logging.basicConfig(filename='example.log', level=logging.DEBUG)
# logging.debug('This message should go to the log file')

LOW_BATT_VAL = 0.2

class States(Enum):
    AT_BASE = 1
    MOVE_OFF_BASE = 2
    NAVIGATION = 3
    RETURN_TO_BASE = 4
    DRIVE_UP_BASE = 5
    DUMP = 6

class AtBaseState:
    def __init__(self):
        pass

class MoveOffBaseState:
    def __init__(self):
        self.time_counter = 0

class NavigationState:
    def __init__(self):
        pass

class ReturnToBaseState:
    def __init__(self):
        self.hit_front_bar = False

class DriveUpBaseState:
    def __init__(self):
        pass

class DumpState:
    def __init__(self):
        pass

class RandomController:
    def __init__(self, driver):
        self.driver = driver
        self.lock = Lock()
        self.cntrl_thread = None

        self.state = States.MOVE_OFF_BASE
        self.state_data = MoveOffBaseState()

    def call_start(self):
        if self.cntrl_thread is None:
            self.cntrl_thread = Thread(target=self.main_loop)
            self.cntrl_thread.start()
        return "Thread started"

    def call_return_to_base(self):
        self.lock.acquire()
        self.RETURN_TO_BASE = True
        self.lock.release()

    # ALWAYS IMMEDIATELY RETURN AFTER CALLING THIS METHOD!!!
    # state_data will become invalid if you don't immediately return
    def transition_state(self, new_state):
        self.state = new_state

        if self.state == States.AT_BASE:
            self.state_data = AtBaseState()
        elif self.state == States.MOVE_OFF_BASE:
            self.state_data = MoveOffBaseState()
        elif self.state == States.NAVIGATION:
            self.state_data = NavigationState()
        elif self.state == States.RETURN_TO_BASE:
            self.state_data = ReturnToBaseState()
        elif self.state == States.DRIVE_UP_BASE:
            self.state_data = DriveUpBaseState()
        elif self.state == States.DUMP:
            self.state_data = DumpState()

    # # set the stopping condition of the control loop
    # def terminate(self):
    #     self.lock.acquire()
    #     self.RUNNING = False
    #     self.TIME_COUNT = 0
    #     self.DUMP_TIME = 0
    #     self.STOP_FRONT_BAR = False
    #     self.AT_BASE_STATION = False
    #     self.START_NAVIGATION = False
    #     self.lock.release()

    # drive up onto base station
    def move_to_base_station(self):
        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()

            if model_name == b'charger':
                position = i.get_position()
                deviation = float(position[0])
                depth = float(position[2])

                if deviation <= 0.03 and deviation >= -0.03:
                    if depth <= -0.1:
                        self.driver.move('forward')

                    self.transition_state(States.DUMP)
                elif deviation < -0.03:
                    self.driver.turn('left')
                elif deviation > 0.03:
                    self.driver.turn('right')
            else:
                self.driver.turn('left')

    # drive down from base station and turn right
    def move_off_base_station(self):
        if self.state_data.time_counter <= 175:
            self.driver.move('backward')
        elif self.state_data.time_counter <= 230:
            self.driver.turn('right')
        else:
            self.transition_state(States.NAVIGATION)
            return

        self.state_data.time_counter += 1

    # back to base station
    def back_to_base_station(self):
        pass
        # TODO: correctly implement this
        # if self.state_data.hit_front_bar == False:
        #     self.random_navigation()

        #     for i in self.driver.camera.getRecognitionObjects():
        #         model_name = i.get_model()
        #         # print(str(model_name))

        #         if model_name == b'base station bar':
        #             position = i.get_position()
        #             depth = float(position[2])
        #             # print(depth)

        #             if depth >= -0.8:
        #                 self.driver.move('stop')
        #                 self.state_data.hit_front_bar = True

        #             break
        # else:
        #     self.transition_state(DRIVE_UP_BASE)

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

    def dump(self):
        pass
        # if self.TIME_COUNT <= self.DUMP_TIME + 150:
        #     self.driver.front_motor.setPosition(2.5)
        #     self.driver.front_motor.setVelocity(1.0)
        #     self.driver.bottom_motor.setPosition(1.5)
        #     self.driver.bottom_motor.setVelocity(0.5)
        # elif self.TIME_COUNT <= self.DUMP_TIME + 300:
        #     self.driver.front_motor.setPosition(0.0)
        #     self.driver.front_motor.setVelocity(1.0)
        #     self.driver.bottom_motor.setPosition(0.0)
        #     self.driver.bottom_motor.setVelocity(0.5)
        # elif self.TIME_COUNT <= self.DUMP_TIME + 350:
        #     self.AT_BASE_STATION = False
        #     self.MOVE_OFF = True #TODO remove this
        #     self.RETURN_TO_BASE = False
        #     self.DUMP_TIME = 0

    def main_loop(self):
        while self.driver.state_of_robot() != -1:
            self.lock.acquire()

            battery_value = self.driver.get_battery_value()
            print(f'battery: {battery_value}')
            # self.driver.battery_value -= 0.0001
            self.driver.battery_value -= 0.001

            if battery_value < LOW_BATT_VAL:
                self.transition_state(States.RETURN_TO_BASE)

            print(self.state)

            if self.state == States.AT_BASE:
                self.driver.move('stop') # do nothing basically?
            elif self.state == States.MOVE_OFF_BASE:
                self.move_off_base_station()
            elif self.state == States.NAVIGATION:
                self.random_navigation()
            elif self.state == States.RETURN_TO_BASE:
                self.back_to_base_station()
            elif self.state == States.DRIVE_UP_BASE:
                self.move_to_base_station()
            elif self.state == States.DUMP:
                self.dump()

            self.lock.release()
