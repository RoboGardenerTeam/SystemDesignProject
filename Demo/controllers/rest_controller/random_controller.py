import numpy as np
import logging
import random
from robot_driver import Driver
from threading import Lock, Thread
from enum import Enum, auto

LOW_BATT_VAL = 0.2

class States(Enum):
    AT_BASE = auto()
    MOVE_OFF_BASE = auto()
    NAVIGATION = auto()
    RETURN_TO_BASE = auto()
    DRIVE_UP_BASE = auto()
    DUMP = auto()
    PAUSE = auto()

class AtBaseState:
    def __init__(self):
        pass

class MoveOffBaseState:
    def __init__(self):
        self.time_counter = 0

class NavigationState:
    def __init__(self):
        self.found = False
        self.id = None
        self.reorienting = False
        self.new_orientation = 0

class ReturnToBaseState:
    def __init__(self):
        pass

class DriveUpBaseState:
    def __init__(self):
        pass

class DumpState:
    def __init__(self):
        self.time_counter = 0

class PauseState:
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
        return self.get_status()

    def call_return_to_base(self):
        with self.lock:
            if self.state == States.NAVIGATION:
                self.transition_state(States.RETURN_TO_BASE)
        return self.get_status()

    def call_continue(self):
        with self.lock:
            if self.state == States.AT_BASE:
                self.transition_state(States.MOVE_OFF_BASE)
            if self.state == States.PAUSE:
                self.transition_state(States.NAVIGATION)
        return self.get_status()

    def call_pause(self):
        with self.lock:
            if self.state == States.NAVIGATION:
                self.transition_state(States.PAUSE)
        return self.get_status()

    def get_status(self):
        if self.state == States.AT_BASE:
            return {
                'status': "AT_BASE",
                'statusMessage': "Waiting at base station"
            }
        elif self.state == States.MOVE_OFF_BASE:
            return {
                'status': "MOVE_OFF_BASE",
                'statusMessage': "Leaving base station"
            }
        elif self.state == States.NAVIGATION:
            return { 
                'status': "NAVIGATION",
                'statusMessage': "Navigating"
            }
        elif self.state == States.RETURN_TO_BASE:
            return { 
                'status': "RETURN_TO_BASE",
                'statusMessage': "Returning to base station"
            }
        elif self.state == States.DRIVE_UP_BASE:
            return {
                'status': "DRIVE_UP_BASE",
                'statusMessage': "Parking at the base station"
            }
        elif self.state == States.DUMP:
            return {
                'status': "DUMP",
                'statusMessage': "Dumping load"
            } 
        elif self.state == States.PAUSE:
            return {
                'status': "PAUSE",
                'statusMessage': "Paused"
            } 
        else:
            return {
                'status': "INV",
                'statusMessage': "Invalid state"
            }

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
        elif self.state == States.PAUSE:
            self.state_data == PauseState()

    # drive up onto base station
    def drive_up_base_station(self):
        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()

            if model_name == b'charger':
                position = i.get_position()
                deviation = float(position[0])
                depth = float(position[2])

                if deviation <= 0.03 and deviation >= -0.03:
                    if depth <= -0.1:
                        self.driver.move('forward')
                    else:
                        self.driver.move('stop')
                        self.transition_state(States.DUMP)
                        return
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
        elif self.state_data.time_counter <= 270:
            self.driver.turn('left')
        else:
            self.transition_state(States.NAVIGATION)
            return

        self.state_data.time_counter += 1

    # back to base station
    def back_to_base_station(self):
        self.navigation_to_base_station()

        for i in self.driver.camera.getRecognitionObjects():
            model_name = i.get_model()

            if model_name == b'base station bar':
                position = i.get_position()
                depth = float(position[2])

                if depth >= -0.8:
                    self.transition_state(States.DRIVE_UP_BASE)
                    return

    def navigation_to_base_station(self):
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
            self.driver.turn('right', control=0)
        # if close to an object on the right, turn left
        elif (np.any(right_sensors_value < 1000)):
            self.driver.turn('left', control=0)
        # else drive forward
        else:
            self.driver.move('forward')

    # random navigation
    def random_navigation(self):
        battery_value = self.driver.get_battery_value()

        if battery_value < LOW_BATT_VAL:
            self.transition_state(States.RETURN_TO_BASE)
            return

        left_sensors_value = np.array([
            sensor.getValue() for sensor in
            [self.driver.sensors[0], self.driver.sensors[1], self.driver.sensors[2]]])
        right_sensors_value = np.array([
            sensor.getValue() for sensor in
            [self.driver.sensors[3], self.driver.sensors[4], self.driver.sensors[5]]])

        if self.state_data.reorienting:
            compass_vec = self.driver.compass.getValues()
            compass_vec = np.array([compass_vec[0], compass_vec[2]])

            new_orientation_vector = np.array([np.cos(np.deg2rad(self.state_data.new_orientation)), np.sin(np.deg2rad(self.state_data.new_orientation))])
            v = rotate_vector_radians(new_orientation_vector, -1 * np.arctan2(compass_vec[1], compass_vec[0]))
            signed_angle = np.arctan2(v[1], v[0])

            if abs(signed_angle) < 0.087: # 5 degrees
                self.state_data.reorienting = False
            elif signed_angle > 0:
                self.driver.turn('left')
            else:
                self.driver.turn('right')
        elif (np.any(left_sensors_value < 1000)) or (np.any(right_sensors_value < 1000)):
            self.state_data.new_orientation = random.randrange(0, 360)
            self.state_data.reorienting = True
        else:
            self.driver.move('forward')
            self.turn_towards_pinecone()

    def turn_towards_pinecone(self):
        objects = self.driver.camera.getRecognitionObjects()

        # if the robot found a object
        if self.state_data.found:
            # it indicates the node of the object
            # have to do this because there is no option to delete the node using ID
            p = self.driver.getFromId(self.state_data.id)
            # object
            pinecone = None
            # find the exact pine cone we found before because the robot can be
            # stranded if there are more than 2 pine cones
            # the robot is going to the pine cone what it found first
            for i in objects:
                if i.get_id() == self.state_data.id:
                    pinecone = i
            # if the pine cone is not in camera sight, clear flags
            # and go back to the search state
            if pinecone is None:
                self.state_data.found = False
                self.state_data.id = None
                return
            # get the position of the pine cone
            position = pinecone.get_position()
            # print(position)
            x = float(position[0])
            z = float(position[2])

            # adjust the robot's path on x axis, so make the object in center
            if (x > 0.1):
                self.driver.turn('right')
            elif (x < -0.1):
                self.driver.turn('left')
            else:
                # if the robot is close enough to the pine cone, remove the pine cone
                # and go back to the search state
                if z > -1:
                    model = str(pinecone.get_model())
                    print(model)
                    # idk why the simulator puts b in front of the model name
                    # the model names in the node and here should be same
                    if model == "b'pine cone'":
                        # remove the node we stored above
                        p.remove()
                    # clear the flags
                    self.state_data.found = False
                    self.state_data.id = None

        # check if pine cones are in detected objects
        else:
            # initial distance
            temp_dist = -1000
            # check every object on camera
            for i in objects:
                # get the model name of the object
                model = str(i.get_model())
                # get the z axis distance
                dist = i.get_position()[2]
                # initial precision
                precision = 1
                # distance between the robot and the pine cone is
                # less than 3m
                if i.get_position()[2] < -3:
                    precision = 0
                # between 1m to 3m
                elif i.get_position()[2] >= -3 and i.get_position()[2] < -1:
                    precision = 0.5
                # less than 1m
                elif i.get_position()[2] >= -1:
                    precision = 0.8

                # set a random number
                rn = random.random()
                # find the nearest pine cone
                if (model == "b'pine cone'") and (dist > temp_dist):
                    # if precision is larger than rd, True Positive case
                    if rn < precision:
                        self.state_data.id = i.get_id()
                        temp_dist = dist
                        self.state_data.found = True
                else:
                    # misdetection. False Positive
                    # the robot will be heading to this object
                    if (rn > precision):
                        self.state_data.id = i.get_id()
                        temp_dist = dist
                        self.state_data.found = True
                    # miss the object. True Negative
                    else:
                        self.state_data.found = False
                        self.state_data.id = None

    def dump(self):
        if self.state_data.time_counter <= 150:
            self.driver.front_motor.setPosition(2.5)
            self.driver.front_motor.setVelocity(1.0)
            self.driver.bottom_motor.setPosition(1.5)
            self.driver.bottom_motor.setVelocity(0.5)
        elif self.state_data.time_counter <= 300:
            self.driver.front_motor.setPosition(0.0)
            self.driver.front_motor.setVelocity(1.0)
            self.driver.bottom_motor.setPosition(0.0)
            self.driver.bottom_motor.setVelocity(0.5)
        elif self.state_data.time_counter <= 350:
            self.transition_state(States.AT_BASE)
            return

        self.state_data.time_counter += 1

    def chill_at_base(self):
        self.driver.move('stop')
        self.driver.battery_value = 1.0 # recharge immediately up to full battery

    def pause_robot(self):
        self.driver.move('stop')

    def main_loop(self):
        while self.driver.state_of_robot() != -1:
            with self.lock:
                battery_value = self.driver.get_battery_value()
                self.driver.battery_value -= 0.00001

                print(self.state)
                print(f'battery: {battery_value}')

                if self.state == States.AT_BASE:
                    self.chill_at_base()
                elif self.state == States.MOVE_OFF_BASE:
                    self.move_off_base_station()
                elif self.state == States.NAVIGATION:
                    self.random_navigation()
                elif self.state == States.RETURN_TO_BASE:
                    self.back_to_base_station()
                elif self.state == States.DRIVE_UP_BASE:
                    self.drive_up_base_station()
                elif self.state == States.DUMP:
                    self.dump()
                elif self.state == States.PAUSE:
                    self.pause_robot()

        print('END LOOP')

def rotate_vector_radians(vector, theta):
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    return np.dot(rotation_matrix, vector)
