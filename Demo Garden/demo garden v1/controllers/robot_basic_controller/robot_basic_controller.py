"""robot_cbasic_controller_z"""

# need to import Supervisor 
from controller import Robot, Supervisor
import numpy as np
import math
import random
from random import randrange

# remember to activate 'supervisor' in the robot node
# and name 'model' of all pine cone nodes as 'pine cone'

# robot should be supervisor to make pine cones disappear
robot = Supervisor()

# global variables
found = False
id = None 

# change direction when the robot find a pine cone
def toward_pinecone(objects):
    # found will be up after the robot detect something
    global found
    # id of the detected object
    global id
        
    # if the robot found a object
    if found:
        # it indicates the node of the object
        # have to do this because there is no option to delete the node using ID
        p = robot.getFromId(id)
        # object
        pinecone = None
        # find the exact pine cone we found before because the robot can be
        # stranded if there are more than 2 pine cones
        # the robot is going to the pine cone what it found first
        for i in objects:
            if i.get_id() == id:
                pinecone = i
        # if the pine cone is not in camera sight, clear flags 
        # and go back to the search state
        if pinecone is None:
            found = False
            id = None
            return
        # get the position of the pine cone    
        position = pinecone.get_position()
        # print(position)
        x = float(position[0])
        z = float(position[2])
        
        # adjust the robot's path on x axis, so make the object in center
        if (x > 0.1):
            turn('right')
        elif (x < -0.1):
            turn('left')
        else:
            # if the robot is close enough to the pine cone, remove the pine cone
            # and go back to the search state
            if z > -1:
                # get the model name of the object
                model = str(pinecone.get_model())
                # idk why the simulator puts b in front of the model name
                # the model names in the node and here should be same
                if model == "b'pine cone'":
                    # remove the node we stored above
                    p.remove()
                # clear the flags    
                found = False
                id = None
                
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
            elif i.get_position()[2] > -1 and i.get_position()[2] < -3:
                precision = 0.5
            # less than 1m
            elif i.get_position()[2] > -1:
                precision = 0.8

            # set a random number
            rn = random.random()
            # find the nearest pine cone
            if (model == "b'pine cone'") and (dist > temp_dist):
                # if precision is larger than rd, True Positive case
                if rn < precision:
                    id = i.get_id()
                    temp_dist = dist
                    found = True                             
            else:
                # misdetection. False Positive
                # the robot will be heading to this object           
                if (rn > precision):
                    id = i.get_id()
                    temp_dist = dist
                    found = True
                # miss the object. True Negative  
                else:
                    found = False
                    id = None

# until here is my code
# check automatic_navigation() to see how I use this
###########################################################################


# get time steps
timestep = int(robot.getBasicTimeStep())

# define maximum speed
MAX_SPEED = 10

# define time step count
TIME_COUNT = 0

# set up wheels motor on the robot
wheels = []
wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 
               'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
for i in range(6):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# set up sensors on the robot 
sensorL1 = robot.getDevice('left_side_sensor')
sensorL2 = robot.getDevice('left_outer_sensor')
sensorL3 = robot.getDevice('left_mid_sensor')
sensorR1 = robot.getDevice('right_side_sensor')
sensorR2 = robot.getDevice('right_outer_sensor')
sensorR3 = robot.getDevice('right_mid_sensor')

# activate sensors
sensorL1.enable(timestep)
sensorL2.enable(timestep)
sensorL3.enable(timestep)
sensorR1.enable(timestep)
sensorR2.enable(timestep)
sensorR3.enable(timestep)

# set up front camera
camera = robot.getDevice('front_camera')

# activate camera and recognition
camera.enable(timestep)
camera.recognitionEnable(timestep)

# activate battery sensor
robot.batterySensorEnable(timestep)

# activate compass
compass = robot.getDevice('compass')
compass.enable(timestep)

# activate dump truck motors
front_motor = robot.getDevice('front_basket_motor')
bottom_motor = robot.getDevice('lift_bottom_motor')

# define movement in specific direction
def move(direction, control=1.0):
    if direction == 'stop': inp = 0 
    if direction == 'forward': inp = 1 
    if direction == 'backward': inp = -1
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED * control) 
    wheels[5].setVelocity(inp * MAX_SPEED * control)

# define turning in specific direction
def turn(direction, control=1.0):
    if direction == 'stop': inp = 0 
    if direction == 'right': inp = 1 
    if direction == 'left': inp = -1 
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(-inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(-inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED * control) 
    wheels[5].setVelocity(-inp * MAX_SPEED * control)

reorienting = False
new_orientation = 0

  

def rotate_vector_radians(vector, theta):
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    return np.dot(rotation_matrix, vector)

                
    
# automatical navigation by sensors
def automatic_navigation():
    global reorienting
    global new_orientation

    left_sensors_value = np.array([sensor.getValue() for sensor in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([sensor.getValue() for sensor in [sensorR1, sensorR2, sensorR3]])

    if reorienting:
        compass_vec = compass.getValues()
        compass_vec = np.array([compass_vec[0], compass_vec[2]])

        new_orientation_vector = np.array([np.cos(np.deg2rad(new_orientation)), np.sin(np.deg2rad(new_orientation))])
        v = rotate_vector_radians(new_orientation_vector, -1 * np.arctan2(compass_vec[1], compass_vec[0]))
        signed_angle = np.arctan2(v[1], v[0])

        if abs(signed_angle) < 0.087: # 5 degrees
            reorienting = False
        elif signed_angle > 0:
            turn('left')
        else:
            turn('right')
    elif (np.any(left_sensors_value < 1000)) or (np.any(right_sensors_value < 1000)):
        new_orientation = randrange(0, 360)
        reorienting = True
    else:
        # need to move forward otherwise the robot stay still
        move('forward')
        toward_pinecone(camera.getRecognitionObjects())
        
        
# automatical navigation by sensors
# def automatic_navigation(control=1.0):
    # left_sensors_value = np.array([s.getValue() for s in [sensorL1, sensorL2, sensorL3]])
    # right_sensors_value = np.array([s.getValue() for s in [sensorR1, sensorR2, sensorR3]])
    # if (np.any(left_sensors_value < 1000)):
        # turn('right', control)
    # elif (np.any(right_sensors_value < 1000)):
        # turn('left', control)
    # else:
        # move('forward')
        
# define random navigation algorithm        
# def random_navigation_algorithm():
    # battery_value = robot.batterySensorGetValue()
    # if battery_value >= 300000:
        # automatic_navigation()
    # else:
        # automatic_navigation(0)

# define turning towards base station   
def turn_towards_base_station():
    current_direction = compass.getValues()[2]
    if current_direction <= 0.002 and current_direction > -0.002:
        move('stop')
    elif current_direction < 0.002:
        turn('left')
    elif current_direction > 0.002:
        turn('right')
    else:
        move('stop')
        
# define function to move to base station bar       
def move_to_base_station_bar():
    for i in camera.getRecognitionObjects():
        model_name = i.get_model()
        if model_name == "base station bar":
            position = i.get_position()
            y = float(position[1])
            if y >= 0.08:
                move('forward', 0)
            else:
                move('stop')
            break
            
# define function to move to base station charger        
def move_to_base_station_charger():
    for i in camera.getRecognitionObjects():
        model_name = i.get_model()
        if model_name == "charger":
            position = i.get_position()
            deviation = float(position[0])
            if deviation <= 0.02 and deviation >= -0.02:
                move('forward', 0.1)
            elif deviation < -0.02:
                turn('left')
            elif deviation > 0.02:
                turn('right')
            else:
                move('stop')

# define function to clear dump truck            
def clear_dump_truck(time_count):
    if time_count <= 550:
        front_motor.setPosition(2.5)
        front_motor.setVelocity(1.0)
        bottom_motor.setPosition(1.5)
        bottom_motor.setVelocity(0.5)
    elif time_count <= 700:
        front_motor.setPosition(0.0)
        front_motor.setVelocity(1.0)
        bottom_motor.setPosition(0.0)
        bottom_motor.setVelocity(0.5)
    else:
        move('stop')

# define function to leave the base station     
def leave_base_station(time_count):
    if time_count <= 850:
        move('backward')
    elif time_count <= 900:
        turn('right')
    else:
        random_navigation_algorithm()

# define function to make a demo       
def back_to_base_station_demo(time_count):
# robot initial position: -2.96, 0.23, 7.08
# robot initial rotation: 0, 0, -1.57
    if time_count <= 100:
        move_to_base_station_bar()
    elif time_count <= 250:
        turn_towards_base_station()
    elif time_count <= 400:
        move_to_base_station_charger()
    elif time_count <= 800:
        clear_dump_truck(time_count)
    else:
        leave_base_station(time_count)
       
# p = robot.getFromId(724)
# print(p)
# p.remove()
# main loop - in each time step, do following
while robot.step(timestep) != -1:
 

    automatic_navigation()
    # TIME_COUNT = TIME_COUNT + 1
    # back_to_base_station_demo(TIME_COUNT)
    print(id)
    pass
