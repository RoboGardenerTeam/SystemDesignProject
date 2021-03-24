"""robot_cbasic_controller_z"""

from controller import Robot
import numpy as np
import math

# set up robot
robot = Robot()

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
    
# automatical navigation by sensors
def automatic_navigation(control=1.0):
    left_sensors_value = np.array([s.getValue() for s in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([s.getValue() for s in [sensorR1, sensorR2, sensorR3]])
    if (np.any(left_sensors_value < 1000)):
        turn('right', control)
    elif (np.any(right_sensors_value < 1000)):
        turn('left', control)
    else:
        move('forward')
        
# define random navigation algorithm        
def random_navigation_algorithm():
    battery_value = robot.batterySensorGetValue()
    if battery_value >= 300000:
        automatic_navigation()
    else:
        automatic_navigation(0)

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
       
# main loop - in each time step, do following
while robot.step(timestep) != -1:

    TIME_COUNT = TIME_COUNT + 1
    back_to_base_station_demo(TIME_COUNT)
    
    pass
