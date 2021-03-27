from controller import Robot
from random import randrange

class Driver(Robot):
    def __init__(self):

        Robot.__init__(self)

        # setup variables used
        self.TIME_STEP = int(self.getBasicTimeStep())
        self.MAX_SPEED = 4
        
        # set up wheels
        self.wheels = []
        wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 
                       'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
        for i in range(6):
            self.wheels.append(self.getDevice(wheelsNames[i]))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)

        # set up sensors
        self.sensors = []
        sensorsNames = ['left_side_sensor', 'left_outer_sensor', 'left_mid_sensor', 
                       'right_side_sensor', 'right_outer_sensor', 'right_mid_sensor']
        for i in range(6):
            self.sensors.append(self.getDevice(sensorsNames[i]))
            self.sensors[i].enable(self.TIME_STEP)

        # set up front camera
        self.camera = self.getDevice('front_camera')
        self.camera.enable(self.TIME_STEP)
        self.camera.recognitionEnable(self.TIME_STEP)
        
        # set up battery
        self.batterySensorEnable(self.TIME_STEP)
        self.BATT_MAX_VAL = 2000000 #TODO CHECK THIS IS CORRECT
        
        # set up dump truck motors
        self.front_motor = self.getDevice('front_basket_motor')
        self.bottom_motor = self.getDevice('lift_bottom_motor')
    
    # define movement in specific direction
    def move(self, direction):
        if direction == 'stop': inp = 0 
        if direction == 'forward': inp = 1 
        if direction == 'backward': inp = -1
        self.wheels[0].setVelocity((randrange(60,99)/100) * inp * self.MAX_SPEED)
        self.wheels[1].setVelocity((randrange(70,99)/100) * inp * self.MAX_SPEED)
        self.wheels[2].setVelocity((randrange(90,99)/100) * inp * self.MAX_SPEED)
        self.wheels[3].setVelocity((randrange(90,99)/100) * inp * self.MAX_SPEED)
        self.wheels[4].setVelocity((randrange(95,99)/100) * inp * self.MAX_SPEED) 
        self.wheels[5].setVelocity((randrange(95,99)/100) * inp * self.MAX_SPEED)
    
    # define turning in specific direction
    def turn(self, direction):
        if direction == 'stop': inp = 0 
        if direction == 'right': inp = 1 
        if direction == 'left': inp = -1
        self.wheels[0].setVelocity(inp * self.MAX_SPEED)
        self.wheels[1].setVelocity(-inp * self.MAX_SPEED)
        self.wheels[2].setVelocity(inp * self.MAX_SPEED)
        self.wheels[3].setVelocity(-inp * self.MAX_SPEED)
        self.wheels[4].setVelocity(inp * self.MAX_SPEED * (randrange(70,99)/100)) 
        self.wheels[5].setVelocity(-inp * self.MAX_SPEED * (randrange(90,99)/100))
    
    def state_of_robot(self):
        return self.step(self.TIME_STEP)
        
    def get_battery_value(self):
        return self.batterySensorGetValue()/self.BATT_MAX_VAL

