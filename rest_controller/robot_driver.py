from controller import Robot

class Driver(Robot):
    def __init__(self):
        self.MAX_SPEED=5
        Robot.__init__(self)
        self.timestep= int(self.getBasicTimeStep())

        # set up the wheels
        self.wheels = []
        wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
        for i in range(6):
            self.wheels.append(self.getDevice(wheelsNames[i]))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)

        # set up the sensors
        self.sensorL1 = self.getDevice('left_side_sensor')
        self.sensorL2 = self.getDevice('left_outer_sensor')
        self.sensorL3 = self.getDevice('left_mid_sensor')
        self.sensorR1 = self.getDevice('right_side_sensor')
        self.sensorR2 = self.getDevice('right_outer_sensor')
        self.sensorR3 = self.getDevice('right_mid_sensor')

        # activate sensors
        self.sensorL1.enable(self.timestep)
        self.sensorL2.enable(self.timestep)
        self.sensorL3.enable(self.timestep)
        self.sensorR1.enable(self.timestep)
        self.sensorR2.enable(self.timestep)
        self.sensorR3.enable(self.timestep)

        # set up front camera
        self.camera = self.getDevice('front_camera')
        self.camera.enable(self.timestep)
    
    # define movement in specific direction
    def move(self, direction):

        if direction == 'stop': inp = 0 
        if direction == 'forward': inp = 1 
        if direction == 'backward': inp = -1 

        self.wheels[0].setVelocity(inp * self.MAX_SPEED)
        self.wheels[1].setVelocity(inp * self.MAX_SPEED)
        self.wheels[2].setVelocity(inp * self.MAX_SPEED)
        self.wheels[3].setVelocity(inp * self.MAX_SPEED)
        self.wheels[4].setVelocity(inp * self.MAX_SPEED)
        self.wheels[5].setVelocity(inp * self.MAX_SPEED)

    # define turning in specific direction
    def turn(self, direction):

        if direction == 'stop': inp = 0 
        if direction == 'right': inp = 1 
        if direction == 'left': inp = -1 

        self.wheels[0].setVelocity(inp * self.MAX_SPEED)
        self.wheels[1].setVelocity(-inp * self.MAX_SPEED)
        self.wheels[2].setVelocity(inp * self.MAX_SPEED)
        self.wheels[3].setVelocity(-inp * self.MAX_SPEED)
        self.wheels[4].setVelocity(inp * self.MAX_SPEED)
        self.wheels[5].setVelocity(-inp * self.MAX_SPEED)
    
    def step_one(self):
        return self.step(self.timestep)

