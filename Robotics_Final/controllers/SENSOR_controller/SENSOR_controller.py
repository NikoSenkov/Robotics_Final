"""lab3 controller."""
from controller import Robot, Motor, GPS, Emitter
import json
import math
import numpy as np

N_PARTS = 8 # Total joints

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
timestep = 32

#GPS initialization
gps = robot.getDevice("gps")
gps.enable(timestep)

# Emitter initialization
EM = robot.getDevice("emitter")
EM.setChannel(-1)

part_names = ["left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
              "right motor 1", "right motor 2", "right motor 3", "right motor 4"]
# initialize
parts = [robot.getDevice(part) for part in part_names]

# helper functions
# change robot direction  
def setVelPN(identifier): [parts[i].setVelocity(identifier*(parts[i].getMaxVelocity() / 30)) for i in range(N_PARTS)]

# set wheel positions 
def setPos(p): [parts[i].setPosition(p) for i in range(N_PARTS)]
       
# to keep the bots going back and forth
setPos(float('inf'))
setVelPN(1)

flag = False
stepcount = 0
while robot.step(timestep) != -1:

    gpsVALS = gps.getValues()
    if stepcount % 1080 == 0:
        if flag == True:
            setVelPN(-1)
            flag = False
        else:
            setVelPN(1)
            flag = True
        
    if flag == True: gpsVALS.append("L")
    else: gpsVALS.append("R")
    
    
    # convert to json because ,send only takes strings
    # change to utf-8 encoding so .send accepts value
    message = json.dumps(gpsVALS).encode('utf-8')
    #print("sent message at timestep {}".format(stepcount))
    #1. middle
    #2. front
    #3. back
    EM.send(message)
    
    
    stepcount += 1
 
   