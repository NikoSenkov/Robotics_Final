"""URE_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, PositionSensor
from controller import Supervisor
from controller import Node
import numpy as np
import time
# create the Robot instance.

robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
timestep = 32
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Distance censor initialization
ds = robot.getDevice('distance sensor')
ds.enable(timestep)

#hand_part_names = ("finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1")
f1_part_names = ("palm_finger_1_joint", "finger_1_joint_1", "finger_1_joint_2", "finger_1_joint_3")
f2_part_names = ("palm_finger_2_joint", "finger_2_joint_1", "finger_2_joint_2", "finger_2_joint_3")
fm_part_names = ("finger_middle_joint_1", "finger_middle_joint_2", "finger_middle_joint_3")

ur_part_names = ("shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
#hand_parts = [robot.getDevice(part_name)  for part_name in hand_part_names]
f1_parts = [robot.getDevice(part_name)  for part_name in f1_part_names]
f2_parts = [robot.getDevice(part_name)  for part_name in f2_part_names]
fm_parts = [robot.getDevice(part_name)  for part_name in fm_part_names]
#palm finger joints range: -0.1784 < p < 0.192
#finger joint 1: 0.0495 < p < 1.2218
#  joint 2: 0 < p < 1.5708
#  joint 3: -1.2217 < p < -0.0523
e_sensor = robot.getDevice("elbow_joint_sensor")
e_sensor.enable(timestep)
s_sensor = robot.getDevice("shoulder_lift_joint_sensor")
s_sensor.enable(timestep)

robot_parts = [robot.getDevice(part_name) for part_name in ur_part_names]

robot_parts[0].setAvailableTorque(robot_parts[0].getMaxTorque())
robot_parts[1].setAvailableTorque(robot_parts[1].getMaxTorque())

robot_parts[0].setAvailableForce(robot_parts[0].getMaxForce())
robot_parts[1].setAvailableForce(robot_parts[1].getMaxForce())


#releases ball; starts to return to initial position
def release():
    #first finger: release
    f1_parts[0].setPosition(-0.1)
    f1_parts[1].setPosition(0.1)
    f1_parts[2].setPosition(0)
    f1_parts[3].setPosition(-1)
    #middle finger: release
    fm_parts[0].setPosition(0.1) #min: 0.0495
    fm_parts[1].setPosition(0) #min: 0
    #retract arms
    robot_parts[0].setPosition(0)
    robot_parts[1].setPosition(0)
    #flip arm
    robot_parts[-1].setPosition(0)
    #reset fingers
    f1_parts[0].setPosition(0.0)
    f1_parts[1].setPosition(0.495)
    f1_parts[2].setPosition(0)

def pickup():
    f1_parts[0].setPosition(0.19) #0.19
    f1_parts[1].setPosition(1.1) #1.2
    f1_parts[2].setPosition(0.9) # 1.3
    fm_parts[0].setPosition(0.5)
    fm_parts[1].setPosition(0.6)
     

def speed_select(d):
    ds = d-0.1
    dl = d+0.1
    #launch angle = 45 #degrees; we're not simulating aerodynamics, so this should be more-or-less optimal
    xm = np.sqrt(2)/2
    ym = np.sqrt(2)/2
    rel_ht = 1.6 #release height
    box_ht = 0.6
    g = -9.81
    timestep = 0.001 #seconds
    
    #launch_vels = np.array(range(1,101))/10
    
    calc_vel = None
    ub = 10
    lb = 0

    for _ in range(10): #adjust as necessary, for precision
        #initializing positions and velocities
        vel = (lb+ub)/2
        vx = vel*xm
        vy = vel*ym
        xpos = 0
        ypos = rel_ht
        
        #setting up loop
        max_iters = 10000
        i = 0
        step_x = timestep*vx
        step_vy = timestep*vy
        over = False
        while i < max_iters:
            i+=1
            #recalculating position and velocity
            xpos += step_x
            vy -= step_vy
            ypos += timestep*vy
            #convergence check
            if ypos < box_ht:
                if ds<xpos and dl > xpos:
                    calc_vel = vel
                else:
                    over = (xpos > d)
                i += max_iters
        if calc_vel is not None:
            break
        if over:
            ub = vel
        else:
            lb = vel
    return calc_vel
#robot length: 2.5 meters
#max release velocity: 2.5+1.25 m/s
#current crate distance: 5m (to base) + sqrt(2)/2 * 2.5

#I wrote this dot product function because python threw errors multiplying matrix by vector. Probably misunderstood something there.
def vec_rotate(m, v):
    result1 = m[0] * v[0] + m[1] * v[1] + m[2] * v[2]
    result2 = m[3] * v[0] + m[4] * v[1] + m[5] * v[2]
    result3 = m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
    result = [result1, result2, result3]
    return np.array(result)
    
#print(robot_parts[0].getMaxVelocity(), robot_parts[1].getMaxVelocity())

# Main loop:
# - perform simulation steps until Webots is stopping the controller
ctr = 0
grip = False
flip = False
swing = False
count = 0 

while robot.step(timestep) != -1:
    tgt_vel = speed_select(6.2)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #robot_parts[0].setPosition(-1.8)
    ctr+=1
    
    # This is how I give the robot time to adjust between motions
    if grip == True or flip == True or swing == True:
        count += 1
    
    #if(ctr % 8 == 0):
    #print("ds: ", ds.getValue())  print distance sensor value
    
    #pick up ball
    #290
    #if ctr==290 or ctr == 861:
    #print("0: ", f1_parts[0].getTargetPosition())
    #print("1: ", f1_parts[1].getTargetPosition())
    #print("2: ", f1_parts[2].getTargetPosition())
    
    # Grasp ball when distance sensor < 80
    if (ds.getValue() < 80):
        pickup()
        grip = True

        ####### PICK UP BALL
#        f1_parts[0].setPosition(0.19) #0.19
#        f1_parts[1].setPosition(1.1) #1.2
#        f1_parts[2].setPosition(0.9) # 1.3
#        fm_parts[0].setPosition(0.5)
#        fm_parts[1].setPosition(0.6)

        #########
        #f1_parts[3].setPosition(-0.06)
        
        #hand_parts[0].setPosition(1)
        #hand_parts[1].setPosition(1)
        #hand_parts[2].setPosition(1)
    
    # This is all unused code from trying to figure out Ball position relative to Robot
    #ball1 = robot.getFromDef('TARGET')
    #ball1 = np.array(ball1.getPosition())
    #rot_ur10e = np.array(ur10e.getOrientation())
    #rot_ur10e.reshape(3, 3)
    #rot_ur10e = np.transpose(rot_ur10e)
    #pos_ur10e = np.array(ur10e.getPosition())

    #ball2_pos = np.array(ball2.getPosition())
    #grip_pos = np.array(grip.getPosition())
    #print("ball pos: ", ball2_pos)
    #print("grip pos: ", grip_pos)
    
    #ball2_pos = np.subtract(ball2_pos, pos_ur10e)
    
    #print("ball pos: ", ball2_pos)
    #print("ur10e rot: ", rot_ur10e)
    
    #ball2_pos = vec_rotate(rot_ur10e, ball2_pos)
    #ball2_pos = vec_rotate(rot_ur10e, ball2_pos)
    #print(ball2_pos)
   
    #ball_rot_robot = vec_rotate(rot_ur10e, np.array(ball2.getOrientation()).reshape(3, 3))
    
    #print(ball_rot_robot)
    #print("ctr:", ctr)
    #print("count:", count)
    
    # lift arm
    if grip == True:        
        if count < 10:
            continue
        else: 
            robot_parts[0].setPosition(-0.4)
            grip = False
            count = 0
            flip = True
    
    # flip hand
    if flip == True:
        if count < 10:
            continue
        else:
            robot_parts[-1].setPosition(3.1)
            robot_parts[1].setPosition(0.5)
            count = 0
            flip = False
            swing = True
    
    # swing elbow
    if swing == True:
        if count < 30:
            continue
        else:
            robot_parts[0].setVelocity(tgt_vel*2/3*(3.14/2.5))
            robot_parts[1].setVelocity(tgt_vel/3*(3.14/1.25))
            
            #robot_parts[0].setVelocity(robot_parts[0].getMaxVelocity())
            #robot_parts[1].setVelocity(robot_parts[1].getMaxVelocity())
            
            robot_parts[0].setPosition(-3) #-1.8
            robot_parts[1].setPosition(-1.5)
            swing = False
            count = 0
    
    # release
    if e_sensor.getValue()<0*(-np.pi/4+0.1) and s_sensor.getValue()<-np.pi/4:
        release()
        
        
        #fm_parts[2].setPosition(-3)
    #if ctr%100 == 0:
     #   print(s_sensor.getValue(), e_sensor.getValue())
    #        print(ctr)
# Enter here exit cleanup code.
