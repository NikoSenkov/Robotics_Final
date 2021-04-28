"""URE_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, PositionSensor
from controller import Node, Receiver, Supervisor, Field
import json
import numpy as np
# create the Robot instance.

robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
timestep = 32

# receiver initialization
RES = robot.getDevice("receiver")
RES.enable(1)
RES.setChannel(-1) 

# Distance censor initialization
ds = robot.getDevice('distance sensor')
ds.enable(timestep)

#hand_part_names = ("finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1")
f1_part_names = ("palm_finger_1_joint", "finger_1_joint_1", "finger_1_joint_2", "finger_1_joint_3")
f2_part_names = ("palm_finger_2_joint", "finger_2_joint_1", "finger_2_joint_2", "finger_2_joint_3")
fm_part_names = ("finger_middle_joint_1", "finger_middle_joint_2", "finger_middle_joint_3")

ur_part_names = ("shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "shoulder_pan_joint", "wrist_3_joint")
#hand_parts = [robot.getDevice(part_name)  for part_name in hand_part_names]
f1_parts = [robot.getDevice(part_name)  for part_name in f1_part_names]
f2_parts = [robot.getDevice(part_name)  for part_name in f2_part_names]
fm_parts = [robot.getDevice(part_name)  for part_name in fm_part_names]
#palm finger joints range: -0.1784 < p < 0.192
#finger joint 1: 0.0495 < p < 1.2218
#  joint 2: 0 < p < 1.5708
#  joint 3: -1.2217 < p < -0.0523

e_sensor = robot.getDevice("elbow_joint_sensor")
e_sensor.enable(1)
s_sensor = robot.getDevice("shoulder_lift_joint_sensor")
s_sensor.enable(1)

robot_parts = [robot.getDevice(part_name) for part_name in ur_part_names]

robot_parts[0].setAvailableTorque(robot_parts[0].getMaxTorque())
robot_parts[1].setAvailableTorque(robot_parts[1].getMaxTorque()/1.25)

robot_parts[0].setAvailableForce(robot_parts[0].getMaxForce())
robot_parts[1].setAvailableForce(robot_parts[1].getMaxForce()/1.25)

#robot_parts[-2].setVelocity(robot_parts[-2].getMaxVelocity())

# Color target crates black
crates = ["D1", "D2", "B2", "B1", "A3", "D3", "A2", "A1"]
for id in crates:
    node = robot.getFromDef(id)
    color = node.getField("color")
    color.setSFColor([0, 0, 0])

def getCordData():
    packet = RES.getData() #peek from queue
    RES.nextPacket() #pop from queue
    return json.loads(packet)

#releases ball; starts to return to initial position

def release_fingers():
    #first finger: release
    f1_parts[0].setPosition(0) #-0.1
    f1_parts[1].setPosition(0.0495) #0.1 (min 0.0495)
    f1_parts[2].setPosition(0) #0
    f1_parts[3].setPosition(-0.0523) #-1 (max -0.0523)
    #middle finger: release
    fm_parts[0].setPosition(0.1) #min: 0.0495
    fm_parts[1].setPosition(0) #min: 0
    #move elbow and shoulder backwards
    #to get the fingers out of the ball's way
    robot_parts[0].setPosition(-.3)
    robot_parts[1].setPosition(.3)
    #rotate two wrist joints to pickup position
    robot_parts[2].setPosition(0)
    robot_parts[-1].setPosition(0)
    #set speed of shoulder and elbow joints to
    #consistently return arm after throwing
    robot_parts[0].setVelocity(2)
    robot_parts[1].setVelocity(2)
   
def retract_arms():   
    #retract arms
    robot_parts[0].setPosition(0)
    robot_parts[1].setPosition(0)
    #flip arm
    #robot_parts[-1].setPosition(0)
    #correct wrist
    #robot_parts[2].setPosition(0)
    
def correct_shoulder():
    #correct shoulder pan
    robot_parts[-2].setPosition(0)   
    
def pickup(ball_in_hand):
    f1_parts[0].setPosition(0.19) #0.19
    f1_parts[1].setPosition(1.2) #1.2
    f1_parts[2].setPosition(0.8) # 1.3
    fm_parts[0].setPosition(0.5)
    fm_parts[1].setPosition(0.6)
    ball_in_hand[0] = True
     
def speed_from_dist(d):
    #if d < 5.36:
    return 1.69+(3-1.6)/(4.96-1.33)*(d-1.6)

#print(robot_parts[0].getMaxVelocity(), robot_parts[1].getMaxVelocity())

# Main loop:
# - perform simulation steps until Webots is stopping the controller
ctr = 0

#state machine variables
grip = False
flip = False
swing = False
ball_in_hand = [False] #array for pass-by-reference-updates
retracting = False
shouldering = False
count = 0
target_iter = 0
color_iter = 0

#initialize GPS coordinates for carts
front_cart_cords = [0, 0, 0, ""]
middle_cart_cords = [0, 0, 0, ""]
back_cart_cords = [0, 0, 0, ""]


coordinateFlag = False
coordinateIndex = 0
while robot.step(timestep) != -1:

    if coordinateFlag:
        for _ in range(3):
            data = getCordData()
            
            # update middle cart
            if coordinateIndex == 0: 
                middle_cart_cords = data
                coordinateIndex += 1
                
            # update front cart
            elif coordinateIndex == 1: 
                front_cart_cords = data
                coordinateIndex += 1
                
            # update back cart
            elif coordinateIndex == 2: 
                back_cart_cords = data
                coordinateIndex = 0
                
    if middle_cart_cords[3] == 'L': middle_cart_cords[3] = "R"
    else: middle_cart_cords[3] = "L"
       
        
    # front basket coordinates
    D1 = [ front_cart_cords[0]-0.68495, front_cart_cords[1], front_cart_cords[2]+3.45, front_cart_cords[3] ]
    D2 = [ front_cart_cords[0]-0.03, front_cart_cords[1], front_cart_cords[2]+3.45, front_cart_cords[3] ]
    D3 = [ front_cart_cords[0]+0.655, front_cart_cords[1], front_cart_cords[2]+2.79, front_cart_cords[3] ]
    
    # middle basket coordinates
    B1 = [ middle_cart_cords[0]-0.6662775512, middle_cart_cords[1], middle_cart_cords[2]-0.01, middle_cart_cords[3] ]
    B2 = [ middle_cart_cords[0]+0.8137224488, middle_cart_cords[1], middle_cart_cords[2]-0.01, middle_cart_cords[3] ]
    
    A1 = [ back_cart_cords[0]-0.7511770145, back_cart_cords[1], back_cart_cords[2]-3.466, back_cart_cords[3] ]
    A2 = [ back_cart_cords[0]-0.06, back_cart_cords[1], back_cart_cords[2]-2.93, back_cart_cords[3] ]
    A3 = [ back_cart_cords[0]+0.6488229855, back_cart_cords[1], back_cart_cords[2]-3.465837942, back_cart_cords[3] ]
    
    targets = [D1, D2, B2, B1, A3, D3, A2, A1]
    
    # skips first iteration to ensure receiver queue is not empty

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #robot_parts[0].setPosition(-1.8)
    ctr+=1
    
    # This is how I give the robot time to adjust between motions
    if grip or flip or swing or retracting or shouldering:
        count += 1
 
    # Grasp ball when distance sensor < 50
    if (ds.getValue() < 50): #80
        pickup(ball_in_hand)
        grip = True

    # lift arm
    if grip and ball_in_hand[0]:
        if count < 10:
            pass
        else:    
            count = 0
            robot_parts[0].setPosition(-0.4)   
            grip = False      
            flip = True
    
    # flip hand
    if flip and ball_in_hand[0]:
        if count < 5:
            pass
        else:
            #print(B2)
            C1 = targets[target_iter]
            
            robot_parts[-1].setPosition(-np.pi/2)
            robot_parts[2].setPosition(-np.pi/2)
            
            robot_parts[1].setPosition(0.5)
            #MODIFIED CODE:
            zdist = -C1[2] + 1.25
            xdist = -C1[0]
            if C1[3] == 'R':
                xdist -= 0.674
            else:
                xdist += 0.674
            #print(C1)
            
            theta = np.arctan(xdist/zdist)
            
    #increment for binary search
            dtheta = .33
            while dtheta>1e-4:
                #ball position coordinates (relative to robot base)
                bx = 1.2*np.sin(theta) + 0.46*np.cos(theta)
                bz = 1.2*np.cos(theta) - 0.46*np.sin(theta)
                #wrist position coordinates (relative to robot base)
                wx = 1.2*np.sin(theta)
                wz = 1.2*np.cos(theta)
                #print(bx,bz)
                #print(wx/wz, wx)
                #print(bx/bz, bx)
                #print(xdist/zdist,xdist)
                wrist_angle = np.arctan(wx/wz)
                #print(wx, xdist, wz, zdist)
                ball_angle = np.arctan((bx+xdist)/ (bz+zdist)) #relative to crate
                
                if wrist_angle < ball_angle:
                    #print("true")
                    theta += dtheta
                else:
                    #print("false")
                    theta -= dtheta
                
                dtheta /= 2
           
            
            robot_parts[-2].setPosition(theta)
            crate_dist = np.sqrt(zdist**2+xdist**2)
            flip = False
            swing = True
            count = 0
    
    # swing elbow
    if swing and ball_in_hand[0]:
        if count < 30:
            pass
        else:
            count = 0
            ##CALCULATING PAN AND DISTANCE
            #B2 = [x,y,z]
            #bx = x
            #bz = z-1.25
            
            
            z = -4 #z coordinate of basket; throwing straight
            
            #spd = speed_from_dist(1.24 - z)
            spd = speed_from_dist(crate_dist)
            if spd > 3.14:
                print("max speed exceeded")
                spd = 3.14
            #assigning speed
            robot_parts[0].setVelocity(spd)
            robot_parts[1].setVelocity(spd)
            #shoulder and elbow lift
            robot_parts[0].setPosition(-3) #-1.8
            robot_parts[1].setPosition(-1.5)
        
            swing = False     
    
    # release
    #if e_sensor.getValue()<0*(-np.pi/4+0.1) and s_sensor.getValue()<-np.pi/4:
    if e_sensor.getValue()<0.1 and s_sensor.getValue()<-np.pi/4+0 and ball_in_hand[0]:    
        #release()
        release_fingers()
        ball_in_hand[0] = False 
        shouldering = True
        
        # Color current target blue
        crates = ["D1", "D2", "B2", "B1", "A3", "D3", "A2", "A1"]
        crate = robot.getFromDef(crates[color_iter])
        color = crate.getField("color")
        color.setSFColor([.3333, .6667, 1])      
        
    if retracting:
        if count < 10:
            pass
        else:
            count = 0
            retract_arms()
            target_iter += 1
            color_iter += 1
            retracting = False
            if color_iter == 8:
                break
          
    if shouldering:
        if count < 10:
            pass
        else:
            count = 0       
            correct_shoulder()
            shouldering = False
            retracting = True
    
    # for skipping first receiver check
    coordinateFlag = True
# Enter here exit cleanup code.
