# Robotics_Final
Final Project for CS 3302: A WeBots Carnival Throwing Game Simulation 

Hi Guys, I uploaded some very basic groundwork.

Robots: UR10e and UR5e with the toolslot attachment: Robotiq3fGripper which is a claw with 3 fingers. I am not sure what sensors are available, or specifically how we can use them to achieve our goal, this is a next step.

Ball Placement: 
  1) We can use a table, but would have to place the balls along an arc which the robot has access to (since robot limbs are not extendable). 
  2) We can feed the robots with balls on the conveyer belt I set up. This seems like a preferable idea to me. 

Targets: 
  I created two plastic test crates. With your approval, we can use these plastic crates as a goal. I think we can programatically randomize their placement, during the initialization of our simulation. 
  
  
  I just wanted to get this basic world up to see what you guys think, and I welcome any changes, suggestions or adjustments. Of course many details and aesthetics can be improved, but this is my basic idea. 
  
  I have imported the code freom the UR robot sample world. It is a C controller code, but gives us an idea on how to control these robots with Python. 
  I suggest opening the Sample World for UR robots to see it in action. 
