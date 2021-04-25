# Robotics_Final
Final Project for CS 3302: A WeBots Carnival Throwing Game Simulation 

Niko's changes to world:
  1) Made the conveyor belt width equal to ball diameter width. This prevents ball drift. 
  2) Robot throwing motion (grasp, lift arm, flip hand, swing elbow, release) is no longer actuated by hard coded time steps. Now it does the motions in sequence with a short delay between each motion to let the robot get into proper position. See main WHILE loop. 
  3) Robot throws 5 balls into the targeted crate (mostly accurate, sometimes it misses). If you ever see a ball fail to be grasped or thrown, let me know. 
