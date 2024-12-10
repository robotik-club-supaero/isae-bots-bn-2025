# Robotik - Pamy Simulator

### Requirements :
    - Python 3
    - Pygame

### How to run :
run `main.py` script, a pygame window should start.

### How it works :

The main loop update all the geometry components and the all robots are updated.
When a robot is updated he called the MachineEtat function to update his behavior.
According to his state he will move or not toward a target (=cursor by default).

### How to custom : 

The robot target can be set in its definition in the init() with the argument target=[x, y], 
makes sure to remove the target=cursor in the update function of the robot.

The pathfinding may be custom in `Pathfinding.py`, you can add argument to take in account the vision of the robot
and his detector (Ray Caster). A Robot have 2 detectors, vision_right and vision_left, and each one is composed of 8 rays.
Call in self.getVision() to get the distance to the hitting point for all of those rays.

#### For any question ask Arthur Oudeyer

