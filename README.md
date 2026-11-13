# RoboQuest-Robot-Navigation
2D Robot Navigation and Obstacle Avoidance Simulation using Python, Pygame and A* Pathfinding.
ROBOQUEST – ROBOT NAVIGATION AND OBSTACLE AVOIDANCE
====================================================

OVERVIEW
--------
RoboQuest is a 2D robot-navigation simulation built with Python and Pygame.
It allows the robot to be controlled manually or navigate autonomously using
the A* pathfinding algorithm.

The simulation includes:
- Manual keyboard-based robot movement
- Autonomous navigation using A* pathfinding
- A grid-based arena with obstacles
- User-placed obstacles
- Temporary obstacles with warning and active phases
- Rotating laser hazards
- Simulated 360-degree LiDAR visualization
- Collision detection and robot health monitoring
- Automatic route recalculation when the path becomes blocked

REQUIREMENTS
------------
- Python 3.10 or newer
- Pygame 2.x

INSTALLATION
------------
1. Download or clone this repository.
2. Open a terminal or command prompt in the project folder.
3. Install the required Python package:

   python -m pip install -r requirements.txt

RUN THE APPLICATION
-------------------
Run the following command from the project folder:

   python Poojesh_robot.py

CONTROLS
--------
M                       Switch between manual and autonomous mode
W, A, S, D              Move the robot in manual mode
Arrow keys              Move the robot in manual mode
P                       Calculate or recalculate the route
Left mouse click        Add or remove an obstacle in an arena cell
Space                   Pause or resume the simulation
R                       Restart the mission
H                       Show or hide the mission help
Esc                     Close the simulation

HOW IT WORKS
------------
The arena is divided into a grid of cells. In autonomous mode, the A*
algorithm searches for a route from the robot's current position to the goal.
It uses Manhattan distance as its heuristic because movement is planned in
four directions: up, down, left and right.

The robot follows the calculated route. If an obstacle or hazard blocks the
route, the application can request a new path. Collision checks prevent the
robot from passing through solid obstacles or leaving the arena.

The LiDAR visualization is simulated in software. Rotating laser hazards
switch between active and inactive states. Contact with an active laser can
reduce the robot's health and return it to the starting position. Temporary
obstacles display a warning before becoming active for a limited period.

PROJECT FILES
-------------
Poojesh_robot.py   Main application source code
requirements.txt   Python package dependencies
README.txt         Project information and run instructions

NOTES
-----
This project is a 2D software simulation. It does not control a physical
robot or use a real LiDAR sensor. Robot movement, sensing and hazards are
simulated, so their behaviour is simplified compared with real hardware.
