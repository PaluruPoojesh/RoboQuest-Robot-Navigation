ROBOQUEST – ROBOT NAVIGATION AND OBSTACLE AVOIDANCE
MegaMinds IT Services | Robotics Internship Practical Assessment

PROJECT
RoboQuest is a 2D robot-navigation simulation made with Python and Pygame.
The robot can be driven with the keyboard or sent to the goal using A* pathfinding.
The arena includes fixed obstacles, user-placed obstacles, temporary obstacles,
rotating laser hazards and a simulated LiDAR display.

REQUIREMENTS
- Python 3.10 or newer
- Pygame 2.x

SETUP
1. Extract all files from the ZIP folder.
2. Open a terminal in the extracted folder.
3. Install the dependency:

   python -m pip install -r requirements.txt

RUN
   python Poojesh_robot.py

CONTROLS
- M: switch between manual and autonomous mode
- W, A, S, D or arrow keys: move the robot in manual mode
- P: calculate or recalculate the route
- Left-click an arena cell: add or remove a user obstacle
- Space: pause or resume
- R: restart the mission
- H: show or hide the mission help
- Esc: close the simulation

HOW IT WORKS
The arena is divided into a grid. A* searches this grid for a route from the
robot's current cell to the goal. It uses the Manhattan distance as its heuristic
because movement is planned in four directions. In autonomous mode, the robot
follows the route one cell at a time. When an obstacle appears or the hazard map
changes, the route is recalculated. Collision checks keep the robot inside the
arena and prevent it from passing through solid obstacles.

The LiDAR display is simulated in software; no physical LiDAR sensor is used.
Temporary obstacles first show an orange warning and then become active for a
limited time. Laser hazards rotate and can reduce the robot's health on contact.

LIMITATIONS
This is a 2D simulation, not a physical robot model. The LiDAR, laser hazards
and movement are simplified. A* uses a four-direction grid and does not model
wheel slip, acceleration or real sensor noise. Performance may vary by computer.

FILES
- Poojesh_robot.py: simulation source code
- Poojesh_demo.mp4: recording of the simulation
- README.txt: setup and project notes
- requirements.txt: Python dependency list

NOTE
AI assistance was used during development. The implementation was reviewed
and adjusted for this assessment. I can explain the main functions and make
changes to the code. Any required AI-use disclosure should be made according
to the assessment instructions.
