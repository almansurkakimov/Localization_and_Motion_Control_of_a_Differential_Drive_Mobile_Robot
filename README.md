# robotics_project

The project was carried out within the framework of the discipline "Introduction to Intelligent Systems" at Astana IT University. In this project you can find an explanation of the MATLAB code and implementation.

1. Topic: Localization and Motion Control of a Differential-Drive Mobile Robot
2. Student: Almansur Kakimov
3. Instructor: Sanzhar Kusdavletov
4. Discipline: Introduction to Intelligent Systems
5. Department of Intelligent Systems and Cybersecurity, Astana IT University
7. February 2025

- Algorithms Used
1. Localization:	Odometry-based tracking
2. Path Planning:	A* Algorithm
3. Path Smoothing:	Bézier Curve Interpolation
4. Motion Control:	Angle-based rotation + speed control
65. Obstacle Avoidance:	Proximity-based reactive control

1. Initialization and Setup
The program starts by clearing the workspace and closing any open figures to ensure a fresh environment. It then defines key parameters such as:

Map size: A 10x10 grid.
Robot properties: Its radius and speed.
Obstacles: Randomly placed obstacles covering 15% of the map.
Goals: Three random target points that are not occupied by obstacles.
The map is initialized with a grid of zeros, and obstacles are randomly distributed. Then, three goal points are generated in unoccupied locations using the function generateRandomGoals.

2. Visualization
To provide a clear view of the simulation, a figure is created where:

Goals are plotted in green circles.
Obstacles appear as red circles.
The robot is initially positioned at (1,1) and is represented as a blue circle.
A path line is also initialized to track the robot’s movement across the grid.

3. Main Loop: Navigating Through Goals
The core logic of the program is contained within a loop that iterates through each goal. For every goal:

The function navigateToGoal is called, which moves the robot step by step toward its destination while avoiding obstacles.
4. Pathfinding Using A Algorithm*
To determine an efficient path to the goal, the program employs the A (A-star) pathfinding algorithm*, implemented in aStarPathfinding.

It maintains an open list of possible paths and a came-from map to reconstruct the shortest path.
It evaluates movement costs using a heuristic function (Euclidean distance) to guide the robot efficiently toward the goal.
If a path is found, it is further smoothed using Bézier curves, which help create a more natural, curved trajectory instead of a rigid stepwise movement.

5. Robot Motion and Obstacle Avoidance
The robot moves along the calculated path while obeying realistic motion constraints:

Gradual turns: Instead of instantly changing direction, the robot turns incrementally using a maximum turn rate.
Speed control: The robot's movement is adjusted to stay within its maximum speed.
Obstacle avoidance: If an obstacle is encountered, the reactiveAvoidance function shifts the robot away in a perpendicular direction to prevent collisions.
The robot’s position and orientation are continuously updated and reflected in the visualization.

6. Completion and Finalization
Once the robot reaches a goal, a message "Goal X reached!" is displayed. After all goals are reached, the program prints "Simulation complete!".

7. Additional Functionalities
Several additional helper functions improve the simulation:

Path smoothing: bezierSmoothPath ensures smoother turns and movement.
Odometry updates: updateOdometry simulates real-world tracking of movement errors.
Real-time plotting: Ensures the robot’s progress is displayed dynamically.
Conclusion
In summary, this program showcases an effective way to implement autonomous robot navigation in a grid-based environment. By combining A pathfinding, Bézier curve smoothing, and reactive obstacle avoidance*, the robot efficiently reaches its goals while dynamically adjusting its trajectory.

This simulation is a stepping stone for real-world robotics applications, such as autonomous delivery robots, warehouse navigation, and robotic vacuum cleaners.
