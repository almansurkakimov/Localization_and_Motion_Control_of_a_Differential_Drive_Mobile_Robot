% Path Following for a Differential Drive Robot using Pure Pursuit

% Define Waypoints
path = [2.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];

% Set the current location and the goal location of the robot
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% Initial robot orientation (angle in radians)
initialOrientation = 0;

% Define current pose [x y theta] of the robot
robotCurrentPose = [robotInitialLocation initialOrientation]';

% Create a Kinematic Robot Model (Differential Drive)
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Visualize the desired path
figure
plot(path(:,1), path(:,2), 'k--d')
xlim([0 13])
ylim([0 13])

% Define the Path Following Controller using Pure Pursuit
controller = controllerPurePursuit;

% Set the waypoints for the controller
controller.Waypoints = path;

% Set the parameters for the controller
controller.DesiredLinearVelocity = 0.6; % meters per second
controller.MaxAngularVelocity = 2; % radians per second
controller.LookaheadDistance = 0.3; % meters

% Define a goal radius (threshold for stopping the robot)
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;  % seconds
vizRate = rateControl(1/sampleTime);

% Initialize the figure for plotting
figure

% Determine vehicle frame size for the plot
frameSize = robot.TrackWidth/0.8;

% Loop until the robot reaches the goal
while distanceToGoal > goalRadius
    % Compute the controller outputs (linear and angular velocities)
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the robot's current pose
    robotCurrentPose = robotCurrentPose + vel * sampleTime; 
    
    % Recompute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    plot(path(:,1), path(:,2), "k--d") % Plot the desired path
    hold all
    
    % Plot the robot's path
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View", "2D", "FrameSize", frameSize);
    
    % Update the plot settings
    light;
    xlim([0 13])
    ylim([0 13])
    
    % Pause to simulate real-time
    waitfor(vizRate);
end

% After reaching the goal, stop and display the final position
disp('Robot reached the goal!');

% Now using Path Following Controller Along with PRM (Probabilistic Roadmap)
% Load a sample map for path planning
load exampleMaps
map = binaryOccupancyMap(simpleMap);

% Display the map
figure
show(map)

% Inflate the map for robot's physical size
mapInflated = copy(map);
inflate(mapInflated, robot.TrackWidth/2);

% Initialize the PRM (Probabilistic Roadmap)
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;

% Define start and end locations
startLocation = [4.0 2.0];
endLocation = [24.0 20.0];

% Find a path using the PRM
path = findpath(prm, startLocation, endLocation);

% Display the PRM roadmap and the computed path
show(prm);
hold on
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2)

% Redefine the controller with new waypoints from PRM path
release(controller);
controller.Waypoints = path;

% Set the initial location and goal location based on the PRM path
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
robotCurrentPose = [robotInitialLocation initialOrientation]';
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Reinitialize the goal radius
goalRadius = 0.1;

% Drive the robot using the controller output on the new map
reset(vizRate);
figure

% Loop to drive the robot along the PRM path
while distanceToGoal > goalRadius
    % Compute the controller outputs (linear and angular velocities)
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the robot's current pose
    robotCurrentPose = robotCurrentPose + vel * sampleTime;
    
    % Recompute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(map);  % Display the map
    hold all
    
    % Plot the desired path
    plot(path(:,1), path(:,2), "k--d")
    
    % Plot the robot's path
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    % Update the plot settings
    light;
    xlim([0 27])
    ylim([0 26])
    
    % Pause to simulate real-time
    waitfor(vizRate);
end

% After reaching the goal, display the success message
disp('Robot reached the goal using the PRM path!');