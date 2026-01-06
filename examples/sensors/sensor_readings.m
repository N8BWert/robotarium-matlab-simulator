%% GTernal (New Robotarium Robot) Sensor Readings Example
% Soobum Kim
% 1/6/2026
% This script demonstrates how to obtain various sensor readings from the
% GTernal robot in the Robotarium simulator. A single robot navigates through
% a field of obstacles while collecting distance sensor data.

%% Experiment Constants and Parameters
N = 1; % Number of robots

% Create obstacles to be used for distance sensor simulation using line segments. Each obstacle 
% is defined by a set of two points (start and end) which are represented as columns in a 2x2 array,
% and multiple obstacles are stacked along the 3rd dimension, i.e., a 2x2x(num_obstacles) array.
% Currently, there is no support for collision simulation with these obstacles; they are only used for 
% simulating distance sensor readings.
% NOTE: The rectangular boundary of the Robotarium arena is not treated as an obstacle for distance sensors
% by default. If you wish to include the boundary as obstacles, you can add them manually.
% The rectangular boundary of the Robotarium arena is defined by the following coordinates:
% Bottom-left corner: (-1.6, -1)
% Bottom-right corner: (1.6, -1)
% Top-right corner: (1.6, 1)
% Top-left corner: (-1.6, 1)
obstacles = cat(3, [-1.0, -1.0; -0.5, 0.5], ... 
                   [-0.6, -0.6; -0.5, 0.5], ...
                   [0.2, 0.2; -0.5, 0.5], ...
                   [1.0, 1.0; -0.5, 0.5], ...
                   [-1.4, -1.4; -0.9, 0.9], ...
                   [-1.4, 1.4; 0.9, 0.9], ...
                   [1.4, 1.4; 0.9, -0.9], ...
                   [1.4, -1.4; -0.9, -0.9]); % Example obstacles

initial_conditions = [-1.2; -0.6; 0]; % Initial pose: [x; y; theta]

%% Instantiate Robotarium object
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions, ...
               'UseDistanceSensors', true, 'Obstacles', obstacles);

%% Generate goal points in a grid pattern
resolution_x = 0.4;
resolution_y = 0.4;
x_points = -1.2:resolution_x:1.2;
y_points = -0.6:resolution_y:0.6;
[x_goals, y_goals] = meshgrid(x_points, y_points);
y_goals(:, 2:2:end) = -y_goals(:, 2:2:end); % Serpentine pattern

%% Grab tools we need to convert from single-integrator to unicycle dynamics
% Single-integrator position controller
controller = create_si_position_controller();

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dynamics = create_si_to_uni_dynamics();

% Initialization checker for checking if the robot has reached the goal
init_checker = create_is_initialized('PositionError', 0.025, 'RotationError', 2*pi);

%% Initialize data structure for recording sensor data
data = struct();
data.poses = zeros(3,1);
data.distances = zeros(7,1);
data.accelerations = zeros(3,1);
data.magnetic_fields = zeros(3,1);
data.encoders = zeros(2,1);

%% Main Experiment Loop
index = 1; % Index for goal points

while index <= numel(x_goals)
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();

    % Define the current goal
    goal = [x_goals(index); y_goals(index)];

    % Check if the robot has reached the goal and record sensor data
    if init_checker(x, [goal; 0])
        % The poses are already stored in x through r.get_poses()
        poses = x;

        % r.get_distances() returns distances in shape (N_sensors, N_robots) where N_sensors=7.
        % Max range is 1.2 meters, and -1 indicates no obstacle detected within range.
        % The orientations of the sensors in robot frame are
        %          [[-0.04, 0.0,  0.04, 0.05, 0.04,   0.0,   -0.04],
        %           [ 0.04, 0.06, 0.05, 0.0,  -0.05, -0.06, -0.04],
        %           [ pi, pi/2, pi/4, 0.0,  -pi/4, -pi/2, -pi]])
        distances = r.get_distances(); % 7 x N_robots

        % r.get_accelerations() returns accelerations in shape (3, N_robots) in IMU frame.
        % r.get_magnetic_fields() returns magnetic field readings in shape (3, N_robots) in IMU frame.
        % The magnetic field reading is simulated based on recorded data over the Robotarium arena, and 
        % the numbers are in microteslas (uT).
        % r.get_orientations() returns fused orientation (yaw, roll, pitch) in shape (3, N_robots) in IMU frame.
        % The orientations are relative, and they depend on the orientation of the charging station of the robot.
        % A robot's 0 degree is either 90 degrees or 180 degrees relative to the global frame. 
        % The position of the IMU is [[0.0594 - 0.00319], [0.0344628 - 0.0475], [0.0]] in robot frame.
        %       Accelerometer axes (in robot frame):
        %       X-axis: Left
        %       Y-axis: Backward
        %       Z-axis: Down
        %
        %       Magnetometer axes (in robot frame):
        %       X-axis: Right
        %       Y-axis: Forward
        %       Z-axis: Up
        %
        %       Fused Orientation axes (in robot frame):
        %       Roll: Yaw
        %       Pitch: Roll
        %       Yaw: Pitch
        %
        % The IMU simulation is now noise-free for easier debugging.
        accelerations = r.get_accelerations(); % 3 x N_robots
        magnetic_fields = r.get_magnetic_fields(); % 3 x N_robots
        orientations = r.get_orientations(); % 3 x N_robots

        % r.get_encoders() returns wheel encoder readings in shape (2, N_robots) as [left_wheel; right_wheel] in ticks.
        % There are 28 ticks per revolution, and the motor gear ratio is 100.37:1, resulting in approximately
        % 2810 ticks per wheel revolution.
        % Differential drive geometry parameters:
        % Wheel radius: 0.016 m, Wheelbase: 0.105 m
        encoders = r.get_encoders(); % 2 x N_robots

        % Store the retrieved sensor data
        data.poses(:, index) = poses;
        data.distances(:, index) = distances;
        data.accelerations(:, index) = accelerations;
        data.magnetic_fields(:, index) = magnetic_fields;
        data.orientations(:, index) = orientations;
        data.encoders(:, index) = encoders;

        % Increment goal index
        index = index + 1;
    end

    % Compute single-integrator control inputs
    dxi = controller(x(1:2,:), goal);

    % Convert to unicycle dynamics
    dxu = si_to_uni_dynamics(dxi, x);

    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

end

% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();