%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

%% Get Robotarium object used to communicate with the robots/simulator
N = 1;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'UseDistanceSensors', true);

resolution_x = 0.2;
resolution_y = 0.2;
x_points = -1.5:resolution_x:1.5;
y_points = -0.9:resolution_y:0.9;

[x_goals, y_goals] = meshgrid(x_points, y_points);
y_goals(:, 2:2:end) = -y_goals(:, 2:2:end); % Serpentine pattern

controller = create_si_position_controller();
si_to_uni_dynamics = create_si_to_uni_dynamics();
init_checker = create_is_initialized('PositionError', 0.025, 'RotationError', 2*pi);
data = struct();
data.poses = zeros(3,1);
data.distances = zeros(7,1);
data.accelerations = zeros(3,1);
data.magnetic_fields = zeros(3,1);
data.encoders = zeros(2,1);

index = 1;

% Select the number of iterations for the experiment.  This value is
% arbitrary
% iterations = 1000;

% Iterate for the previously specified number of iterations
while index <= numel(x_goals)
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    %% Insert your code here!
    x = r.get_poses();
    goal = [x_goals(index); y_goals(index)];
    dxi = controller(x(1:2,:), goal);
    dxu = si_to_uni_dynamics(dxi, x);

    % dxu = algorithm(x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

    % Data recording
    x = r.get_poses();
    if init_checker(x, [goal; 0])
        data.poses(:, index) = x;
        data.distances(:, index) = r.get_distances();
        data.accelerations(:, index) = r.get_accelerations();
        data.magnetic_fields(:, index) = r.get_magnetic_fields();
        data.encoders(:, index) = r.get_encoders();

        index = index + 1;
    end
    r.step();
end

% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();