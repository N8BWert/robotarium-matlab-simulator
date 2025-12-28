%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

%% Get Robotarium object used to communicate with the robots/simulator
N = 2;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'UseDistanceSensors', true, 'InitialConditions', [1.2, 1.5; 0, 0; 0, pi]);
% r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'UseDistanceSensors', true, 'InitialConditions', [1.2; 0; 0]);
index = 1;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;

start = tic;
% Iterate for the previously specified number of iterations
for i = 1:iterations
    loop = tic;
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    %% Insert your code here!
    x = r.get_poses();
    dxu = [0, 0; 0.1, 0];
    % dxu = [0; 0.1];

    % dxu = algorithm(x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    i = i + 1;
    toc(loop);
end
toc(start)

% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();