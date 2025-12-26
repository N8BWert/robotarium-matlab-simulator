classdef Robotarium < ARobotarium
    % Robotarium This object represents your communications with the 
    % GRITSbots.
    %
    % THIS CLASS SHOULD NEVER BE MODIFIED

    properties (GetAccess = private, SetAccess = private)
        checked_poses_already = false % Whether GET_POSES has been checked this iteration
        called_step_already = true % Whether STEP has been called this iteration
        
        iteration = 0; % How many times STEP has been called
        errors = {}; % Accumulated errors for the simulation
    end

    methods
        function this = Robotarium(varargin)
            % ROBOTARIUM Initializes the object.
            % 
            %   ROBOTARIUM('NumberOfRobots', 4) creates a ROBOTARIUM
            %   object with 4 robots.
            %
            %   ROBOTARIUM('NumberOfRobots', 1, 'ShowFigure', false) 
            %   creates a ROBOTARIUM object with 1 robot and shows no 
            %   figure.
            %
            %   "UseDistanceSensors": Boolean indicating whether or not 
            %   the experiment will utilize the distance sensors on the
            %   robots.
            %   
            %   Example:
            %       r = Robotarium('NumberOfRobots', 10, 'ShowFigure',
            %       true)
            %
            %   Notes:
            %       The option NumberOfRobots should be a positive integer.
            %       The option ShowFigure should be a boolean value.       
            %       The option InitialConditions should be a 3 x
            %       NumberOfRobots matrix of initial poses.
            
            parser = inputParser;
            
            parser.addParameter('NumberOfRobots', -1);
            parser.addParameter('ShowFigure', true);
            parser.addParameter('FigureHandle', []);
            parser.addParameter('InitialConditions', []);
            parser.addParameter('UseDistanceSensors', false);
                        
            parse(parser, varargin{:})
            
            % The input will be validated by ARobotarium
            this = this@ARobotarium(parser.Results.NumberOfRobots, ...
                parser.Results.ShowFigure, parser.Results.FigureHandle, ...
                parser.Results.UseDistanceSensors);
            
            initial_conditions = parser.Results.InitialConditions;
            
            if(isempty(initial_conditions))
                initial_conditions = generate_initial_conditions(this.number_of_robots, ...
                    'Spacing', 1.5*this.robot_diameter, ...
                    'Width', this.boundaries(2)-this.boundaries(1)-this.robot_diameter, ...
                    'Height', this.boundaries(4)-this.boundaries(3))-this.robot_diameter;
            end
            
            assert(all(size(initial_conditions) == [3, this.number_of_robots]), 'Initial conditions must be 3 x %i', this.number_of_robots);            
            
            % Call initialize during initialization
            this.initialize(initial_conditions);
        end

        function poses = get_poses(this)
            % GET_POSES Returns the current poses of the robots
            %
            %   GET_POSES() returns a 3 x NUMBER_OF_ROBOTS matrix of poses
            %
            %   Example:
            %       x = this.get_poses()
            %
            %   Notes:
            %       This function should only be called once per call of
            %       STEP
            
            assert(~this.checked_poses_already, 'Can only call get_poses() once per call of step()!');

            poses = this.poses;

            %Make sure it's only called once per iteration
            this.checked_poses_already = true;
            this.called_step_already = false;
        end

        % function intersection_point = line_intersection_brute_force(this, A, B, C, D)
        %     % LINE_INTERSECTION Computes the intersection of a line with
        %     % the environment boundaries
        %     %
        %     %   LINE_INTERSECTION()
        %     %   Input: A, B - 2D endpoints of line segment 1
        %     %          C, D - 2D endpoints of line segment 2
        %     %
        %     %   Example:
        %     %       object.line_intersection()
        %     %
        %     %   Notes:
        %     %       Used in distance sensor simulation
        %     intersection_point = [NaN; NaN];

        %     r = B - A;
        %     s = D - C;
        %     q = C - A;

        %     rxs = r(1)*s(2) - r(2)*s(1);
        %     qxs = q(1)*s(2) - q(2)*s(1);
        %     qxr = q(1)*r(2) - q(2)*r(1);

        %     if rxs == 0
        %         % Lines are parallel
        %         return;
        %     end

        %     t = qxs / rxs; % Parameter for the intersection on line AB
        %     u = qxr / rxs; % Parameter for the intersection on line CD

        %     if (t >= 0 && t <= 1) && (u >= 0 && u <= 1)
        %         % There is an intersection within the line segments
        %         intersection_point = A + t * r;
        %     end
            
        % end

        function simulate_distance_measurements(this)
            % UPDATE_DISTANCE_MEASUREMENTS Simulates the distance sensor
            % measurements for each robot based on the current robot poses
            % and the environment.

            N_sensors = size(this.distance_sensors_orientation, 2);
            this.distances = NaN(1, 7, this.number_of_robots); % 7 sensors per robot
            % this.intersection_point = NaN(2, 7, this.number_of_robots); % One 2x1 intersection point per sensor

            % Find global positions and orientations of distance sensors
            R = rotation_matrix(this.poses(3,:));
            poses = reshape(this.poses, 3, 1, this.number_of_robots); % 3 x 1 x N for pagemtimes
            global_sensors = poses + pagemtimes(R, this.distance_sensors_orientation);

            % Calculate the endpoints of each sensor ray at max range
            R_sensor = rotation_matrix(global_sensors(3,:,:));
            max_distances = [this.distance_sensor_range * ones(1, 1, this.number_of_robots*N_sensors); ...
                             zeros(1, 1, this.number_of_robots*N_sensors); ...
                             zeros(1, 1, this.number_of_robots*N_sensors)];
            sensor_endpoints_local = pagemtimes(R_sensor, max_distances);
            sensor_endpoints_local = reshape(sensor_endpoints_local, 3, N_sensors, this.number_of_robots);
            sensor_endpoints = global_sensors(1:2, :, :) + sensor_endpoints_local(1:2, :, :);

            % Compute intersections of each sensor ray with each obstacle
            r_all = sensor_endpoints - global_sensors(1:2, :, :); % Vectors from sensor to endpoints
            obstacles = [-1.6, -1.6; -1, 1]; % Start point and end point of each obstacle line segment
            obstacles(:, :, end+1) = [-1.6, 1.6; 1, 1];
            obstacles(:, :, end+1) = [1.6, 1.6; 1, -1];
            obstacles(:, :, end+1) = [1.6, -1.6; -1, -1];
            s_all = obstacles(:,2,:) - obstacles(:,1,:); % Vectors for obstacle line segments

            for i = 1:this.number_of_robots
                rxs = pagemtimes(r_all(1,:,i), s_all(2,:,:)) - pagemtimes(r_all(2,:,i), s_all(1,:,:)); % Cross products. 1 x N_sensors x num_obstacles
                q = obstacles(:,1,:) - global_sensors(1:2, :, i); % Vectors from sensor to obstacle start points
                qxs = pagemtimes(q(1,:,:), s_all(2,:,:)) - pagemtimes(q(2,:,:), s_all(1,:,:)); % Cross products. 1 x N_sensors x num_obstacles
                qxr = q(1,:,:).*r_all(2,:,i) - q(2,:,:).*r_all(1,:,i); % Cross products. 1 x N_sensors x num_obstacles

                t = qxs./rxs; % Parameter for the intersection on the sensor lines
                u = qxr./rxs; % Parameter for the intersection on the obstacle lines

                parameter_on_line = (t >= 0 & t <= 1) & (u >= 0 & u <= 1);
                valid_parameter = t.*parameter_on_line;
                valid_parameter(~parameter_on_line) = this.distance_sensor_range; % Set invalid intersections to NaN
                min_parameter = min(valid_parameter, [], 3); % Minimum t value for each sensor
                this.distances(1, :, i) = min_parameter;
            end

            % Find the endpoints of each sensor ray
            this.distance_end_points = global_sensors(1:2, :, :) + this.distances.*r_all;
        end

        % function update_distance_sensors(this)
        %     % UPDATE_DISTANCE_SENSORS Updates the simulated measurements from
        %     % the distance sensors of the robots
        %     %
        %     %   UPDATE_DISTANCE_SENSORS()
        %     %
        %     %   Example:
        %     %       object.update_distance_sensors()
        %     %
        %     %   Notes:
        %     %       Should be called within STEP after poses are updated
        %     N = this.number_of_robots;
        %     sensors = this.distance_sensors_orientation;

        %     distances = reshape(this.distances, 1, []);
        %     dist_size = size(distances);
        %     distances = [distances; zeros(2,dist_size(2))];  

        %     end_points = NaN(2, dist_size(2));

            
        %     for g = 1:N
                
        %         % takes current oriention of robot g to transform sensor vectors
        %         % R = angle2dcm(0, 0, x(3,g))
        %         R = [cos(this.poses(3,g)), -sin(this.poses(3,g)), 0;
        %             sin(this.poses(3,g)),  cos(this.poses(3,g)), 0;
        %                     0,           0,  1];
                
        %         % global x and y positions of distance sensors 1-7 for robot g
        %         global_sensors = this.poses(:,g) + R * sensors;
                
        %         for h = 1:7
        %             % takes global sensor orientation to find end point of ray
        %             % sensor_angle = angle2dcm(0, 0, global_sensors(3,h))
        %             sensor_angle = [cos(global_sensors(3,h)), -sin(global_sensors(3,h)), 0;
        %                             sin(global_sensors(3,h)),  cos(global_sensors(3,h)), 0;
        %                                                 0,           0,               1];
                    
        %             if distances(1, 7*(g-1)+h) ~= -1 
        %                 % finds global sensor end point
        %                 sensor_point = sensor_angle * distances(:, 7*(g-1)+h);
                        
        %                 % finds global sensor end point
        %                 global_sensor_point = global_sensors(1:2,h) + sensor_point(1:2);
                
        %                 end_points(:,7*(g-1)+h) = global_sensor_point;
        %             end

        %         end

        %     end

        %     this.distance_end_points = end_points;
        % end
        
        function initialize(this, initial_conditions)
            this.poses = initial_conditions;
        end

        function step(this)
            % STEP Steps the simulation, updating poses of robots and
            % checking for errors
            %
            %   STEP()
            %
            %   Example:
            %       object.step()
            %
            %   Notes:
            %       Should be called everytime GET_POSES is called
            
            assert(~this.called_step_already, 'Make sure you call get_poses before calling step!');

            %Vectorize update to states
            i = 1:this.number_of_robots;
                        
            % Validate before thresholding velocities
            es = this.validate();
            this.errors = [this.errors, es];
            this.iteration = this.iteration + 1;
            
            this.velocities = this.threshold(this.velocities);

            %Update velocities using unicycle dynamics
            temp = this.time_step.*this.velocities(1, i);
            this.poses(1, i) = this.poses(1, i) + temp.*cos(this.poses(3, i));
            this.poses(2, i) = this.poses(2, i) + temp.*sin(this.poses(3, i));
            this.poses(3, i) = this.poses(3, i) + this.time_step.*this.velocities(2, i);

            %Ensure that the orientations are in the right range
            this.poses(3, i) = atan2(sin(this.poses(3, i)), cos(this.poses(3, i)));

            % Update distance sensors if enabled
            if this.distance_sensors_enabled
                % this.update_distance_sensors();
                this.simulate_distance_measurements();
            end

            %Allow getting of poses again
            this.checked_poses_already = false;
            this.called_step_already = true;            
            
            if(this.show_figure)
                this.draw_robots();
                uistack([this.robot_handle{:}],'top');
            end            
        end
        
        function debug(this)
            num_errors = 3;
            count = zeros(1, num_errors);
            for i = 1:numel(this.errors)
              count(this.errors{i}) = count(this.errors{i}) + 1;                 
            end
            
            fprintf('Your simulation took approximately %.2f real seconds.\n', this.iteration*this.time_step);
            
            error_strings = cell(1, num_errors);
            error_strings{RobotariumError.RobotsTooClose} = 'time steps where robots were too close (potential collision)';
            error_strings{RobotariumError.RobotsOutsideBoundaries} = 'time steps where robots were outside boundaries'; 
            error_strings{RobotariumError.ExceededActuatorLimits} = 'staged velocity commands exceeded actuator limits';                
            
            fprintf('Error count for current simulation:\n');
            print_error = @(x) fprintf('\t Simulation had %i %s errors.\n', count(x), error_strings{x});            
            print_error(RobotariumError.RobotsTooClose)
            print_error(RobotariumError.RobotsOutsideBoundaries);
            print_error(RobotariumError.ExceededActuatorLimits);
            
            if(isempty(this.errors))
                fprintf('No errors in your simulation!  Acceptance of experiment likely.\n')               
            else
                fprintf('Please fix the noted errors in your simulation; otherwise, your experiment may be rejected.\n');
            end            
        end
    end
end
