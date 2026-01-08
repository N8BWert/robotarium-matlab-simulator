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

        B % Function handle for magnetic field interpolation
        starting_orientations % Initial relative orientations of robots when powered on
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
            parser.addParameter('Obstacles', [NaN, NaN; NaN, NaN]); % Default no obstacles
                        
            parse(parser, varargin{:})
            
            % The input will be validated by ARobotarium
            this = this@ARobotarium(parser.Results.NumberOfRobots, ...
                parser.Results.ShowFigure, parser.Results.FigureHandle, ...
                parser.Results.UseDistanceSensors, ...
                parser.Results.Obstacles);
            
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

            % Draw obstacles if any
            if ~isempty(this.obstacles) && this.show_figure
                hold(this.figure_handle.Children, 'on');
                line_objects = gobjects(1, size(this.obstacles, 3));
                for i = 1:size(this.obstacles, 3)
                    line_objects(i) = plot(this.figure_handle.Children, ...
                        this.obstacles(1, :, i), this.obstacles(2, :, i), 'Color', [0.5, 0.5, 0.5], 'LineWidth', 2);
                end
                hold(this.figure_handle.Children, 'off');
            end

            % Initialize magnetic field grid
            load('recorded_magnetic_fields_world_frame.mat')
            N_x_points = length(recorded_magnetic_fields.x_points);
            N_y_points = length(recorded_magnetic_fields.y_points);
            Bx = reshape(recorded_magnetic_fields.B(1,:), [N_y_points, N_x_points]);
            By = reshape(recorded_magnetic_fields.B(2,:), [N_y_points, N_x_points]);
            Bz = reshape(recorded_magnetic_fields.B(3,:), [N_y_points, N_x_points]);

            Fx = griddedInterpolant({recorded_magnetic_fields.y_points, recorded_magnetic_fields.x_points}, Bx, 'linear', 'nearest');
            Fy = griddedInterpolant({recorded_magnetic_fields.y_points, recorded_magnetic_fields.x_points}, By, 'linear', 'nearest');
            Fz = griddedInterpolant({recorded_magnetic_fields.y_points, recorded_magnetic_fields.x_points}, Bz, 'linear', 'nearest');

            this.B = @(x) [Fx(x(2,:), x(1,:)); Fy(x(2,:), x(1,:)); Fz(x(2,:), x(1,:))];

            % Initialize starting orientations randomly
            this.starting_orientations = (2*randi([0, 1], 1, this.number_of_robots) - 1)*pi/2;
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

        function simulate_distance_measurements(this)
            % UPDATE_DISTANCE_MEASUREMENTS Simulates the distance sensor
            % measurements for each robot based on the current robot poses
            % and the environment.

            N_sensors = size(this.distance_sensors_orientation, 2);
            this.distances = NaN(1, 7, this.number_of_robots); % 7 sensors per robot

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
            s_all = this.obstacles(:,2,:) - this.obstacles(:,1,:); % Vectors for obstacle line segments

            for i = 1:this.number_of_robots
                rxs = pagemtimes(r_all(1,:,i), s_all(2,:,:)) - pagemtimes(r_all(2,:,i), s_all(1,:,:)); % Cross products. 1 x N_sensors x num_obstacles
                q = this.obstacles(:,1,:) - global_sensors(1:2, :, i); % Vectors from sensor to obstacle start points
                qxs = pagemtimes(q(1,:,:), s_all(2,:,:)) - pagemtimes(q(2,:,:), s_all(1,:,:)); % Cross products. 1 x N_sensors x num_obstacles
                qxr = q(1,:,:).*r_all(2,:,i) - q(2,:,:).*r_all(1,:,i); % Cross products. 1 x N_sensors x num_obstacles

                t = qxs./rxs; % Parameter for the intersection on the sensor lines
                u = qxr./rxs; % Parameter for the intersection on the obstacle lines

                parameter_on_line = (t >= 0 & t <= 1) & (u >= 0 & u <= 1);
                valid_parameter = t.*parameter_on_line;
                valid_parameter(~parameter_on_line) = NaN; % Set invalid intersections to NaN
                min_parameter = min(valid_parameter, [], 3); % Minimum t value for each sensor

                % Check if any rays intersect other robots
                f = global_sensors(1:2,:,i) - poses(1:2, :, :); % Vectors from sensors to robot centers. 2 x N_sensors x N_robots
                a = dot(r_all(:,:,i),r_all(:,:,i)); % Squared magnitudes of ray vectors. 1 x N_sensors
                b = 2*sum(f.*r_all(:,:,i), 1); % 2 * dot product of f and r. 1 x N_sensors x N_robots
                c = dot(f,f) - (this.robot_diameter/2)^2; % Squared distance from sensor to robot center minus squared radius
                discriminant = b.^2 - 4*a.*c; % Discriminant of quadratic
                t_circle = (-b - sqrt(discriminant))./(2*a); % Parameter values for intersection points
                parameter_on_line_circle = (t_circle >= 0 & t_circle <= 1 & imag(t_circle) == 0); % Check if intersection points are on the ray
                valid_parameter_circle = t_circle.*parameter_on_line_circle;
                valid_parameter_circle(~parameter_on_line_circle) = NaN; % Set invalid intersections to NaN

                valid_parameter_all = cat(3, valid_parameter, valid_parameter_circle); % Combine obstacle and robot intersection parameters
                min_parameter = min(valid_parameter_all, [], 3); % Minimum t value for each

                this.distances(1, :, i) = min_parameter.*(1 + this.distance_sensor_error*(2*rand(1, size(valid_parameter_all, 2)) - 1)); % Add noise to distance measurements
            end

            % Find the endpoints of each sensor ray
            this.distance_end_points = global_sensors(1:2, :, :) + this.distances.*r_all;

            % Convert NaN distances to -1 for consistency with real robot API
            this.distances(isnan(this.distances)) = -1;
            this.distances = reshape(this.distances, [7, this.number_of_robots]);
        end

        function simulate_encoder_readings(this)
            % SIMULATE_ENCODER_READINGS Simulates the encoder readings
            % based on the current robot velocities.
            
            % Convert linear and angular velocities to motor angular velocities
            left_motor_angular_velocity = (2*this.velocities(1, :) - this.base_length*this.velocities(2, :))./(2*this.wheel_radius);
            right_motor_angular_velocity = (2*this.velocities(1, :) + this.base_length*this.velocities(2, :))./(2*this.wheel_radius);

            delta_encoder = this.encoder_counts_per_revolution*this.motor_gear_ratio/(2*pi)*[left_motor_angular_velocity; right_motor_angular_velocity]*this.time_step;
            encoders = this.encoders + round(delta_encoder);

            this.encoders = encoders;
        end

        function simulate_imu_measurements(this)
            % SIMULATE_IMU_MEASUREMENTS Simulates the IMU measurements
            % based on the current robot poses and velocities.
            %
            %   Notes:
            %   The IMU axis may need to be adjusted. It seems weird and inconsistent with the datasheet. It is now empirical yet certain.
            %   Accelerometer axes (in robot frame):
            %      X-axis: Left
            %      Y-axis: Backward
            %      Z-axis: Down
            %
            %   Magnetometer axes (in robot frame):
            %      X-axis: Right
            %      Y-axis: Forward
            %      Z-axis: Up
            %
            %   Fused Orientation axes (in robot frame):
            %      Roll: Yaw
            %      Pitch: Roll
            %      Yaw: Pitch
            %
            % The IMU simulation is now noise-free for easier debugging.

            % Compute accelerations
            linear_accelerations = (this.velocities(1,:) - this.velocities_old(1,:))/this.time_step; % 1 x N_robots
            angular_accelerations = (this.velocities(2,:) - this.velocities_old(2,:))/this.time_step; % 1 x N_robots

            linear_accelerations_3d = [linear_accelerations; zeros(1, this.number_of_robots); 9.81*ones(1, this.number_of_robots)]; % 3 x N_robots
            angular_accelerations_3d = [zeros(1, this.number_of_robots); zeros(1, this.number_of_robots); angular_accelerations]; % 3 x N_robots
            angular_velocities_3d = [zeros(1, this.number_of_robots); zeros(1, this.number_of_robots); this.velocities(2,:)]; % 3 x N_robots

            axle_to_imu_vector = [this.imu_orientation(1)*ones(1, this.number_of_robots); ...
                                  this.imu_orientation(2)*ones(1, this.number_of_robots); ...
                                  zeros(1, this.number_of_robots)]; % Vector from axle center to IMU in robot frame. 3 x N_robots

            imu_accelerations = linear_accelerations_3d + ...
                                [angular_accelerations_3d(2,:).*axle_to_imu_vector(3,:) - angular_accelerations_3d(3,:).*axle_to_imu_vector(2,:); ...
                                 angular_accelerations_3d(3,:).*axle_to_imu_vector(1,:) - angular_accelerations_3d(1,:).*axle_to_imu_vector(3,:); ...
                                 angular_accelerations_3d(1,:).*axle_to_imu_vector(2,:) - angular_accelerations_3d(2,:).*axle_to_imu_vector(1,:)] + ... % 3 x N_robots. Acceleration due to angular acceleration
                                 angular_velocities_3d.*sum(angular_velocities_3d.*axle_to_imu_vector, 1) - ... % 3 x N_robots. Centrifugal acceleration
                                 sum(angular_velocities_3d.^2, 1).*axle_to_imu_vector; % 3 x N_robots. Centripetal acceleration

            % Convert accelerations from robot frame to sensor frame
            imu_accelerations_sensor_frame = [imu_accelerations(2,:); imu_accelerations(1,:); imu_accelerations(3,:)]; % 3 x N_robots

            % Add noise to accelerometer readings
            this.accelerations = imu_accelerations_sensor_frame; % + this.imu_acceleration_noise*randn(size(imu_accelerations));

            % Compute magnetic field readings
            magnetic_fields_world_frame_parallel = reshape(this.B(this.poses(1:2, :)), 3, 1, this.number_of_robots); % 3 x 1 x N_robots
            R_rw = pagetranspose(rotation_matrix(this.poses(3,:))); % From world frame to robot frame
            magnetic_fields_robot_frame_parallel = pagemtimes(R_rw, magnetic_fields_world_frame_parallel);
            this.magnetic_fields = reshape(magnetic_fields_robot_frame_parallel, 3, this.number_of_robots);

            % Simulate orientation readings
            orientation_yaw = mod((this.poses(3, :)*(180/pi) + 360 + this.starting_orientations*(180/pi) + 360), 360); % Degrees in [0, 360)
            orientation_roll = zeros(1, this.number_of_robots); % No roll
            orientation_pitch = zeros(1, this.number_of_robots); % No pitch
            this.orientations = [orientation_yaw; orientation_roll; orientation_pitch]; % Degrees. Robots are powered on at either 90 or 180 degrees.

            % Update old velocities
            this.velocities_old = this.velocities;
        end
        
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
                this.simulate_distance_measurements();
            end

            % Update encoder readings
            this.simulate_encoder_readings();

            % Update IMU measurements
            this.simulate_imu_measurements();

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
