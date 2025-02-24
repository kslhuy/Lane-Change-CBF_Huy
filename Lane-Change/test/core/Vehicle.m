classdef Vehicle < handle
    properties
        vehicle_number;
        % dynamics_flag; % 1 stands for ego-vehicle,
        %                 % 0 stands for normal surrounding vehicles (move at a constant speed or acceleration)
        %                 % 2 stands for surrounding vehicles that will change the lane
        %                 % 3 stands for folloing vehicle
        typeController;
        initial_lane_id; % initial lane id of the vehicle
        lane_id; % curren lane id of the vehicle
        param; % Parameters of the vehicle
        state; % current state of the vehicle (   X , Y , phi , velocity, acceleration        )
        state_log; % history of states
        input; % current input of the vehicle
        input_log; % history of inputs (speed and slip angle for the ego vehicle and lane change vehicle; speed and acceleartion for normal vehicles)
        controller; % controller used for lane changing
        dt; % time gap for simulation
        total_time_step; % total time steps for simulation
        lanes;
        direction_flag; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing
        % to the right adjacent lane
        acc_flag;
        scenarios_config;
        other_log;
        observer;
        other_vehicles;
        weight_module;
        num_vehicles;
        trip_models;
        center_communication; % Add a reference to the CenterCommunication object
        graph;
        trust_log;
        
    end
    methods
        function self = Vehicle(vehicle_number , typeController, veh_param, state_initial, initial_lane_id, lanes, direction_flag, acc_flag, scenarios_config,weight_module)
            
            self.scenarios_config = scenarios_config;
            self.dt = scenarios_config.dt;
            self.total_time_step = scenarios_config.simulation_time/self.dt;
            
            self.vehicle_number = vehicle_number ;
            
            self.state = state_initial;
            
            self.param = veh_param;
            
            % at first time step, the current input is same as initial input
            self.state_log = state_initial;
            
            % the first element of inputs' history
            self.initial_lane_id = initial_lane_id;
            self.lane_id = initial_lane_id;
            self.lanes = lanes;
            self.direction_flag = direction_flag;
            self.acc_flag = acc_flag;
            
            self.other_log = [initial_lane_id; 1];
            
            self.typeController = typeController;
            % at first time step, the current state of the vehicle is same to initial state
            self.input = [0;0]; % at first time step, the current input is same as initial input
            % the first element of states' history
            self.input_log = self.input;
            
            self.weight_module = weight_module;
            
            
            
            
            
            
            
        end
        
        function update(self,instant_index)
            
            %% Identify vehicles directly measurable by the ego vehicle's sensor
            connected_vehicles = find(self.graph(self.vehicle_number, :) == 1); % Get indices of connected vehicles
            %% Calculate trust and opinion scores directly in self.trust_log
            calculateTrustAndOpinion(self, instant_index, connected_vehicles);

            %% Wiegth trust update
            % self.weight_module.calculate_weights_Trust(self.vehicle_number , self.trust_log(1, instant_index, :));
            weights = self.weight_module.calculate_weights_Defaut(self.vehicle_number); % Matrix ('nb_vehicles + 1' x 'nb_vehicles + 1' )
            weights_Dis = weights(self.vehicle_number + 1,:); % array (1 x 'nb_vehicles + 1' ) 
            %% Update normal car dynamics , like controller and observer
            normal_car_update(self,instant_index , weights_Dis);
            
            %% Send to center communication
            self.center_communication.update_local_state(self.vehicle_number, self.observer.est_local_state_current);
            self.center_communication.update_global_state(self.vehicle_number, self.observer.est_global_state_current);
            self.center_communication.update_trust(self.vehicle_number, self.trust_log(1, instant_index, :));
            
        end
        
        function normal_car_update(self,instant_index , weights)
            
            if self.typeController ~= "None"
                % disp('Controller is not empty');
                self.get_lane_id(self.state);
                
                %% Observer
                self.observer.Local_observer(self.state);
                self.observer.Distributed_Observer(instant_index , weights);
                %% Controller
                [~, u, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag,0);
                
                
                %% Update new state
                % calculate the optimal input of the vehicle
                dx = self.Bicycle(self.state(1:3), [self.state(4) + self.dt * u(1); u(2)]); % calculate dX through nonlinear model
                self.state = self.state + self.dt .* [dx; u(1)]; % update the state of the vehicle
                self.input = u; % update the input
                
            else
                %% Observer
                self.observer.Local_observer(self.state);
                self.observer.Distributed_Observer(instant_index, weights);
                % disp('Controller is empty');
                self.get_lane_id(self.state);
                speed = self.state(4);
                acceleration = self.input(1);
                
                self.state(4) = self.state(4) + acceleration * self.dt; % new speed
                % speed limit according to different scenarios
                [ulim, llim] = self.scenarios_config.getLimitSpeed(); 

                if self.state(4) >= ulim
                    self.state(4) = ulim;
                elseif self.state(4) <= llim
                    self.state(4) = llim;
                end
                dx = speed * self.dt + 0.5 * acceleration * self.dt^2; %dx=v*dt+0.5*a*dt^2
                self.state = [self.state(1) + dx; self.state(2); self.state(3); self.state(4)]; % new state of normal cars
                %  no need update input , beacuse the input is constant
                % self.input = [0;0]; % update the input
                
                e = 0;
            end
            %% Save the state and input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
            
            
        end
        
        function ego_vehicle_update(self)
            self.get_lane_id(self.state);
            [self.acc_flag, u, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag, self.acc_flag);
            % calculate the optimal input of the vehicle
            % ----------------------------------self.state(4) + self.dt * u(1); convert acc to speed,  v = v + a*dt
            dx = self.Bicycle(self.state(1:3), [self.state(4) + self.dt * u(1); u(2)]); % calculate dX through nonlinear model
            self.state = self.state + self.dt .* [dx; u(1)]; % update the state of the vehicle
            self.input = u; % update the input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
        end
        
        function lane_change_car_update(self)
            self.get_lane_id(self.state);
            [n, u, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag , 0);
            
            % calculate the optimal input of the vehicle
            dx = self.Bicycle(self.state(1:3), [self.state(4) + self.dt * u(1); u(2)]); % calculate dX through nonlinear model
            self.state = self.state + self.dt .* [dx; u(1)]; % update the state of the vehicle
            self.input = u; % update the input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
        end
        
        function dX = Bicycle(self, state, input)
            
            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            [x, y, phi] = self.unpack_state(state);
            [v, beta] = self.unpack_input(input); % input is V beacause its aldready convert in calling fucntion
            delta_f = atan(l*tan(beta)/l_r); % calculate front steering angle from slip angle
            xdot = v * cos(phi+beta); % velocity in x direction
            ydot = v * sin(phi+beta); % velocity in y direction
            phidot = v * sin(beta) / l_r; % yaw rate
            dX = [xdot; ydot; phidot];
        end
        
        function Bicycle_second_v_update(self, state)
            % Unpack parameters
            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            
            % Unpack state
            [x, y, phi, v] = self.unpack_state(state);
            a = state(5); % Acceleration
            
            % Unpack inputs
            [non, input, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag);
            
            u_a = input(1); % input Jerk (rate of change acceleration)
            beta = input(2); % Slip angle
            
            % Calculate front steering angle
            delta_f = atan(l * tan(beta) / l_r);
            
            % Extended model dynamics
            xdot = v * cos(phi + beta); % Velocity in x direction
            ydot = v * sin(phi + beta); % Velocity in y direction
            phidot = v * sin(beta) / l_r; % Yaw rate
            vdot = a; % Acceleration
            adot = 1/self.param.tau * u_a - 1/self.param.tau*a; % Jerk
            % Return state derivative
            dX = [xdot; ydot; phidot; vdot; adot];
            
            self.state = self.state + self.dt .* dX; % update the state of the vehicle
            
            self.input = u; % update the input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
        end
        
        function [] = get_lane_id(self, state)
            if state(2) <= self.lanes.lane_width - 0.5 * self.param.width
                self.lane_id = 1;
            else if state(2) <= self.lanes.lane_width + 0.5 * self.param.width
                    self.lane_id = 1.5;
                else if state(2) <= 2 * self.lanes.lane_width - 0.5 * self.param.width
                        self.lane_id = 2;
                    else if state(2) <= 2 * self.lanes.lane_width + 0.5 * self.param.width
                            self.lane_id = 2.5;
                        else if state(2) <= 3 * self.lanes.lane_width - 0.5 * self.param.width
                                self.lane_id = 3;
                            else if state(2) <= 3 * self.lanes.lane_width + 0.5 * self.param.width
                                    self.lane_id = 3.5;
                                else if state(2) <= 4 * self.lanes.lane_width - 0.5 * self.param.width
                                        self.lane_id = 4;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        function [speed, beta] = unpack_input(self, input)
            speed = input(1);
            beta = input(2);
        end
        function [x, y, phi] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
        end
        
        function [x, y, phi,v] = unpack_state_v2(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            v = state(4);
        end
        


    
        function calculateTrustAndOpinion(self, instant_index, connected_vehicles)
        
            %% Step 1: Calculate trust for direct vehicles
            received_trusts = []; % Store received trust scores of all direct vehicles
            received_vehicles = []; % Store vehicle indices providing trust scores
        
            for vehicle_direct = connected_vehicles
                if abs(vehicle_direct - self.vehicle_number) == 1
                    %% Calculate trust for direct vehicle
                    [self.trust_log(1, instant_index, vehicle_direct), ~, ~, ~, ~] = ...
                        self.trip_models{vehicle_direct}.calculateTrust(...
                        self, self.other_vehicles(vehicle_direct), self.other_vehicles(1), self.dt, self.dt);
                end
        
                %% Store received trust values for opinion calculation
                trust_score = self.center_communication.get_trust_score(vehicle_direct);
                if ~isempty(trust_score)
                    received_trusts = [received_trusts; trust_score];
                    received_vehicles = [received_vehicles; vehicle_direct];
                end
            end
        
            %% Step 2: Calculate opinion-based trust for non-direct vehicles
            if ~isempty(received_trusts)
                for vehicle_id = 1:self.num_vehicles
                    if vehicle_id == self.vehicle_number
                        self.trust_log(1, instant_index, vehicle_id) = 1; % Ego vehicle trust is 1
                        continue;
                    end
        
                    %% If trust score is zero, compute opinion-based trust
                    if self.trust_log(1, instant_index, vehicle_id) == 0
                        received_trusts_vehicle = received_trusts(:,:,vehicle_id); % Trust scores for vehicle_id
                        non_zero_trusts = received_trusts_vehicle(received_trusts_vehicle ~= 0); % Filter zeros
                        non_zero_and_one_trusts = non_zero_trusts(non_zero_trusts ~= 1); % Filter ones
        
                        new_received_vehicles = received_vehicles(received_trusts_vehicle ~= 0); % Filter zeros
                        new_received_vehicles = new_received_vehicles(non_zero_trusts ~= 1); % Filter ones
        
                        if ~isempty(non_zero_and_one_trusts)
                            %% Opinion-based trust aggregation (Design 1)
                            opinion_weights = 1 ./ (1 + abs(new_received_vehicles - vehicle_id)); % Weight by distance
                            normalized_weights = opinion_weights / sum(opinion_weights); % Normalize weights
                            opinion_score = sum(normalized_weights .* non_zero_and_one_trusts, 1); % Weighted sum
                        else
                            opinion_score = 0; % Default to zero if no valid trust scores
                        end
        
                        self.trust_log(1, instant_index, vehicle_id) = opinion_score;
                    end
                end
            end
        end


        %% ----- Very important function ----- 
        %% Assign the other vehicles to the ego vehicle , and many other things 
        function assign_neighbor_vehicle(self, other_vehicles, center_communication, graph)
            self.graph = graph;
            disp('Assigning other vehicles');
            self.other_vehicles = other_vehicles;
            
            state_initial = self.state;
            % local state of the ego vehicle
            inital_local_state = state_initial;
            
            
            %-------- To get global state
            inital_global_state = state_initial;
            for i = 1:length(other_vehicles) - 1
                inital_global_state = [inital_global_state , state_initial];
            end
            %--------
            %% Create an observer for the vehicle
            self.observer = Observer(self, self.param ,inital_global_state, inital_local_state);
            
            %% Create controller
            
            if self.typeController == "None"
                self.controller = [];
            else
                param_opt = ParamOptEgo(self.dt);
                self.controller = Controller(self,[], "IDM", param_opt, self.param, self.lanes); % Create a controller for the vehicle
            end
            
            
            
            self.num_vehicles = length(self.other_vehicles);
            %% Create trust models for the vehicle
            self.trip_models = arrayfun(@(x) TriPTrustModel(), 1:self.num_vehicles, 'UniformOutput', false);
            self.trust_log = zeros(1, self.total_time_step, self.num_vehicles);
            
            
            %% Update inital value for center_communcation
            self.center_communication = center_communication; % Initialize the CenterCommunication reference
            self.center_communication.register_vehicle(self); % Register the vehicle with the central communication hub
            
        end
        
        
        %% Function for plot 
        function plot_ground_truth_vs_estimated(self, est_global_log)
            figure;
            subplot(3,1,1);
            plot( self.state_log(1,:), 'b', 'LineWidth', 1.5); hold on;
            plot( self.observer.est_global_log(1,:), 'r--', 'LineWidth', 1.5);
            legend('Ground Truth X', 'Estimated X');
            title('X Position Comparison');
            
            subplot(3,1,2);
            plot( self.state_log(2,:), 'b', 'LineWidth', 1.5); hold on;
            plot( est_global_log(2,:), 'r--', 'LineWidth', 1.5);
            legend('Ground Truth Y', 'Estimated Y');
            title('Y Position Comparison');
            
            subplot(3,1,3);
            plot( self.state_log(4,:), 'b', 'LineWidth', 1.5); hold on;
            plot( est_global_log(4,:), 'r--', 'LineWidth', 1.5);
            legend('Ground Truth Speed', 'Estimated Speed');
            title('Speed Comparison');
            xlabel('Time (s)');
        end
        
        function plot_trust_log(self)
            % PLOT_TRUST_LOG Plots the trust values over time for each vehicle.
            %
            % Inputs:
            %   trust_log: 3D array of trust values (1 x time_steps x num_vehicles)
            %   num_vehicles: Number of vehicles (integer)
            
            % Get the number of time steps
            time_steps = size(self.trust_log, 2);
            
            % Create a time vector for the x-axis
            time_vector = 1:time_steps;
            
            % Create a new figure
            figure;
            hold on;
            
            % Plot the trust values for each vehicle
            for vehicle_idx = 1:self.num_vehicles
                plot(time_vector, squeeze(self.trust_log(1, :, vehicle_idx)), 'DisplayName', ['Vehicle ' num2str(vehicle_idx)]);
            end
            
            % Add labels and legend
            xlabel('Time Step');
            ylabel(['Trust Value of' num2str(self.vehicle_number)]);
            title(['Trust Values Over Time of' num2str(self.vehicle_number)]);
            legend show;
            grid on;
            
            hold off;
        end
        
        
        function h =  plot_vehicle(self)
            L = self.param.l_fc + self.param.l_rc;
            H = self.param.width;
            theta = self.state(3);
            center1 = self.state(1);
            center2 = self.state(2);
            % Rotation matrix based on the vehicle's orientation
            R = ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
            % Define the corners of the vehicle in its local frame
            X = ([-L / 2, L / 2, L / 2, -L / 2]);
            Y = ([-H / 2, -H / 2, H / 2, H / 2]);
            
            % Rotate the corners to the global frame
            for i = 1:4
                T(:, i) = R * [X(i); Y(i)];
            end
            % Calculate the coordinates of the vehicle's corners in the global frame
            
            x_lower_left = center1 + T(1, 1);
            x_lower_right = center1 + T(1, 2);
            x_upper_right = center1 + T(1, 3);
            x_upper_left = center1 + T(1, 4);
            y_lower_left = center2 + T(2, 1);
            y_lower_right = center2 + T(2, 2);
            y_upper_right = center2 + T(2, 3);
            y_upper_left = center2 + T(2, 4);
            
            % Combine the coordinates into arrays for plotting
            x_coor = [x_lower_left, x_lower_right, x_upper_right, x_upper_left];
            y_coor = [y_lower_left, y_lower_right, y_upper_right, y_upper_left];
            
            if (self.vehicle_number == 1)
                % color = [0, 1, 0]; % green
                color = [1, 0, 0]; % red
            else
                color = [0, 0, 1]; % blue
                % color = [1, 0, 1]; % magenta
            end
            
            h = patch('Vertices', [x_coor; y_coor]', 'Faces', [1, 2, 3, 4], 'Edgecolor', color, 'Facecolor', color, 'Linewidth', 1.5);
            axis equal;
        end
        
        
        
        
    end
end