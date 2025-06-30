classdef CACC < handle
    properties
        param_opt;
        param_sys;
        goal;
        straightlane;
        vehicle_number;
        controller;
    end
    methods
        function self = CACC( controller)
            self.controller = controller;
            self.param_opt = controller.param_opt;
            self.param_sys = controller.param_sys;
            self.goal = controller.goal;
            self.straightlane = controller.straightlane;
        end
        function [acc_flag,input, e] = get_optimal_input(self,host_car_id, state, last_input, lane_id, input_log, inital_land_ID, direction_flag, type_state,acc_flag)
            acc_flag = 0;
            % Extract vehicle parameters
            alpha = self.param_opt.alpha; % Maximum acceleration
            beta = self.param_opt.beta;   % Comfortable deceleration
            v0 = self.param_opt.v0;       % Desired velocity
            delta = self.param_opt.delta; % Acceleration exponent
            T = self.param_opt.T;         % Safe time headway
            s0 = self.param_opt.ri;       % Minimum gap distance
            K = self.param_opt.K;         % CACC control gain
            h = self.param_opt.hi;         % Time-gap parameter

            % Extract current state
            %% Depending on the type of state, we can use the true state or estimated state
            %% So if use estimated state, 
            %% In case of attack ,the function of 'get_surrounding_vehicles' not working properly (already fixed , below)
            [x, y, theta, v,a] = self.unpack_state(state);

            % Get surrounding vehicles: preceding AND following
            % : why use self.controller.vehicle.state(1) , Because we want to get the true host car position (not the estimated one)
            [~, preceding_vehicles, ~, following_vehicles] = self.controller.get_surrounding_vehicles(self.controller.vehicle.state(1), lane_id, direction_flag, host_car_id);

            num_preceding = length(preceding_vehicles);
            num_following = length(following_vehicles);

            % num_vehicles = length(surrounding_vehicles);

            if num_preceding == 0
                % No preceding vehicles, use IDM model
                disp("CACC No car preceding vehicle "+ num2str(host_car_id))

                s = 200; % Assume large gap
                delta_v = 0;
                % acc = alpha * (1 - (v / v0)^delta); % IDM acceleration
                acc = 0;
            else
                %% NEW : Consensus-based CACC with surrounding vehicles
                u_coop = 0;
                count = 0;

                % Process preceding vehicles
                for j = 1:num_preceding
                    car_j = preceding_vehicles(j);
                    if (type_state == "true")
                        x_j = car_j.state(1);
                        v_j = car_j.state(4);
                        a_j = car_j.state(5);
                    else
                        x_j = self.controller.vehicle.observer.est_global_state_current(1,car_j.vehicle_number);
                        v_j = self.controller.vehicle.observer.est_global_state_current(4,car_j.vehicle_number);
                        a_j = self.controller.vehicle.observer.est_global_state_current(5,car_j.vehicle_number);
                    end

                    spacing_error = x_j - x - ((host_car_id - car_j.vehicle_number) * s0 + h * v);
                    velocity_error = v_j - v;
                    accel_error = a_j - a;

                    if self.controller.vehicle.scenarios_config.control_use_accel
                        u_coop = u_coop + K * [spacing_error ; velocity_error ; accel_error];
                    else
                        u_coop = u_coop + K(1:2) * [spacing_error ; velocity_error];
                    end
                    count = count + 1;
                end

                if self.controller.vehicle.scenarios_config.CACC_bidirectional
                    % Process following vehicles
                    for j = 1:num_following
                        car_j = following_vehicles(j);
                        if (type_state == "true")
                            x_j = car_j.state(1);
                            v_j = car_j.state(4);
                            a_j = car_j.state(5);
                        else
                            x_j = self.controller.vehicle.observer.est_global_state_current(1,car_j.vehicle_number);
                            v_j = self.controller.vehicle.observer.est_global_state_current(4,car_j.vehicle_number);
                            a_j = self.controller.vehicle.observer.est_global_state_current(5,car_j.vehicle_number);
                        end
    
                        spacing_error = x - x_j - ((car_j.vehicle_number - host_car_id) * s0 + h * v_j); % reverse gap
                        velocity_error = v - v_j;  % reverse sign
                        accel_error = a - a_j;
    
                        if self.controller.vehicle.scenarios_config.control_use_accel
                            u_coop = u_coop + K * [spacing_error ; velocity_error ; accel_error];
                        else
                            u_coop = u_coop + K(1:2) * [spacing_error ; velocity_error];
                        end
                        count = count + 1;
                    end
                end

                % Normalize
                if count > 0
                    u_coop = u_coop / count;
                end

                acc = u_coop;

            end

            % Assuming a simple steering model (constant for now)
            delta = 0;

            input = [acc; delta];
            e = 0; % Placeholder for errors



        end


        function [acc, beta] = unpack_input(self, input)
            acc = input(1);
            beta = input(2);
        end
        function [x, y, psi, v,a] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            psi = state(3);
            v = state(4);
            a = state(5);
        end
    end
end