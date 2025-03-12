classdef CACC
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
            [x, y, theta, v] = self.unpack_state(state);

            % Get leading vehicles
            [~, surrounding_vehicles, ~, ~] = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag,host_car_id);

            num_vehicles = length(surrounding_vehicles);

            if num_vehicles == 0
                % No preceding vehicles, use IDM model
                disp('No car ahead');
                disp(self.vehicle_number);
                s = 200; % Assume large gap
                delta_v = 0;
                acc = alpha * (1 - (v / v0)^delta); % IDM acceleration
            else
                % Compute cooperative control input using consensus-based CACC
                u_coop = 0;
                

                for j = 1:num_vehicles
                    car_j = surrounding_vehicles(j);
                    if (type_state == "true")
                        x_j = car_j.state(1);
                        v_j = car_j.state(4);
                    else %estimated                        
                        x_j = self.controller.vehicle.observer.est_global_state_current(1,car_j.vehicle_number);
                        v_j = self.controller.vehicle.observer.est_global_state_current(4,car_j.vehicle_number); 
                    end
                    
                    spacing_error = x_j - x - ((host_car_id - car_j.vehicle_number) * s0 + h * v);
                    velocity_error = v_j - v;

                    u_coop = u_coop + K * [spacing_error ; velocity_error];
                end

                u_coop = u_coop / num_vehicles; % Normalize by number of preceding vehicles
                acc = u_coop; % Cooperative acceleration
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
        function [x, y, psi, v] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            psi = state(3);
            v = state(4);
        end
    end
end