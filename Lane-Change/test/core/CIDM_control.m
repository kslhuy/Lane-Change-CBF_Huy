classdef CIDM_control % Extended Look-Ahead Controller for Lane-Changing Vehicles
    properties
        param_opt;
        param_sys;
        goal;
        straightlane;
        vehicle_number;
        controller;
        fixed_weights;

    end
    methods
        function self = CIDM_control(controller)
            self.controller = controller;
            self.param_opt = controller.param_opt;
            self.param_sys = controller.param_sys;
            self.goal = controller.goal;
            self.straightlane = controller.straightlane;

            % Define fixed weight distributions based on number of cooperative vehicles
            self.fixed_weights = {
                [1],                    % 1 vehicle
                [0.8, 0.2],             % 2 vehicles
                [0.8, 0.15, 0.05],        % 3 vehicles
                [0.8, 0.15, 0.04, 0.01], % 4 vehicles
                [0.8, 0.15, 0.04, 0.01, 0] % 5 vehicles
                };
        end

        function [acc_flag,input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, inital_land_ID, direction_flag, type_state,acc_flag)
            acc_flag = 0;

            % Extract vehicle parameters
            alpha = self.param_opt.alpha; % Maximum acceleration
            beta = self.param_opt.beta;   % Comfortable deceleration
            v0 = self.param_opt.v0;       % Desired velocity
            delta = self.param_opt.delta; % Acceleration exponent
            T = self.param_opt.T;         % Safe time headway
            s0 = self.param_opt.s0;       % Minimum gap distance

            % Extract current state
            [x, y, theta, v] = self.unpack_state(state);

            % Get surrounding vehicles
            [~,car_fss, ~, ~] = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag);

            if isempty(car_fss)
                disp('No car ahead'  );
                disp(self.vehicle_number);
                s_n = 200; % Assume large gap if no car ahead
                delta_v = 0;
            end




            num_vehicles = min(length(car_fss), 5); % Consider at most 5 vehicles
            weights = self.fixed_weights{num_vehicles}; % Get the correct weight distribution

            % Compute cooperative desired gap and total weighted gap
            s_star = s0 + v * T;
            weighted_sum_numerator = 0;
            weighted_sum_denominator = 0;
            total_weight = 0;


            for j = num_vehicles:-1:1
                if (type_state == "true")
                    s_j = car_fss(j).state(1) - x;
                    delta_v = v - car_fss(j).state(4);
                else %estimated
                    s_j = self.controller.vehicle.observer.est_global_state_current(1,car_fss(j).vehicle_number) - x;
                    delta_v = v - self.controller.vehicle.observer.est_global_state_current(4,car_fss(j).vehicle_number);
                end
                
                % w_nj = exp(-abs(s_j) / 50); % Exponential weight decay with distance
                if (j>1)
                    w_nj = weights(j-1);
                else
                    w_nj = weights(num_vehicles);
                end
                weighted_sum_numerator = weighted_sum_numerator + w_nj * (v* delta_v);
                weighted_sum_denominator = weighted_sum_denominator + w_nj * s_j;
                % total_weight = 1;
                total_weight = total_weight + w_nj;
            end

            if total_weight > 0
                s_star = s_star + (weighted_sum_numerator / total_weight)/(2 * sqrt(alpha * beta));
                s_n = weighted_sum_denominator / total_weight;
            else
                s_n = s0; % Default to minimum safe gap if no vehicles
            end

            % Compute acceleration using CIDM
            acc = alpha * (1 - (v/v0)^delta - (s_star / s_n)^2);

            % Assuming a simple steering model (constant for now)
            delta = 0;

            input = [acc; delta];
            e = 0; % Placeholder for errors
        end

        function [x, y, theta, v] = unpack_state(~, state)
            x = state(1);
            y = state(2);
            theta = state(3);
            v = state(4);
        end

        function kappa = compute_curvature(~, v, theta)
            % Compute curvature kappa = omega / v (Assume small angles for simplicity)
            kappa = theta / max(v, 0.1); % Avoid division by zero
        end

        function s_bar = compute_s_bar(~, kappa, ri, hi, v)
            % Compute magnitude of extension vector
            if kappa == 0
                s_bar = 0;
            else
                s_bar = (-1 + sqrt(1 + kappa^2 * (ri + hi * v)^2)) / kappa;
            end
        end




    end
end
