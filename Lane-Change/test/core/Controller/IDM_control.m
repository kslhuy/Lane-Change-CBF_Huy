classdef IDM_control < handle % Extended Look-Ahead Controller for Lane-Changing Vehicles
    properties
        controller;
        param_opt;
        param_sys;
        goal;
        straightlane;
        % other_vehicles;
        vehicle_number;
    end
    methods
        function self = IDM_control(controller)
            self.controller = controller;
            self.param_opt = controller.param_opt;
            self.param_sys = controller.param_sys;
            self.goal = controller.goal;
            self.straightlane = controller.straightlane;
            % self.other_vehicles = other_vehicles;
        end



        function [acc_flag,input, e] = get_optimal_input(self, host_vehicle_id , state, last_input, lane_id, input_log, inital_land_ID, direction_flag, type_state,acc_flag)
            % Normally host_vehicle_id = self.vehicle_number
            % But we make it like a paramter , so we can further extend ,
            % in the case we want to estimate the controller of car j
            % within car i , So in car i cal this function with
            % host_vehicle_id = j


            acc_flag = 0;


            % Extract vehicle parameters
            alpha = self.param_opt.alpha; % Maximum acceleration
            beta = self.param_opt.beta;   % Comfortable deceleration
            v0 = self.param_opt.v0;       % Desired velocity
            delta = self.param_opt.delta; % Acceleration exponent
            T = self.param_opt.T;         % Safe time headway
            s0 = self.param_opt.s0;       % Minimum gap distance

            % disp(length(self.other_vehicles))

            % Extract current state
            [x, y, theta, v] = self.unpack_state(state);

            % Get leading vehicle information
            % why use self.controller.vehicle.state(1) , Because we want to get the true host car position (not the estimated one)
            [car_fc,car_fss, ~, ~] = self.controller.get_surrounding_vehicles(self.controller.vehicle.state(1), lane_id, direction_flag , host_vehicle_id);

            if isempty(car_fc)
                % car_fss
                disp("IDM No car preceding vehicle "+ num2str(host_car_id))
                s = 200; % Assume large gap if no car ahead
                delta_v = 0;
            else
                if (type_state == "true")
                    s = car_fc.state(1) - x;
                    delta_v = v - car_fc.state(4);
                else %estimated
                    %since this we use that like local , so use mesurement instead of estimated
                    % mesurement mean true state
                    s = car_fc.state(1) - x;
                    delta_v = v - car_fc.state(4);
                    % if (self.controller.vehicle.noise_flag)
                    %     s = car_fc.state(1) - x + randn(1) ;
                    %     delta_v = v - car_fc.state(4) + randn(1)*0.1;
                    % end

                    % est_car_fc = self.controller.vehicle.observer.est_global_state_current(:,car_fc.vehicle_number) ;
                    % s = est_car_fc(1) - x;
                    % delta_v = v - est_car_fc(4);
                end
            end

            % Compute desired minimum gap s*(v, Δv)
            s_star = s0 + T * v + (v * delta_v) / (2 * sqrt(alpha * beta));

            % Compute IDM acceleration
            acc = alpha * (1 - (v / v0)^delta - (s_star / s)^2);

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
