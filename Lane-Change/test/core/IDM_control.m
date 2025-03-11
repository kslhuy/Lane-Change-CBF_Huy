classdef IDM_control % Extended Look-Ahead Controller for Lane-Changing Vehicles
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

    
        
        function [acc_flag,input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, inital_land_ID, direction_flag, acc_flag)
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

            [~,car_fss, ~, ~] = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag);
            car_font = car_fss(end); 
            
            if isempty(car_font)
                disp('No car ahead'  );
                disp(self.vehicle_number); 
                s = 200; % Assume large gap if no car ahead
                delta_v = 0;
            else
                % est_car_fc = self.controller.vehicle.observer.est_global_state_current(:,car_font.vehicle_number) ;
                % s = est_car_fc(1) - x;
                % delta_v = v - est_car_fc(4);
                s = car_font.state(1) - x;
                delta_v = v - car_font.state(4);
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

        % function self = assign_otherveicle(self , other_vehicles)
        %     disp('Assigning other vehicles IDM controller');
        %     self.other_vehicles = other_vehicles;
        % end
        
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
