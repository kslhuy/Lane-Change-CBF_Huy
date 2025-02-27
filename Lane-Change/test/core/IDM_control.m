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

            [car_fc, ~, ~] = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag);
            
            if isempty(car_fc)
                disp('No car ahead'  );
                disp(self.vehicle_number); 
                s = 200; % Assume large gap if no car ahead
                delta_v = 0;
            else
                s = car_fc.state(1) - x;
                delta_v = v - car_fc.state(4);
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
        
        % function [car_fc, car_bt, car_ft] = get_surrounding_vehicles(self, x, current_lane_id, direction_flag)
        %     current_lane_vehicles = [];
        %     target_lane_vehicles = [];
        %     num_veh = length(self.other_vehicles);
            
        %     % sort surrounding vehilces according to the lane id information
        %     for i = 1:num_veh
        %         if (self.other_vehicles(i).lane_id == current_lane_id | self.other_vehicles(i).lane_id == current_lane_id - direction_flag * 0.5)
        %             % the vehicle is in the current lane
        %             if self.other_vehicles(i).state(1) >= x
        %                 % collect the vehicle in the current lane before the
        %                 % ego vehicle
        %                 current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
        %             end
        %         elseif self.other_vehicles(i).lane_id == current_lane_id + direction_flag * 0.5
        %             % the vehicle is accross the dividing line
        %             if self.other_vehicles(i).state(1) >= x
        %                 current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
        %             end
        %             % Collect the vehicle in the target lane
        %             target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                
        %         % Check if the vehicle is in the target lane
        %         elseif (self.other_vehicles(i).lane_id == current_lane_id + direction_flag)
        %             target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
        %         % Check if the vehicle is in the target lane offset by 1.5 times the direction flag
        %         elseif self.other_vehicles(i).lane_id == current_lane_id + 1.5 * direction_flag
        %             target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
        %         end
        %     end
            

        %     car_fc = [];
        %     carfc_range = x + 100;
        %     for j = 1:length(current_lane_vehicles)
        %         if current_lane_vehicles(j).state(1) <= carfc_range
        %             car_fc = current_lane_vehicles(j);
        %             carfc_range = current_lane_vehicles(j).state(1);
        %         end
        %     end
        %     % if (self.vehicle_number == 2)
        %     %     disp("other vehicle : " + num_veh);
        %     %     disp("current_lane_vehicles : " + length(target_lane_vehicles));
        %     % 
        %     % end

        %     car_bt = [];
        %     car_bt_range = x - 100;
        %     car_ft = [];
        %     carft_range = x + 100;
        %     for i = 1:length(target_lane_vehicles)
        %         if target_lane_vehicles(i).state(1) <= x && target_lane_vehicles(i).state(1) >= car_bt_range
        %             car_bt = target_lane_vehicles(i);
        %             car_bt_range = target_lane_vehicles(i).state(1);
        %         end
        %         if target_lane_vehicles(i).state(1) >= x && target_lane_vehicles(i).state(1) <= carft_range
        %             car_ft = target_lane_vehicles(i);
        %             carft_range = target_lane_vehicles(i).state(1);
        %         end
        %     end
        % end


    end
end
