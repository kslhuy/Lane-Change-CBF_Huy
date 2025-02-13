classdef CACC
    properties
        param_sys; % parameters of vehicles
        goal; % control objective
        straightlane; % driving lanes
        other_vehicles; % surrounding vehicles
        vehicle_number;

    end
    methods
        function self = CACC( veh_param, controller_goal, straightlane, other_vehicles)
            self.param_sys = veh_param;
            self.goal = controller_goal;
            self.straightlane = straightlane;
            self.other_vehicles = other_vehicles;
        end
        function [acc_flag, input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, current_lane_id, direction_flag, acc_flag)

            %% load parameter and control objective
            safety_factor = self.goal.safety_factor;
            lim_speed = self.goal.lim_speed;
            l_rc = self.param_sys.l_rc;
            l_fc = self.param_sys.l_fc;
            l_f = self.param_sys.l_f;
            l_r = self.param_sys.l_r;
            width = self.param_sys.width;
            dt = self.param_sys.dt;
            [x, y, psi, v] = self.unpack_state(state);
            [acc, beta] = self.unpack_input(last_input);

            target_y = self.goal.target_y;

            %{
            This script is responsible for loading and sorting surrounding vehicles based on their lane information and position relative to the ego vehicle. It identifies the closest leading vehicle in the current lane, the closest vehicle behind the ego vehicle in the target lane, and the closest leading vehicle in the target lane. The script also determines the target speed of the ego vehicle based on whether it is accelerating or not.

            Variables:
            - current_lane_vehicles: Array to store vehicles in the current lane.
            - target_lane_vehicles: Array to store vehicles in the target lane.
            - num_veh: Number of surrounding vehicles.
            - car_fc: Closest leading vehicle in the current lane.
            - carfc_range: Range within which a vehicle is considered as the closest leading vehicle in the current lane.
            - car_bt: Closest vehicle in the target lane that is behind the ego vehicle.
            - car_bt_range: Range within which a vehicle is considered as the closest vehicle behind the ego vehicle in the target lane.
            - car_ft: Closest leading vehicle in the target lane.
            - carft_range: Range within which a vehicle is considered as the closest leading vehicle in the target lane.
            - acc_flag: Flag indicating if the ego vehicle is accelerating.
            - target_speed: Target speed of the ego vehicle.

            Steps:
            1. Initialize arrays for current and target lane vehicles.
            2. Sort surrounding vehicles based on lane ID and position relative to the ego vehicle.
            3. Identify the closest leading vehicle in the current lane.
            4. Identify the closest vehicle behind the ego vehicle and the closest leading vehicle in the target lane.
            5. Determine the target speed of the ego vehicle based on the acceleration flag.
            %}

            %% load surrounding vehicles
            current_lane_vehicles = [];
            target_lane_vehicles = [];
            num_veh = length(self.other_vehicles);
            % sort surrounding vehilces according to the lane id information
            for i = 1:num_veh
                if (self.other_vehicles(i).lane_id == current_lane_id | self.other_vehicles(i).lane_id == current_lane_id - direction_flag * 0.5)
                    % the vehicle is in the current lane
                    if self.other_vehicles(i).state(1) >= x
                        % collect the vehicle in the current lane before the
                        % ego vehicle
                        current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
                    end
                elseif self.other_vehicles(i).lane_id == current_lane_id + direction_flag * 0.5
                    % the vehicle is accross the dividing line
                    if self.other_vehicles(i).state(1) >= x
                        current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
                    end
                    % Collect the vehicle in the target lane
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                
                % Check if the vehicle is in the target lane
                elseif (self.other_vehicles(i).lane_id == current_lane_id + direction_flag)
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                % Check if the vehicle is in the target lane offset by 1.5 times the direction flag
                elseif self.other_vehicles(i).lane_id == current_lane_id + 1.5 * direction_flag
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                end
            end
            car_fc = []; % car_fc is the closest leading vehicle in the current lane (vehicle fc)
            carfc_range = x + 100; % this value can be tuned, it shows in which range the vehicle would be considered
            for j = 1:length(current_lane_vehicles)
                if current_lane_vehicles(j).state(1) <= carfc_range
                    car_fc = current_lane_vehicles(j);
                    carfc_range = current_lane_vehicles(j).state(1);
                end
            end
            car_bt = []; % car_bt is the closeset vehicle in the target lane that is behind ego vehicle (vehicle bt)
            car_bt_range = x - 100;
            car_ft = []; % car_ft is the closet leading vehicle in the target lane (vehicle ft)
            carft_range = x + 100;
            for i = 1:length(target_lane_vehicles)
                if target_lane_vehicles(i).state(1) <= x & target_lane_vehicles(i).state(1) >= car_bt_range
                    car_bt = target_lane_vehicles(i);
                    car_bt_range = target_lane_vehicles(i).state(1);
                end
                if target_lane_vehicles(i).state(1) >= x & target_lane_vehicles(i).state(1) <= carft_range
                    car_ft = target_lane_vehicles(i);
                    carft_range = target_lane_vehicles(i).state(1);
                end
            end
            if lane_id == current_lane_id + direction_flag
                acc_flag = 0; % indicates if ego vehicle is accelerating
            end
            if acc_flag == 0
                target_speed = self.goal.target_speed;
            else
                target_speed = lim_speed;
            end

            %% 

            %% lateral position CLF
            h_y = y - target_y;
            V_y = h_y^2;
            phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y;
            phi1_y = [0, 2 * h_y * v * cos(psi)];

            %% velocity CLF
            h_v = v - target_speed;
            V_v = h_v^2;
            phi0_v = alpha_v * V_v;
            phi1_v = [2 * h_v * 1, 0];

            %% yaw angle CLF
            h_yaw = psi;
            V_yaw = h_yaw^2;
            phi0_yaw = alpha_yaw * V_yaw;
            phi1_yaw = [0, 2 * h_yaw * v * l_r];

            
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