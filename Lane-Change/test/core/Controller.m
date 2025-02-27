classdef Controller < handle
    properties
        goal;% The goal state or position for the controller
        type; % type of the controller, value 1 stands for CBF_CLF_QP controller of the ego-vehicle;
                                        % value 2 stands for CLF_QP controller of surrounding vehicle
        param_opt; % parameters for optimization problem
        param_sys; % parameter for vehicle
        straightlane;
        vehicle;
        logic_ctrl;
    end
    methods
        function self = Controller(vehicle,controller_goal, controller_type, method_param, veh_param, straightlane)
            self.vehicle = vehicle;
            self.goal = controller_goal;% Set the goal state
            self.type = controller_type;
            self.param_sys = veh_param; % Set the vehicle system parameters
            self.straightlane = straightlane; % Set the straight lane information

            if self.type == "CBF_CLF_QP"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters

                % Use CBF_CLF_QP controller for the ego-vehicle
                self.logic_ctrl = CLF_CBF_QP(self);
            
            elseif self.type == "CLF_QP"
                self.param_opt = ParamOptSurroundingVeh(self.vehicle.dt); % Set the optimization parameters
                % Use CLF_QP controller for the surrounding vehicle
                self.logic_ctrl = CLF_QP(self);
            
            elseif self.type == "CACC"
                % Use CACC controller for the surrounding vehicle
                self.logic_ctrl = CACC(self);
            
            elseif self.type == "IDM"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters
                % Use IDM controller for the surrounding vehicle
                self.logic_ctrl = IDM_control(self);
            
            elseif self.type == "CIDM"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters

                % Use CIDM controller for the surrounding vehicle
                self.logic_ctrl = CIDM_control(self);
              
            elseif self.type == "MPC"
                % Use MPC controller for the surrounding vehicle
                self.logic_ctrl = MPC(self);
            
            elseif self.type == "look_ahead"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters
                self.logic_ctrl = look_ahead_Control(self);
            
            else
                % Dont have a controller
                % self.logic_ctrl = null;
                disp("No controller is assigned to the vehicle");
                return;
            end
            self.logic_ctrl.vehicle_number = self.vehicle.vehicle_number;


        end

        % Method to get the optimal input for the controller based on the current state and other parameters
        function [acc_flag, optimal_input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, initial_lane_id, direction_flag, acc_flag)
            if isempty(self.logic_ctrl)
                % Use default controller
                acc_flag = 0;
                optimal_input = last_input;
                e = 0;
                return;            
            end

            [acc_flag, optimal_input, e] = self.logic_ctrl.get_optimal_input(state, last_input, lane_id, input_log, initial_lane_id, direction_flag, acc_flag);
            
        end

        % Method to get the surrounding vehicles
        function [car_fc, car_bt, car_ft] = get_surrounding_vehicles(self , x, current_lane_id, direction_flag)
            current_lane_vehicles = [];
            target_lane_vehicles = [];
            num_veh = length(self.vehicle.other_vehicles);
            
            % Get the host vehicle's ID or unique identifier
            host_vehicle_id = self.vehicle.vehicle_number; % Assuming vehicle_number is the unique identifier
            
            % sort surrounding vehilces according to the lane id information
            for i = 1:num_veh

                % Skip the host vehicle
                if self.vehicle.other_vehicles(i).vehicle_number == host_vehicle_id
                    continue;
                end

                if (self.vehicle.other_vehicles(i).lane_id == current_lane_id | self.vehicle.other_vehicles(i).lane_id == current_lane_id - direction_flag * 0.5)
                    % the vehicle is in the current lane
                    if self.vehicle.other_vehicles(i).state(1) >= x
                        % collect the vehicle in the current lane before the
                        % ego vehicle
                        current_lane_vehicles = [current_lane_vehicles, self.vehicle.other_vehicles(i)];
                    end
                elseif self.vehicle.other_vehicles(i).lane_id == current_lane_id + direction_flag * 0.5
                    % the vehicle is accross the dividing line
                    if self.vehicle.other_vehicles(i).state(1) >= x
                        current_lane_vehicles = [current_lane_vehicles, self.vehicle.other_vehicles(i)];
                    end
                    % Collect the vehicle in the target lane
                    target_lane_vehicles = [target_lane_vehicles, self.vehicle.other_vehicles(i)];
                
                % Check if the vehicle is in the target lane
                elseif (self.vehicle.other_vehicles(i).lane_id == current_lane_id + direction_flag)
                    target_lane_vehicles = [target_lane_vehicles, self.vehicle.other_vehicles(i)];
                % Check if the vehicle is in the target lane offset by 1.5 times the direction flag
                elseif self.vehicle.other_vehicles(i).lane_id == current_lane_id + 1.5 * direction_flag
                    target_lane_vehicles = [target_lane_vehicles, self.vehicle.other_vehicles(i)];
                end
            end
            
            range_check = 100;

            car_fc = []; % car_fc is the closest leading vehicle in the current lane (vehicle fc)
            carfc_range = x + range_check;
            for j = 1:length(current_lane_vehicles)
                if current_lane_vehicles(j).state(1) <= carfc_range
                    car_fc = current_lane_vehicles(j);
                    carfc_range = current_lane_vehicles(j).state(1);
                end
            end

            car_bt = [];% car_bt is the closeset vehicle in the target lane that is behind ego vehicle (vehicle bt)
            car_bt_range = x - range_check;
            car_ft = [];% car_ft is the closet leading vehicle in the target lane (vehicle ft)
            carft_range = x + range_check;
            
            for i = 1:length(target_lane_vehicles)
                if target_lane_vehicles(i).state(1) <= x && target_lane_vehicles(i).state(1) >= car_bt_range
                    car_bt = target_lane_vehicles(i);
                    car_bt_range = target_lane_vehicles(i).state(1);
                end
                if target_lane_vehicles(i).state(1) >= x && target_lane_vehicles(i).state(1) <= carft_range
                    car_ft = target_lane_vehicles(i);
                    carft_range = target_lane_vehicles(i).state(1);
                end
            end
        end
    end
end
