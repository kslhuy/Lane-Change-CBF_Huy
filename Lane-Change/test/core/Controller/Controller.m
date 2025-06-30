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
        connected_vehicles_idx;
    end
    methods
        function self = Controller(vehicle,controller_goal, controller_type, method_param, veh_param, straightlane,connected_vehicles_idx)
            self.vehicle = vehicle;
            self.goal = controller_goal;% Set the goal state
            self.type = controller_type;
            self.param_sys = veh_param; % Set the vehicle system parameters
            self.straightlane = straightlane; % Set the straight lane information
            self.connected_vehicles_idx = connected_vehicles_idx;

            if self.type == "CBF_CLF_QP"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters

                % Use CBF_CLF_QP controller for the ego-vehicle
                self.logic_ctrl = CLF_CBF_QP(self);

            elseif self.type == "CLF_QP"
                self.param_opt = ParamOptSurroundingVeh(self.vehicle.dt); % Set the optimization parameters
                % Use CLF_QP controller for the surrounding vehicle
                self.logic_ctrl = CLF_QP(self);

            elseif self.type == "CACC"
                self.param_opt = ParamOptEgo(self.vehicle.dt); % Set the optimization parameters
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
        function [acc_flag, optimal_input, e] = get_optimal_input(self, host_car_id, state, last_input, lane_id, input_log, initial_lane_id, direction_flag,type_state, acc_flag)
            if isempty(self.logic_ctrl)
                % Use default controller
                acc_flag = 0;
                optimal_input = last_input;
                e = 0;
                return;
            end

            [acc_flag, optimal_input, e] = self.logic_ctrl.get_optimal_input( host_car_id ,state, last_input, lane_id, input_log, initial_lane_id, direction_flag,type_state, acc_flag);

        end


        function [car_fc, car_fss, car_bt, car_ft, car_bss] = get_surrounding_vehicles(self, x, current_lane_id, direction_flag, host_vehicle_id)

            % Initialisation
            car_fc = [];    % First vehicle ahead in current lane
            car_fss = [];   % All vehicles ahead (any lane)
            car_bss = [];   % All vehicles behind (any lane)
            car_bt = [];    % Closest vehicle behind in target lane
            car_ft = [];    % Closest vehicle ahead in target lane

            current_lane_vehicles_ahead = [];
            target_lane_vehicles = [];

            % Paramètre de portée
            range_check = 100;
            carfc_range = x + range_check;
            carbt_range = x - range_check;
            carft_range = x + range_check;

            % Parcourir tous les véhicules
            for i = 1:length(self.vehicle.other_vehicles)
                veh = self.vehicle.other_vehicles(i);

                % Ignorer le véhicule hôte
                if veh.vehicle_number == host_vehicle_id
                    continue;
                end

                % Récupérer position et voie
                veh_x = veh.state(1);
                veh_lane = veh.lane_id;

                % Véhicules devant (toutes voies)
                if veh_x >= x
                    car_fss = [car_fss, veh];
                end

                % Véhicules derrière (toutes voies)
                if veh_x < x
                    car_bss = [car_bss, veh];
                end

                % Identifier si le véhicule est dans la même voie ou une voie cible
                in_current_lane = (veh_lane == current_lane_id || ...
                    veh_lane == current_lane_id - 0.5 * direction_flag);

                in_target_lane = (veh_lane == current_lane_id + direction_flag || ...
                    veh_lane == current_lane_id + 0.5 * direction_flag || ...
                    veh_lane == current_lane_id + 1.5 * direction_flag);

                % Traitement des véhicules dans la même voie
                if in_current_lane
                    if veh_x >= x && veh_x < carfc_range
                        car_fc = veh; % Le plus proche devant
                        carfc_range = veh_x;
                    end
                end

                % Traitement des véhicules dans la voie cible
                if in_target_lane
                    target_lane_vehicles = [target_lane_vehicles, veh];

                    if veh_x >= x && veh_x < carft_range
                        car_ft = veh;
                        carft_range = veh_x;
                    elseif veh_x < x && veh_x > carbt_range
                        car_bt = veh;
                        carbt_range = veh_x;
                    end
                end
            end
        end

    end
end
