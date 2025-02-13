classdef Controller
    properties
        goal;% The goal state or position for the controller
        type; % type of the controller, value 1 stands for CBF_CLF_QP controller of the ego-vehicle;
                                        % value 2 stands for CLF_QP controller of surrounding vehicle
        param_opt; % parameters for optimization problem
        param_sys; % parameter for vehicle
        straightlane;
        other_vehicles; % surrounding vehicle
        vehicle_number;
        logic_ctrl;
    end
    methods
        function self = Controller(controller_goal, controller_type, method_param, veh_param, straightlane, other_vehicles)
            self.goal = controller_goal;% Set the goal state
            self.type = controller_type;
            self.param_opt = method_param; % Set the optimization parameters
            self.param_sys = veh_param; % Set the vehicle system parameters
            self.straightlane = straightlane; % Set the straight lane information
            self.other_vehicles = other_vehicles;

            if self.type == "CBF_CLF_QP"
                % Use CBF_CLF_QP controller for the ego-vehicle
                self.logic_ctrl = CLF_CBF_QP(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            elseif self.type == "CLF_QP"
                % Use CLF_QP controller for the surrounding vehicle
                self.logic_ctrl = CLF_QP(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            elseif self.type == "CACC"
                % Use CACC controller for the surrounding vehicle
                self.logic_ctrl = CACC(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            elseif self.type == "IDM"
                % Use IDM controller for the surrounding vehicle
                self.logic_ctrl = IDM_control(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            elseif self.type == "CIDM"
                % Use IDM controller for the surrounding vehicle
                self.logic_ctrl = CIDM_control(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
              
            elseif self.type == "MPC"
                % Use MPC controller for the surrounding vehicle
                self.logic_ctrl = MPC(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            elseif self.type == "look_ahead"
                % Use MPC controller for the surrounding vehicle
                self.logic_ctrl = look_ahead_Control(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
            
            else
                % Dont have a controller
                self.logic_ctrl = null;
            end
            self.logic_ctrl.vehicle_number = self.vehicle_number;


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
    end
end
