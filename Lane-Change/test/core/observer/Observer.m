classdef Observer
    properties
        type; % type of the controller, value 1 stands for CBF_CLF_QP controller of the ego-vehicle;
                                        % value 2 stands for CLF_QP controller of surrounding vehicle
        param_sys; % parameter for vehicle
        straightlane;
        other_vehicles; % surrounding vehicle
        vehicle_number;

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


        function x_hat_i_j = Distributed_Observer(i ,  j , A , B , x_bar_j , x_hat_i_j, u_j , W )
        
            Sig = [0 ; 0 ;0]; % Initialize consensus term
        
            % Calculate the consensus term
            % Start from 2 , because the first element is the local state of the vehicle
            for l = 2:length(W)
                % l-1 because start from 2 , but we need to start from 1 for vehicle 
                Sig = W(2)*( x_hat_i_j(1:3 , l-1) - x_hat_i_j(1:3 , l-1) ) + Sig;
            end
        
            % Only if we try to estimated j = i , so we have real local state that was estimated by Local observer .
            % But if we try to estimate j != i , we have (fake) the local state of j , that by vehicle j send to i vehicle
        
            % Estimate the j vehicle , in i vehicle 
            % So the observer is implement in i vehicle
            w_i0 = W(1); % Weight of the local state
            
            %% Original paper (TRUE WORK)
            x_hat_i_j = A*( x_hat_i_j(1:3 , j) + Sig + w_i0 * (x_bar_j - x_hat_i_j(1:3 , j)) ) + B*u_j ; 
        
        end

        function x_bar = Local_observer(A , B , x_bar , u)
            x_bar = A*x_bar + B*u;
        end
        
    end
end
