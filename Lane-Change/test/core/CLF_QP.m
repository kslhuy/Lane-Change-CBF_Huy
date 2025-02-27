classdef CLF_QP % an qp for surrounding vehicle that is changing the lane
    properties
        controller;
        param_opt;
        param_sys;
        goal;
        straightlane;
        vehicle_number;

    end
    methods
        function self = CLF_QP(controller)
 
            self.controller = controller;
            self.param_opt = controller.param_opt;
            self.param_sys = controller.param_sys;
            self.goal = controller.goal;
            self.straightlane = controller.straightlane;
        end
        % GET_OPTIMAL_INPUT Computes the optimal control input for lane change
        %
        %   [acc_flag, input, e] = GET_OPTIMAL_INPUT(self, state, last_input, lane_id, input_log, initial_lane_id, direction_flag)
        %   computes the optimal acceleration and steering angle for a vehicle to
        %   perform a lane change maneuver using Control Lyapunov Function (CLF) 
        %   and Quadratic Programming (QP).
        %
        %   Inputs:
        %       self - the object instance containing parameters and goals
        %       state - the current state of the vehicle [x; y; phi; v]
        %       last_input - the last control input [acc; beta]
        %       lane_id - the current lane ID
        %       input_log - log of previous inputs (not used in this function)
        %       initial_lane_id - the initial lane ID before the lane change
        %       direction_flag - flag indicating the direction of lane change (+1 for right, -1 for left)
        %
        %   Outputs:
        %       acc_flag - flag indicating if acceleration is within limits (0 if within limits)
        %       input - the optimal control input [acc_n; beta_n]
        %       e - error term (currently set to 0)
        %
        %   The function uses the vehicle's dynamics and a Linear Quadratic Regulator (LQR)
        %   to compute the optimal control input that minimizes the deviation from the
        %   target lane and speed. The constraints on the control inputs are enforced
        %   using Quadratic Programming (QP).
        function [acc_flag, input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, initial_lane_id, direction_flag , acc_flag)
            alpha = self.param_opt.alpha;
            target_y = self.goal.target_y;
            target_speed = self.goal.target_speed;
            l_r = self.param_sys.l_r;
            l_f = self.param_sys.l_f;
            width = self.param_sys.width;
            dt = self.param_opt.dt;
            [x, y, phi, v] = self.unpack_state(state);
            [acc, beta] = self.unpack_input(last_input);
            A = [0, 0, -v * sin(phi + beta), cos(phi + beta); ...
                0, 0, v * cos(phi + beta), sin(phi + beta); ...
                0, 0, 0, sin(beta) / l_r; ...
                0, 0, 0, 0];
            B = [0, -v * sin(phi + beta); ...
                0, v * cos(phi + beta); ...
                0, v * cos(beta) / l_r; ...
                1, 0];
            % if lane ID matches target lane
            if lane_id == initial_lane_id + direction_flag
                Q = diag([10^-15, 10^-1, 10^(10), 0.5]);
            else
                Q = diag([10^-15, 0.4, 10^(3), 1]);
            end
            R = diag([1, 1]);
            [K, P] = lqr(A, B, Q, R);
            x_lqr = state - [0; target_y; 0; target_speed];
            V = x_lqr' * P * x_lqr;
            phi0 = x_lqr' * (A' * P + P * A) * x_lqr + alpha * V;
            phi1 = [2 * x_lqr' * P * B];
            Aclf = [phi1, -1];
            bclf = [-phi0];
            H = self.param_opt.H;
            F = self.param_opt.F;
            % input constraint
            A_u = [1, 0, 0; ...
                -1, 0, 0; ...
                0, 1, 0; ...
                0, -1, 0];
            A_u0 = [0, 1, 0; ...
                0, -1, 0];
            b_u = [self.goal.lim_acc; self.goal.lim_acc; self.goal.lim_beta; self.goal.lim_beta];
            b_u0 = [beta + self.goal.lim_slip_rate * dt; -beta + self.goal.lim_slip_rate * dt];
            Constraint_A = [Aclf; A_u; A_u0];
            Constraint_b = [bclf; b_u; b_u0];
            options = optimoptions('quadprog', 'Display', 'off');
            % in some Windows OS PC, adding options to the QP may get error
            u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
            acc_n = u(1);
            beta_n = u(2);
            input = [acc_n; beta_n];
            acc_flag = 0;
            e = 0;
        end
        function [acc, beta] = unpack_input(self, input)
            acc = input(1);
            beta = input(2);
        end
        function [x, y, phi, v] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            v = state(4);
        end
    end
end