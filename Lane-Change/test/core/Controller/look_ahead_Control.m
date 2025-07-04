classdef look_ahead_Control < handle % Extended Look-Ahead Controller for Lane-Changing Vehicles
    properties
        param_opt;
        param_sys;
        goal;
        straightlane;
        vehicle_number;
        controller;
    end
    methods
        function self = look_ahead_Control(controller)
            self.controller = controller;
            self.param_opt = controller.param_opt;
            self.param_sys = controller.param_sys;
            self.goal = controller.goal;
            self.straightlane = controller.straightlane;
        end

        function [acc_flag,input, e] = get_optimal_input(self, host_car_id,state, last_input, lane_id, input_log, inital_land_ID, direction_flag, type_state,acc_flag)
            acc_flag = 0;

            l_r = self.param_sys.l_r;
            l_f = self.param_sys.l_f;
            C1 = self.param_sys.C1;
            C2 = self.param_sys.C2;
            mass = self.param_sys.mass;




            % Extract parameters
            hi = self.param_opt.hi; % Time-gap policy
            ri = self.param_opt.ri; % Standstill distance
            k1 = 3.5;
            k2 = 3.5;

            % Unpack ego state
            [x, y, theta, v] = self.unpack_state(state);

            % Get lead vehicle
            [car_fc, ~, ~] = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag,host_car_id);

            if isempty(car_fc)
                disp("No car preceding vehicle 4"+ num2str(host_car_id))
                % disp()
                acc = 0;
                omega = 0;
                input = [acc; omega];
                e = 0;
                return;
            end




            if (type_state == "true")
                % Unpack lead vehicle state
                [x_lead, y_lead, theta_lead, v_lead] = self.unpack_state(car_fc.state);
            else %estimated
                %since this we use that like local , so use mesurement instead of estimated
                % mesurement mean true state
                est_car_fc = self.controller.vehicle.observer.est_global_state_current(:,car_fc.vehicle_number) ;
                [x_lead, y_lead, theta_lead, v_lead] = self.unpack_state(est_car_fc);

                %% Here check if the relative state is estimated correctly
                [x_lead_true, y_lead_true, theta_lead_true, v_lead_true] = self.unpack_state(car_fc.state);

                
                mesurement_rel_pos = sqrt(( x_lead_true - x)^2 + (y_lead_true - y)^2);
                mesurement_rel_vel = v_lead_true - v;

                est_rel_pos = sqrt(( x_lead - x)^2 + (y_lead - y)^2);
                est_rel_vel = v_lead - v;

                if (mesurement_rel_pos - est_rel_pos)/mesurement_rel_pos < 0 || ( mesurement_rel_vel - est_rel_vel)/mesurement_rel_vel > 0.1
                    disp("The relative state is not estimated correctly")
                    % using true state
                    [x_lead, y_lead, theta_lead, v_lead] = self.unpack_state(car_fc.state);
                end

                
            end



            % Compute extended look-ahead point
            kappa_lead = self.compute_curvature(v_lead, theta_lead);
            s_bar = self.compute_s_bar(kappa_lead, ri, hi, v);
            sx_lead = s_bar * sin(theta_lead);
            sy_lead = -s_bar * cos(theta_lead);

            % Compute tracking errors
            z1 = x_lead + sx_lead - x - (ri + hi * v) * cos(theta);
            z2 = y_lead + sy_lead - y - (ri + hi * v) * sin(theta);
            z3 = v_lead * cos(theta_lead) - v * cos(theta);
            z4 = v_lead * sin(theta_lead) - v * sin(theta);

            % Compute control inputs
            acc = k1 * z1 + z3;
            omega = k2 * z2 + z4;

            % Convert omega to steering angle delta
            %% Version 1 : using bycle model

            delta = atan(omega * l_r / max(v, 0.1));

            %% Version 2 : The single track vehicle model
            % Wheelbase = l_r + l_f;
            % delta = Wheelbase*omega / max(v, 0.1)  - mass*omega*v/Wheelbase*(l_f/C2 - l_r/C1) ;



            input = [acc; delta];
            % Return errors
            % e = [z1, z2, z3, z4];
            e = 0;
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

        % function [car_fc, car_bt, car_ft] = get_surrounding_vehicles(self, x, current_lane_id, direction_flag)
        %     current_lane_vehicles = [];
        %     target_lane_vehicles = [];
        %     num_veh = length(self.other_vehicles);
        %
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
        %
        %         % Check if the vehicle is in the target lane
        %         elseif (self.other_vehicles(i).lane_id == current_lane_id + direction_flag)
        %             target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
        %         % Check if the vehicle is in the target lane offset by 1.5 times the direction flag
        %         elseif self.other_vehicles(i).lane_id == current_lane_id + 1.5 * direction_flag
        %             target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
        %         end
        %     end
        %
        %     car_fc = [];
        %     carfc_range = x + 100;
        %     for j = 1:length(current_lane_vehicles)
        %         if current_lane_vehicles(j).state(1) <= carfc_range
        %             car_fc = current_lane_vehicles(j);
        %             carfc_range = current_lane_vehicles(j).state(1);
        %         end
        %     end
        %
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
