classdef ControllerGoal_general
    properties
        target_y; % Target lateral position in frame E
        target_speed; % The ego vehicle's desired speed
        lim_beta; % Slip angle limit
        lim_acc; % Acceleration limit
        lim_slip_rate; % Slip angle changing rate limit
        safety_factor; % Safety factor for the vehicle's maneuvers
        lim_speed; % Speed limit based on the scenario
        lim_yaw; % Yaw rate limit (not used in the provided code)
        scenario; % Driving scenario (1 for highway, 2 for urban road)
    end
    methods
        function self = ControllerGoal_general(initial_lane_id, direction_flag, desired_speed, lanes, lim_slip_angle, lim_acc, lim_slip_rate, safety_factor, scenario)
            % Calculate the target lateral position based on initial lane ID, direction of lane change, and lane width
            self.target_y = ((initial_lane_id + direction_flag) - 1) * lanes.lane_width + 0.5 * lanes.lane_width;
            self.target_speed = desired_speed;
            self.lim_beta = lim_slip_angle;
            self.lim_acc = lim_acc;
            self.lim_slip_rate = lim_slip_rate;
            self.safety_factor = safety_factor;
            self.scenario = scenario;
            [self.lim_speed,~] = scenario.getLimitSpeed();

        end

        
        function self = updateGoal(self, new_target_y, new_target_speed)
            if t == 50  % Change the goal at time step 50
                 self.target_y = new_target_y;
                self.target_speed = new_target_speed;
            end
            
        end

    end
end
