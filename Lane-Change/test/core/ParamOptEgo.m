classdef ParamOptEgo
    properties

        %% CBF-CLF-QP parameters
        alpha_y = 0.8;
        alpha_v = 1.7;
        alpha_yaw = 12;
        gamma_1 = 1;
        gamma_2 = 1;
        gamma_3 = 1;
        gamma_4 = 1;
        gamma_5 = 1;
        gamma_6 = 1;
        dt;
        H = [0.01, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0; ...
            0, 0, 15, 0, 0; ...
            0, 0, 0, 0.1, 0; ...
            0, 0, 0, 0, 400; ...
            ];
        F = [0; 0; 0; 0; 0];
        %% Look ahead parameters

        hi = 0.4;
        ri = 8;


        %% IDM parameters
        alpha = 1; % Maximum acceleration
        beta = 2;   % Comfortable deceleration
        v0 = 120;       % Desired velocity in free flow
        delta = 4; % Acceleration exponent
        T = 1;         % Safe time headway
        s0 = 4;       % Minimum gap distance

    end
    methods
        function self = ParamOptEgo(dt)
            self.dt = dt;
        end
    end
end
