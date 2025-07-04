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

        hi = 0.4; %Time-gap
        ri = 8; 


        %% IDM parameters
        alpha = 2; % Maximum acceleration
        beta = 3;   % Comfortable deceleration
        v0 = 33;       % Desired velocity in free flow
        delta = 4; % Acceleration exponent
        T = 0.4;         % Safe time headway
        s0 = 8;       % Minimum gap distance

        %% CACC

        % Control GAin
        % k_s = 2.0;
        % k_v = 2.0;
        % k_a = 1;

        k_s = 2.0;
        k_v = 2.0;
        k_a = 2.0;
        K;


        % low pass filter
        tau_filter = 0.02;
        
    end
    methods
        function self = ParamOptEgo(dt)
            self.dt = dt;
            self.K = [self.k_s , self.k_v , self.k_a];
        end
    end
end
