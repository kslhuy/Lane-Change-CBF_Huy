classdef ParamVeh
    properties
        l_r = 1.74; % distance between vehicle's c.g. and rear axle
        l_f = 1.11; % distance between vehicle's c.g. and front axle
        width = 1.86; % vehicle body's widty
        dt = 0.01;
        l_fc = 2.15; % vehicle body's part that is in front of its c.g.
        l_rc = 2.77; % vechile body's part that is behind its c.g.
        tau = 0.1; % constant time lag for the vehicle model
        tau_v = 0.6; % time constant for the velocity model
        C1 = 46333; % Cornering stiffness front wheel
        C2 = 34252; %  Cornering stiffness rear wheel N/rad
        mass = 708; % kg
    end
end
