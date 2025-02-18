function dX = Bicycle(self, state, input)
    l_f = self.param.l_f;
    l_r = self.param.l_r;
    l = l_f + l_r;
    [x, y, phi] = self.unpack_state(state);
    [v, beta] = self.unpack_input(input);
    delta_f = atan(l*tan(beta)/l_r); % calculate front steering angle from slip angle
    xdot = v * cos(phi+beta); % velocity in x direction
    ydot = v * sin(phi+beta); % velocity in y direction
    phidot = v * sin(beta) / l_r; % yaw rate
    dX = [xdot; ydot; phidot];
end

function [speed, beta] = unpack_input(self, input)
    speed = input(1);
    beta = input(2);
end

function [x, y, phi] = unpack_state(self, state)
    x = state(1);
    y = state(2);
    phi = state(3);
end