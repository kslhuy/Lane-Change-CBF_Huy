% d = [18.79, 18.4]; % Example distance vector

d = [18.79, 18.4]; % Example distance vector

d_norm = 17;
a_host = 1; % Example host acceleration
a_y = 1.3; % Example neighbor acceleration
ts = 0.5; % Example time step


v_rel = abs(diff(d)) / ts;

a_diff = a_y - a_host;

d_add = d_norm  / (d(1));
a_score = max(1 - abs(v_rel * a_diff), 0)
a_score_ts = max(1 - abs(v_rel / ts * a_diff), 0)

a_score_d  = max(1 - abs(v_rel*d_add * a_diff), 0)

% a_score = a_score^1 % Tune wa_nearby
