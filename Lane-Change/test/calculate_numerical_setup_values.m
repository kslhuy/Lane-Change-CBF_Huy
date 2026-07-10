clearvars;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(fullfile(script_dir, 'Function'));
addpath(fullfile(script_dir, 'core'));
addpath(fullfile(script_dir, 'core', 'observer'));
addpath(fullfile(script_dir, 'core', 'Controller'));
addpath(fullfile(script_dir, 'core', 'Trust'));
addpath(fullfile(script_dir, 'core', 'communication'));

config_text = fileread(fullfile(script_dir, 'Config.m'));
main_text = fileread(fullfile(script_dir, 'main_mean_attack_plot.m'));

param_sys = ParamVeh();
trust_model = TriPTrustModel();

num_vehicles = read_numeric_assignment(main_text, 'num_vehicles');
graph = ones(num_vehicles) - eye(num_vehicles);

dt = read_numeric_assignment(config_text, 'dt');
trust_threshold = read_numeric_assignment(config_text, 'trust_threshold');
kappa = read_numeric_assignment(config_text, 'kappa');
local_trust_fusion_mode = read_string_assignment(config_text, ...
    'Scenarios_config.local_trust_fusion_mode');
dirichlet_type = read_string_assignment(config_text, ...
    'Scenarios_config.Dichiret_type');
model_vehicle_type = read_string_assignment(config_text, ...
    'Scenarios_config.model_vehicle_type');

% Match main_mean_attack_plot.m: it rebuilds the 5-vehicle module after Config.m.
weight_module = Weight_Trust_module(graph, trust_threshold, kappa);

l_f = param_sys.l_f;
l_r = param_sys.l_r;
wheelbase = l_f + l_r;

omega_p = trust_model.py_weight_distance;
omega_v = trust_model.py_weight_velocity;
omega_psi = trust_model.py_weight_heading;
omega_u = trust_model.py_weight_acceleration;
psi_th = trust_model.heading_base_tolerance_rad;
beta_local = trust_model.wt;
beta_global = trust_model.wt_global;
missing_packet_decay = trust_model.lambda_h;
c_tr = trust_model.C;
q = trust_model.k;

w0_base = weight_module.w0_fixed;
w_self_base = weight_module.w_self_base;
w_cap = weight_module.w_cap;
startup_fixed_duration_s = weight_module.startup_fixed_duration_s;
neighbor_budget = 1 - w0_base - w_self_base;

dx = [0.01, 0.01, 1.0, 0.2];
vmax = 33.0;
delta_max = 0.35;
gain = scaled_bicycle_gain(dt, wheelbase, vmax, delta_max, dx);

fprintf('Computed from MATLAB source/classes\n');
fprintf('====================================\n');
fprintf('N = %d\n', num_vehicles);
fprintf('graph = all-to-all without self edges, ones(N)-eye(N)\n');
fprintf('T_s = %.6g s\n', dt);
fprintf('l_f = %.6g m\n', l_f);
fprintf('l_r = %.6g m\n', l_r);
fprintf('L_w = l_f + l_r = %.6g m\n', wheelbase);
fprintf('active model_vehicle_type = %s\n\n', model_vehicle_type);

fprintf('Trust parameters\n');
fprintf('----------------\n');
fprintf('local_trust_fusion_mode = %s\n', local_trust_fusion_mode);
fprintf(['local powers (position/distance, velocity, heading, ', ...
    'acceleration/input) = (%.6g, %.6g, %.6g, %.6g)\n'], ...
    omega_p, omega_v, omega_psi, omega_u);
fprintf('Dichiret_type = %s\n', dirichlet_type);
fprintf('rating-vector aging wt(local) = %.6g\n', beta_local);
fprintf('rating-vector aging wt_global = %.6g\n', beta_global);
fprintf('missing-packet trust decay lambda_h = %.6g\n', missing_packet_decay);
fprintf('C_tr = %.6g\n', c_tr);
fprintf('q = %d\n', q);
fprintf('heading base tolerance = %.6g rad\n', psi_th);
fprintf('theta_min = trust_threshold = %.6g\n\n', trust_threshold);

fprintf('Observer weight-module parameters\n');
fprintf('---------------------------------\n');
fprintf('kappa = %d\n', kappa);
fprintf('w0_fixed/base = %.6g\n', w0_base);
fprintf('w_self_base = %.6g\n', w_self_base);
fprintf('nominal neighbor budget = 1 - w0 - w_self = %.6g\n', neighbor_budget);
fprintf('per-neighbor cap w_cap = %.6g\n', w_cap);
fprintf(['startup_fixed_duration_s = %.6g ', ...
    '(class default active after main_mean_attack_plot.m re-instantiates the module)\n\n'], ...
    startup_fixed_duration_s);

fprintf('Scaled bicycle contraction check\n');
fprintf('--------------------------------\n');
fprintf('D_x = diag([0.01 0.01 1 0.2])\n');
fprintf('Envelope: v <= %.6g m/s, |delta| <= %.6g rad\n', vmax, delta_max);
fprintf('max ||D_x A D_x^-1||_inf = %.9f\n', gain.Lx_inf);
fprintf('max ||D_x A D_x^-1||_2   = %.9f\n', gain.Lx_2_check);
fprintf('argmax psi = %.9f rad\n', gain.argmax_psi);
fprintf('argmax delta = %.9f rad\n', gain.argmax_delta);
fprintf('accepted anchor mass eta0 * theta_min = %.6g\n', gain.anchor_mass);
fprintf('alpha = Lx_max * (1 - anchor_mass) = %.9f\n\n', gain.alpha);

fprintf('LaTeX replacement\n');
fprintf('-----------------\n');
fprintf(['The numerical simulation uses five vehicles with all-to-all V2V communication; ', ...
    'each vehicle runs the observer, and small fixed parameter variations are treated ', ...
    'as modeling uncertainty. The main parameters are $N=5$, $T_s=%.2f$~s, ', ...
    'nominal geometry $l_f=%.2f$~m and $l_r=%.2f$~m, giving $L_w=%.2f$~m, ', ...
    'local-trust product fusion with internal powers ', ...
    '$(\\omega_p,\\omega_v,\\omega_\\psi,\\omega_u)=(%.1f,%.1f,%.1f,%.1f)$ ', ...
    'for distance/position, velocity, heading, and acceleration/input consistency, ', ...
    'class-filter aging weight $\\beta_{\\mathrm{trust}}=%.2f$ in the active single-rating update, ', ...
    'class confidence $C_{\\mathrm{tr}}=%.1f$, $q=%d$ trust classes, ', ...
    'heading base tolerance $\\psi_{\\mathrm{th}}=%.2f$~rad, and trust threshold ', ...
    '$\\theta_{\\min}=%.1f$. The observer-weight module uses base gains ', ...
    '$w_{0,\\mathrm{base}}=%.1f$ and $w_{\\mathrm{self,base}}=%.1f$, ', ...
    'with nominal neighbor budget $%.1f$ and per-neighbor cap $%.1f$; after trust ', ...
    'and flag normalization these are adaptive weights rather than fixed lower bounds.\n\n'], ...
    dt, l_f, l_r, wheelbase, omega_p, omega_v, omega_psi, omega_u, beta_local, ...
    c_tr, q, psi_th, trust_threshold, w0_base, w_self_base, neighbor_budget, w_cap);

fprintf(['On $v\\le33$~m/s and $|\\delta|\\le0.35$~rad, the induced one-step ', ...
    'bicycle gain evaluates to $L_{x,\\max}\\approx%.3f$. With accepted anchor ', ...
    'mass $\\underline w_0=\\eta_0\\theta_{\\min}=%.2f$, ', ...
    'Lemma~\\ref{lem_anchor_contraction} gives ', ...
    '$\\alpha=L_{x,\\max}(1-\\underline w_0)\\approx%.3f\\times0.80\\approx%.3f<1$.\n'], ...
    gain.Lx_inf, gain.anchor_mass, gain.Lx_inf, gain.alpha);

function value = read_numeric_assignment(source_text, name)
    pattern = ['(?<![\w.])', regexptranslate('escape', name), ...
        '\s*=\s*([-+]?\d+(\.\d+)?([eE][-+]?\d+)?)'];
    tokens = regexp(source_text, pattern, 'tokens', 'once');
    if isempty(tokens)
        error('calculate_setup:missingNumericAssignment', ...
            'Could not find numeric assignment for "%s".', name);
    end
    value = str2double(tokens{1});
end

function value = read_string_assignment(source_text, name)
    pattern = ['(?<![\w.])', regexptranslate('escape', name), ...
        '\s*=\s*["'']([^"'']+)["'']'];
    tokens = regexp(source_text, pattern, 'tokens', 'once');
    if isempty(tokens)
        error('calculate_setup:missingStringAssignment', ...
            'Could not find string assignment for "%s".', name);
    end
    value = tokens{1};
end

function gain = scaled_bicycle_gain(ts, wheelbase, vmax, delta_max, dx)
    xy_a = dx(1) * ts * vmax / dx(3);
    xy_b = dx(1) * ts / dx(4);
    xy_row = 1 + hypot(xy_a, xy_b);
    heading_row = 1 + dx(3) * ts / wheelbase * tan(delta_max) / dx(4);

    gain.Lx_inf = max([xy_row, heading_row, 1]);
    gain.argmax_psi = atan2(xy_a, xy_b);
    if heading_row >= xy_row
        gain.argmax_delta = delta_max;
    else
        gain.argmax_delta = 0;
    end

    gain.Lx_2_check = spectral_norm_check(ts, wheelbase, vmax, delta_max, dx);
    gain.anchor_mass = 0.4 * 0.5;
    gain.alpha = gain.Lx_inf * (1 - gain.anchor_mass);
end

function best = spectral_norm_check(ts, wheelbase, vmax, delta_max, dx)
    dmat = diag(dx);
    inv_dmat = diag(1 ./ dx);
    psi_values = linspace(0, 2*pi, 4001);
    delta_values = [-delta_max, delta_max];
    best = 0;
    for delta = delta_values
        tan_delta = tan(delta);
        for psi = psi_values
            a = [ ...
                1, 0, -ts * vmax * sin(psi), ts * cos(psi); ...
                0, 1,  ts * vmax * cos(psi), ts * sin(psi); ...
                0, 0, 1, ts / wheelbase * tan_delta; ...
                0, 0, 0, 1];
            scaled = dmat * a * inv_dmat;
            best = max(best, norm(scaled, 2));
        end
    end
end
