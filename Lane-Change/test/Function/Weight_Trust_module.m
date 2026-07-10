classdef Weight_Trust_module < handle
    % Shared trust-aware weight calculator for the MATLAB distributed observer.

    properties
        graph
        vehicle_id = []
        trust_threshold
        kappa
        num_vehicles

        % Python-like shared weight defaults.
        weight_type = "trust_based"
        w0_fixed = 0.4
        w_self_base = 0.2
        w_cap = 0.4
        eta = 0.15
        enable_smoothing = false
        startup_fixed_duration_s = 5.0
        use_gamma_self_weight_adaptation = true
        gamma_self_weight_floor = 0.25
        local_bad_zero_w0_neighbor_total_cap = 0.01
        flag_w0_target_attack_factor = 0.25
        flag_w0_global_est_check_factor = 1.25
        flag_w0_local_est_check_factor = 0.5
        prev_weights = []
    end

    methods
        function self = Weight_Trust_module(graph, trust_threshold, kappa)
            self.graph = graph;
            self.trust_threshold = trust_threshold;
            self.kappa = max(1, round(kappa));
            self.num_vehicles = size(graph, 1);
        end

        function neighbors = get_trusted_neighbors(self, car_idx, trust_scores)
            trust_scores = self.normalize_trust_scores(trust_scores);
            candidates = find(self.graph(car_idx, :) ~= 0);
            candidates(candidates == car_idx) = [];

            trusted = [];
            trusted_scores = [];
            for idx = 1:length(candidates)
                candidate_id = candidates(idx);
                trust = trust_scores(candidate_id);
                if isfinite(trust) && trust >= self.trust_threshold
                    trusted(end + 1) = candidate_id; %#ok<AGROW>
                    trusted_scores(end + 1) = trust; %#ok<AGROW>
                end
            end

            if isempty(trusted)
                neighbors = [];
                return;
            end

            [~, order] = sort(trusted_scores, 'descend');
            trusted = trusted(order);
            neighbors = trusted(1:min(length(trusted), self.kappa));
        end

        function weights_Dis = calculate_weights_Trust(self, vehicle_index, trust_scores, type)
            % Layout: weights_Dis(1) is the virtual/local anchor, vehicle k is k+1.
            if nargin < 4 || strlength(string(type)) == 0
                type = "trust_based";
            end

            mode = lower(string(type));
            if mode == "equal"
                weights_Dis = self.calculate_equal_weights(vehicle_index, trust_scores);
            elseif mode == "paper" || mode == "local" || mode == "distributed"
                weights_Dis = self.calculate_paper_weights(vehicle_index, trust_scores, mode);
            else
                weights_Dis = self.calculate_trust_based_weights(vehicle_index, trust_scores);
            end
        end

        function weights_Dis = calculate_trust_based_weights(self, vehicle_index, trust_scores)
            trust_scores = self.normalize_trust_scores(trust_scores);
            weights_Dis = zeros(1, self.num_vehicles + 1);

            w0 = max(0.0, self.w0_fixed);
            w_self = max(0.0, self.w_self_base);
            neighbor_budget = max(0.0, 1.0 - w0 - w_self);

            weights_Dis(1) = w0;
            weights_Dis(vehicle_index + 1) = w_self;

            trusted_neighbors = self.get_trusted_neighbors(vehicle_index, trust_scores);
            if isempty(trusted_neighbors) || neighbor_budget <= 0
                weights_Dis(vehicle_index + 1) = weights_Dis(vehicle_index + 1) + neighbor_budget;
                weights_Dis = self.finalize_weights(weights_Dis, true);
                return;
            end

            trust_values = max(0.0, trust_scores(trusted_neighbors));
            trust_sum = sum(trust_values);
            if trust_sum <= eps
                weights_Dis(vehicle_index + 1) = weights_Dis(vehicle_index + 1) + neighbor_budget;
                weights_Dis = self.finalize_weights(weights_Dis, true);
                return;
            end

            basis = trust_values / trust_sum;
            raw_neighbor_weights = neighbor_budget * basis;
            [capped_neighbor_weights, overflow_to_self] = self.apply_neighbor_cap(raw_neighbor_weights, basis);

            for idx = 1:length(trusted_neighbors)
                weights_Dis(trusted_neighbors(idx) + 1) = capped_neighbor_weights(idx);
            end
            weights_Dis(vehicle_index + 1) = weights_Dis(vehicle_index + 1) + overflow_to_self;

            weights_Dis = self.finalize_weights(weights_Dis, true);
        end

        function weights_Dis = calculate_weights_for_target(self, host_id, target_id, trust_scores, neighbor_fleet_estimates, direct_measurement, target_trust_model)
            % Python-compatible per-target weights.
            % Layout: weights_Dis(1) is the direct/local anchor, vehicle k is k+1.
            if ~isscalar(target_id)
                parsed_target_id = host_id;
                parsed_trust_scores = target_id;
                parsed_neighbor_fleet_estimates = trust_scores;
                parsed_direct_measurement = [];
                parsed_target_trust_model = [];
                if nargin >= 5
                    parsed_direct_measurement = neighbor_fleet_estimates;
                end
                if nargin >= 6
                    parsed_target_trust_model = direct_measurement;
                end
                if isempty(self.vehicle_id)
                    parsed_host_id = 1;
                else
                    parsed_host_id = self.vehicle_id;
                end
                host_id = parsed_host_id;
                target_id = parsed_target_id;
                trust_scores = parsed_trust_scores;
                neighbor_fleet_estimates = parsed_neighbor_fleet_estimates;
                direct_measurement = parsed_direct_measurement;
                target_trust_model = parsed_target_trust_model;
            else
                if nargin < 6
                    direct_measurement = [];
                end
                if nargin < 7
                    target_trust_model = [];
                end
            end

            mode = lower(string(self.weight_type));
            if mode == "equal"
                weights_Dis = self.calculate_equal_weights_for_target(host_id, target_id, trust_scores, neighbor_fleet_estimates, direct_measurement);
                return;
            elseif mode == "paper"
                weights_Dis = self.calculate_paper_weights_for_target(host_id, target_id, trust_scores, neighbor_fleet_estimates, direct_measurement, target_trust_model);
                return;
            end

            trust_scores = self.normalize_trust_scores(trust_scores);
            available_neighbors = self.get_available_trusted_target_neighbors(host_id, target_id, trust_scores, neighbor_fleet_estimates);

            direct_available = self.is_direct_measurement_available(direct_measurement);
            local_trust = self.read_latest_unit(target_trust_model, ...
                {'local_trust_sample', 'trust_sample_log', 'local_trust_decayed_log'}, 1.0);

            [direct_factor, self_factor, neighbor_factor] = self.resolve_flag_group_factors(target_trust_model);

            if self.use_gamma_self_weight_adaptation
                gamma_self = self.read_latest_unit(target_trust_model, ...
                    {'gamma_self', 'gamma_local_our_self_log'}, 1.0);
                floor_value = self.clip_unit(self.gamma_self_weight_floor, 0.25);
                self_factor = self_factor * (floor_value + (1.0 - floor_value) * gamma_self);
            end

            anchor_gain = max(0.0, self.w0_fixed);
            self_gain = max(0.0, self.w_self_base);
            neighbor_gain = max(0.0, 1.0 - anchor_gain - self_gain);

            if direct_available
                w0_raw = anchor_gain * local_trust * direct_factor;
            else
                w0_raw = 0.0;
            end
            w_self_raw = self_gain * self_factor;

            neighbor_raw = [];
            neighbor_basis = [];
            if ~isempty(available_neighbors)
                trust_values = max(0.0, trust_scores(available_neighbors));
                trust_sum = sum(trust_values);
                if trust_sum > eps
                    neighbor_basis = trust_values / trust_sum;
                    neighbor_raw = neighbor_gain * neighbor_factor * neighbor_basis;
                end
            end

            raw_total = w0_raw + w_self_raw + sum(neighbor_raw);
            weights_Dis = zeros(1, self.num_vehicles + 1);
            if raw_total <= eps
                weights_Dis(host_id + 1) = 1.0;
                return;
            end

            weights_Dis(1) = w0_raw / raw_total;
            weights_Dis(host_id + 1) = w_self_raw / raw_total;
            if ~isempty(neighbor_raw)
                normalized_neighbor_weights = neighbor_raw / raw_total;
                [capped_neighbor_weights, overflow_to_self] = self.apply_neighbor_cap(normalized_neighbor_weights, neighbor_basis);
                for idx = 1:length(available_neighbors)
                    weights_Dis(available_neighbors(idx) + 1) = capped_neighbor_weights(idx);
                end
                weights_Dis(host_id + 1) = weights_Dis(host_id + 1) + overflow_to_self;
            end

            weights_Dis = self.apply_local_bad_zero_w0_bias(weights_Dis, host_id, target_trust_model);
            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_startup_weights_for_target(self, host_id, target_id, neighbor_fleet_estimates, direct_measurement)
            if nargin < 5
                direct_measurement = [];
            end

            available_sources = self.normalize_available_sources(neighbor_fleet_estimates);
            available_neighbors = [];
            for source_id = available_sources
                if source_id == host_id || source_id == target_id
                    continue;
                end
                available_neighbors(end + 1) = source_id; %#ok<AGROW>
            end
            available_neighbors = available_neighbors(1:min(length(available_neighbors), self.kappa));

            weights_Dis = zeros(1, self.num_vehicles + 1);
            if self.is_direct_measurement_available(direct_measurement)
                anchor_weight = max(0.0, self.w0_fixed);
            else
                anchor_weight = 0.0;
            end
            self_weight = max(0.0, self.w_self_base);
            neighbor_budget = max(0.0, 1.0 - anchor_weight - self_weight);

            weights_Dis(1) = anchor_weight;
            if isempty(available_neighbors) || neighbor_budget <= 0
                weights_Dis(host_id + 1) = self_weight + neighbor_budget;
                weights_Dis = self.finalize_weights(weights_Dis, false);
                return;
            end

            basis = ones(1, length(available_neighbors)) / length(available_neighbors);
            raw_neighbor_weights = neighbor_budget * basis;
            [capped_neighbor_weights, overflow_to_self] = self.apply_neighbor_cap(raw_neighbor_weights, basis);
            for idx = 1:length(available_neighbors)
                weights_Dis(available_neighbors(idx) + 1) = capped_neighbor_weights(idx);
            end
            weights_Dis(host_id + 1) = self_weight + overflow_to_self;
            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_equal_weights_for_target(self, host_id, target_id, trust_scores, neighbor_fleet_estimates, direct_measurement)
            trust_scores = self.normalize_trust_scores(trust_scores);
            available_neighbors = self.get_available_trusted_target_neighbors(host_id, target_id, trust_scores, neighbor_fleet_estimates);
            direct_available = self.is_direct_measurement_available(direct_measurement);

            channel_count = length(available_neighbors) + double(direct_available);
            weights_Dis = zeros(1, self.num_vehicles + 1);
            if channel_count <= 0
                weights_Dis(host_id + 1) = 1.0;
                return;
            end

            weight = 1.0 / channel_count;
            if direct_available
                weights_Dis(1) = weight;
            end
            for source_id = available_neighbors
                weights_Dis(source_id + 1) = weight;
            end
            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_paper_weights_for_target(self, host_id, target_id, trust_scores, neighbor_fleet_estimates, direct_measurement, target_trust_model)
            trust_scores = self.normalize_trust_scores(trust_scores);
            available_neighbors = self.get_available_trusted_target_neighbors(host_id, target_id, trust_scores, neighbor_fleet_estimates);
            target_local_trust = self.read_latest_unit(target_trust_model, ...
                {'local_trust_sample', 'trust_sample_log', 'local_trust_decayed_log'}, trust_scores(target_id));
            include_anchor = target_local_trust >= self.trust_threshold && self.is_direct_measurement_available(direct_measurement);

            n_legitimate = length(available_neighbors) + double(include_anchor);
            n_w = max(self.kappa, n_legitimate + 1);
            base_weight = 1.0 / n_w;

            weights_Dis = zeros(1, self.num_vehicles + 1);
            if include_anchor
                weights_Dis(1) = base_weight;
            end
            for source_id = available_neighbors
                weights_Dis(source_id + 1) = base_weight;
            end
            weights_Dis(host_id + 1) = max(0.0, 1.0 - sum(weights_Dis));
            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_equal_weights(self, vehicle_index, trust_scores)
            trusted_neighbors = self.get_trusted_neighbors(vehicle_index, trust_scores);
            weights_Dis = zeros(1, self.num_vehicles + 1);

            channel_count = length(trusted_neighbors) + 1; % virtual/local anchor
            if channel_count <= 0
                weights_Dis(vehicle_index + 1) = 1.0;
                return;
            end

            weight = 1.0 / channel_count;
            weights_Dis(1) = weight;
            for l = trusted_neighbors
                weights_Dis(l + 1) = weight;
            end
            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_paper_weights(self, vehicle_index, trust_scores, type)
            trusted_neighbors = self.get_trusted_neighbors(vehicle_index, trust_scores);
            weights_Dis = zeros(1, self.num_vehicles + 1);

            n_w_i = max(self.kappa, length(trusted_neighbors) + 2);
            base_weight = 1.0 / n_w_i;
            weights_Dis(vehicle_index + 1) = base_weight;
            for l = trusted_neighbors
                weights_Dis(l + 1) = base_weight;
            end

            if type == "local"
                weights_Dis(1) = 1.0 - (length(trusted_neighbors) + 1) * base_weight;
            else
                weights_Dis(1) = base_weight;
            end

            weights_Dis = self.finalize_weights(weights_Dis, false);
        end

        function weights_Dis = calculate_weights_Defaut(self, vehicle_index)
            Vj = self.generate_virtual_graph(self.graph, vehicle_index);
            num_nodes = size(Vj, 1);
            W = zeros(num_nodes);

            for i = 2:num_nodes
                d_i = sum(Vj(i, :));
                for l = 1:num_nodes
                    if Vj(i, l) == 1 || i == l
                        W(i, l) = 1 / (d_i + 1);
                    end
                end
            end

            weights_Dis = self.finalize_weights(W(vehicle_index + 1, :), false);
        end

        function [capped_weights, overflow_to_self] = apply_neighbor_cap(self, neighbor_weights, basis)
            capped_weights = neighbor_weights;
            overflow_to_self = 0.0;
            tol = 1e-12;

            while true
                over_cap = capped_weights > self.w_cap + tol;
                if ~any(over_cap)
                    break;
                end

                overflow = sum(capped_weights(over_cap) - self.w_cap);
                capped_weights(over_cap) = self.w_cap;
                uncapped = capped_weights < self.w_cap - tol;

                if overflow <= tol || ~any(uncapped)
                    overflow_to_self = overflow_to_self + overflow;
                    break;
                end

                basis_sum = sum(max(0.0, basis(uncapped)));
                if basis_sum <= tol
                    overflow_to_self = overflow_to_self + overflow;
                    break;
                end

                capped_weights(uncapped) = capped_weights(uncapped) + ...
                    overflow * max(0.0, basis(uncapped)) / basis_sum;
            end
        end

        function weights = finalize_weights(self, weights, use_smoothing)
            weights = double(weights(:)');
            weights(~isfinite(weights)) = 0.0;
            weights = max(weights, 0.0);

            if sum(weights) <= eps
                weights = zeros(1, self.num_vehicles + 1);
                weights(1) = 1.0;
            else
                weights = weights / sum(weights);
            end

            if use_smoothing && self.enable_smoothing
                if isempty(self.prev_weights) || length(self.prev_weights) ~= length(weights)
                    self.prev_weights = weights;
                else
                    weights = self.eta * weights + (1.0 - self.eta) * self.prev_weights;
                    weights(~isfinite(weights)) = 0.0;
                    weights = max(weights, 0.0);
                    if sum(weights) <= eps
                        weights = self.prev_weights;
                    else
                        weights = weights / sum(weights);
                    end
                end
                self.prev_weights = weights;
            end

            residual = 1.0 - sum(weights);
            if abs(residual) > 1e-12
                [~, max_idx] = max(weights);
                weights(max_idx) = max(0.0, weights(max_idx) + residual);
                weights = weights / sum(weights);
            end
        end

        function trust_scores = normalize_trust_scores(self, trust_scores)
            trust_scores = squeeze(trust_scores);
            trust_scores = double(trust_scores(:)');
            if length(trust_scores) < self.num_vehicles
                trust_scores(end + 1:self.num_vehicles) = 0.0;
            elseif length(trust_scores) > self.num_vehicles
                trust_scores = trust_scores(1:self.num_vehicles);
            end
            trust_scores(~isfinite(trust_scores)) = 0.0;
            trust_scores = min(max(trust_scores, 0.0), 1.0);
        end

        function neighbors = get_available_trusted_target_neighbors(self, host_id, target_id, trust_scores, neighbor_fleet_estimates)
            available_sources = self.normalize_available_sources(neighbor_fleet_estimates);
            trusted = [];
            trusted_scores = [];

            for idx = 1:length(available_sources)
                source_id = available_sources(idx);
                if source_id < 1 || source_id > self.num_vehicles
                    continue;
                end
                if source_id == host_id || source_id == target_id
                    continue;
                end
                trust = trust_scores(source_id);
                if isfinite(trust) && trust >= self.trust_threshold
                    trusted(end + 1) = source_id; %#ok<AGROW>
                    trusted_scores(end + 1) = trust; %#ok<AGROW>
                end
            end

            if isempty(trusted)
                neighbors = [];
                return;
            end

            [~, order] = sort(trusted_scores, 'descend');
            trusted = trusted(order);
            neighbors = trusted(1:min(length(trusted), self.kappa));
        end

        function source_ids = normalize_available_sources(self, neighbor_fleet_estimates)
            if nargin < 2 || isempty(neighbor_fleet_estimates)
                source_ids = 1:self.num_vehicles;
                return;
            end

            if islogical(neighbor_fleet_estimates)
                mask = neighbor_fleet_estimates(:)';
                if length(mask) < self.num_vehicles
                    mask(end + 1:self.num_vehicles) = false;
                end
                source_ids = find(mask(1:self.num_vehicles));
                return;
            end

            if isnumeric(neighbor_fleet_estimates)
                values = double(neighbor_fleet_estimates);
                if isvector(values)
                    values = values(:)';
                    if length(values) == self.num_vehicles && all(values == 0 | values == 1)
                        source_ids = find(logical(values));
                    else
                        source_ids = unique(round(values(isfinite(values) & values >= 1 & values <= self.num_vehicles)));
                    end
                else
                    valid = false(1, min(size(values, 2), self.num_vehicles));
                    for source_id = 1:length(valid)
                        valid(source_id) = all(isfinite(values(:, source_id)));
                    end
                    source_ids = find(valid);
                end
                return;
            end

            source_ids = 1:self.num_vehicles;
        end

        function available = is_direct_measurement_available(~, direct_measurement)
            available = ~isempty(direct_measurement) && all(isfinite(direct_measurement(:)));
        end

        function [direct_factor, self_factor, neighbor_factor] = resolve_flag_group_factors(self, target_trust_model)
            direct_factor = 1.0;
            self_factor = 1.0;
            neighbor_factor = 1.0;

            if self.read_flag(target_trust_model, 'flag_target_attack')
                % Both local/direct and global/fleet evidence are bad:
                % suppress external channels and fall back toward self.
                direct_factor = min(1.0, max(0.0, self.flag_w0_target_attack_factor));
                neighbor_factor = direct_factor;
                self_factor = 1.0 + (1.0 - direct_factor);
            elseif self.read_flag(target_trust_model, 'flag_global_est_check')
                % Local/direct evidence is still usable, but fleet/global
                % consistency is poor. Keep w0 high and reduce neighbor use.
                direct_factor = max(1.0, self.flag_w0_global_est_check_factor);
                neighbor_factor = max(0.0, 1.0 / direct_factor);
            elseif self.read_flag(target_trust_model, 'flag_local_est_check')
                % Local/direct evidence is bad: reduce w0 and lean on
                % trusted fleet/self channels.
                direct_factor = min(1.0, max(0.0, self.flag_w0_local_est_check_factor));
                neighbor_factor = 1.0 + (1.0 - direct_factor);
            end
        end

        function weights = apply_local_bad_zero_w0_bias(self, weights, host_id, target_trust_model)
            if ~self.read_flag(target_trust_model, 'flag_local_est_check')
                return;
            end

            if weights(1) > 1e-9
                return;
            end

            neighbor_indices = 2:length(weights);
            neighbor_indices(neighbor_indices == host_id + 1) = [];
            neighbor_total = sum(weights(neighbor_indices));
            cap_total = self.clip_unit(self.local_bad_zero_w0_neighbor_total_cap, 0.01);
            if neighbor_total > cap_total && neighbor_total > eps
                weights(neighbor_indices) = weights(neighbor_indices) * (cap_total / neighbor_total);
            end
            weights(host_id + 1) = max(0.0, 1.0 - weights(1) - sum(weights(neighbor_indices)));
        end

        function flag = read_flag(~, target_trust_model, flag_name)
            flag = false;
            if isempty(target_trust_model)
                return;
            end

            names = {char(flag_name)};
            if strcmp(char(flag_name), 'flag_target_attack')
                names{end + 1} = 'flag_taget_attk';
            elseif strcmp(char(flag_name), 'flag_global_est_check')
                names{end + 1} = 'flag_glob_est_check';
            end

            for idx = 1:length(names)
                name = names{idx};
                value = [];
                if isstruct(target_trust_model) && isfield(target_trust_model, name)
                    value = target_trust_model.(name);
                elseif isobject(target_trust_model) && isprop(target_trust_model, name)
                    value = target_trust_model.(name);
                end
                if ~isempty(value)
                    flag = logical(value);
                    return;
                end
            end
        end

        function value = read_latest_unit(self, target_trust_model, field_names, default_value)
            value = self.clip_unit(default_value, default_value);
            if isempty(target_trust_model)
                return;
            end

            for idx = 1:length(field_names)
                field_name = field_names{idx};
                raw = [];
                if isstruct(target_trust_model) && isfield(target_trust_model, field_name)
                    raw = target_trust_model.(field_name);
                elseif isobject(target_trust_model) && isprop(target_trust_model, field_name)
                    raw = target_trust_model.(field_name);
                end

                if isempty(raw)
                    continue;
                end
                raw = double(raw(:)');
                raw = raw(isfinite(raw));
                if isempty(raw)
                    continue;
                end
                value = self.clip_unit(raw(end), default_value);
                return;
            end
        end

        function value = clip_unit(~, value, default_value)
            if nargin < 3
                default_value = 0.0;
            end
            if isempty(value) || ~isfinite(double(value))
                value = default_value;
            end
            value = min(max(double(value), 0.0), 1.0);
        end

        function reset(self)
            self.prev_weights = [];
        end

        function Vj = generate_virtual_graph(self, graph, vehicle_index)
            Vj = zeros(self.num_vehicles + 1);
            Vj(2:end, 2:end) = graph;

            Vj(1, vehicle_index + 1) = 1;
            Vj(vehicle_index + 1, 1) = 1;

            neighbors = find(graph(vehicle_index, :));
            for neighbor = neighbors
                Vj(1, neighbor + 1) = graph(vehicle_index, neighbor);
                Vj(neighbor + 1, 1) = graph(neighbor, vehicle_index);
            end
        end
    end
end
