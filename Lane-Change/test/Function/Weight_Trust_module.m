classdef Weight_Trust_module < handle
    % TrustManager - A class to manage trust scores, trusted neighbors, and weights
    % for a platoon of vehicles under potential attacks.

    properties
        graph           % Adjacency matrix of the original graph (n x n)
        % trust_scores    % Cell array of trust scores, trust_scores{i} for vehicle i's neighbors
        trust_threshold % Threshold for determining trusted neighbors (e.g., 0.5)
        kappa           % Parameter to limit neighbor influence (kappa > 0)
        num_vehicles    % Number of vehicles in the platoon
    end

    methods
        % Constructor
        function self = Weight_Trust_module(graph, trust_threshold, kappa)
            % Initialize the TrustManager with graph and parameters
            %
            % Parameters:
            %   graph              - Adjacency matrix (n x n)
            %   initial_trust_scores - Cell array of initial trust scores
            %   trust_threshold    - Trust score threshold (e.g., 0.5)
            %   kappa              - Influence limit parameter
            self.graph = graph;
            self.trust_threshold = trust_threshold;
            self.kappa = kappa;
            self.num_vehicles = size(graph, 1);
        end


        % Compute trusted neighbors
        function neighbors = get_trusted_neighbors(self, car_idx, trust_scores)
            % Compute trusted neighbors for a specific vehicle based on current trust scores
            %
            % Parameters:
            %   car_idx - Index of the vehicle (1 to n)
            %   trust_scores - Array of trust scores for all vehicles
            %
            % Returns:
            %   neighbors - Array of indices of trusted neighbors for vehicle car_idx

            % Get direct neighbors from adjacency matrix
            neighbors = find(self.graph(car_idx, :));

            % Remove neighbors whose trust score is below the threshold
            if ~isempty(neighbors)
                low_trust_mask = trust_scores(neighbors) <= self.trust_threshold;
                neighbors(low_trust_mask) = []; % Remove untrusted neighbors
            end
        end



        % Compute weight matrix based on trusted neighbors
        function weights_Dis = calculate_weights_Trust(self, vehicle_index, trust_scores, type)
            % Calculate trust-based consensus weights for a specific vehicle
            %
            % Parameters:
            %   vehicle_index - Index of the vehicle (1 to n)
            %   trust_scores - Array of trust scores for all vehicles
            %   type - Weight distribution type: "local" (default) or "distributed"
            %
            % Returns:
            %   weights_Dis - Weight array (1 x num_vehicles+1) for consensus

            % Check if the 'type' argument is provided
            if nargin < 4
                type = "local"; % Default value
            end

            virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index);
            trusted_neighbors_set = self.get_trusted_neighbors(vehicle_index, trust_scores);
            num_nodes = size(virtual_graph, 1);
            weights_Dis = zeros(1, num_nodes);

            N_i_t = trusted_neighbors_set; % Trusted neighbors (indices 1 to n)
            n_w_i = max(self.kappa, length(N_i_t) + 1 + 1); % +1 for self virtual node 
            weight = 1 / n_w_i;

            % Set self-weight: W(vehicle_index+1)
            weights_Dis(1, vehicle_index + 1) = weight;

            % Set weights for trusted neighbors
            for l = N_i_t
                weights_Dis(1, l + 1) = weight;
            end

            % Set weight to virtual node: W(1, 1)   
            if strcmp(type, "local")
                % Using weight matrix prioritizing local estimation
                weights_Dis(1, 1) = 1 - (length(N_i_t) + 1) * weight;
            else
                % Using weight matrix distributed equally
                weights_Dis(1, 1) = weight;
            end
            
            % NOTE: The weight matrix W(1, 1) may need adjustment based on trust
            % See more details in Observer class      
        end





        % % Compute weight matrix based on trusted neighbors
        % function W = calculate_weights_Trust(self,vehicle_index , trust_scores)
        %     % Compute weight matrix W based on trusted neighbors for a virtual graph
        %     %
        %     % Parameters:
        %     %   virtual_graph - Adjacency matrix of the virtual graph (n+1 x n+1)
        %     %
        %     % Returns:
        %     %   W            - Weight matrix (n+1 x n+1)

        %     virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index);
        %     trusted_neighbors_set = self.get_trusted_neighbors( vehicle_index , trust_scores);
        %     num_nodes = size(virtual_graph, 1);
        %     W = zeros(num_nodes, num_nodes);

        %     for i = 1:self.num_vehicles
        %         N_i_t = trusted_neighbors_set{i}; % Trusted neighbors (indices 1 to n)
        %         n_w_i = max(self.kappa, length(N_i_t) + 1); % +1 for self
        %         weight = 1 / n_w_i;

        %         % Set self-weight: W(i+1, i+1)
        %         W(i+1, i+1) = weight;

        %         % Set weights for trusted neighbors: W(i+1, l+1)
        %         for l = N_i_t
        %             W(i+1, l+1) = weight;
        %         end

        %         % Set weight to virtual node: W(i+1, 1)
        %         W(i+1, 1) = 1 - length(N_i_t) * weight;
        %     end
        % end


        function weights_Dis = calculate_weights_Defaut(self, vehicle_index)
            % Calculate default consensus weights (without trust) for a given vehicle
            %
            % Parameters:
            %   vehicle_index - Index of the vehicle (1 to n)
            %
            % Returns:
            %   weights_Dis - Weight array (1 x num_vehicles+1) for consensus
            %                 based on graph topology without trust considerations

            Vj = self.generate_virtual_graph(self.graph, vehicle_index);
            num_nodes = size(Vj, 1);
            W = zeros(num_nodes); % Initialize weights matrix

            for i = 2:num_nodes % Start from node 1 (vehicle nodes, skip virtual node 0)
                d_i = sum(Vj(i, :)); % Degree of node i
                for l = 1:num_nodes % Include all nodes in the virtual graph
                    if Vj(i, l) == 1 || i == l % Neighbor or self
                        W(i, l) = 1 / (d_i + 1);
                    end
                end
            end

            weights_Dis = W(vehicle_index + 1, :); % Extract weights for specified vehicle
        end

        function Vj = generate_virtual_graph(self, graph, vehicle_index)
            % Generate virtual graph by adding an extra node (node 0) for consensus
            %
            % This function creates a virtual graph by adding node 0 and connecting
            % it to the specified vehicle and its neighbors for consensus protocols.
            %
            % Parameters:
            %   graph - Adjacency matrix of the original graph (n x n)
            %   vehicle_index - Index of the vehicle to which node 0 connects (1 to n)
            %
            % Returns:
            %   Vj - Virtual graph adjacency matrix ((n+1) x (n+1))

            % Function to generate virtual graph for a given vehicle
            Vj = zeros(self.num_vehicles + 1); % Initialize virtual graph with an extra node

            % Copy the adjacency matrix to the virtual graph
            Vj(2:end, 2:end) = graph;

            % Set the bidirectional edge between node 0 and the specified vehicle
            Vj(1, vehicle_index + 1) = 1;
            Vj(vehicle_index + 1, 1) = 1;

            % Set the edges between node 0 and node j’s neighbors
            neighbors = find(graph(vehicle_index, :));
            for neighbor = neighbors
                Vj(1, neighbor + 1) = graph(vehicle_index, neighbor);
                Vj(neighbor + 1, 1) = graph(neighbor, vehicle_index);
            end
        end

    end
end