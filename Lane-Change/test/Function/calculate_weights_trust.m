function W = calculate_weights_trust(graph, trusted_neighbors, kappa)
    % Compute weight matrix W based on trusted neighbors.
    %
    % Parameters:
    %   graph             - Adjacency matrix of the original graph (n x n).
    %   trusted_neighbors - Cell array where trusted_neighbors{i} contains indices
    %                       (1 to n) of trusted neighbors of vehicle i.
    %   kappa             - Parameter to limit neighbor influence (kappa > 0).
    %
    % Returns:
    %   W                 - Weight matrix (n+1 x n+1) for the virtual graph,
    %                       where W(i+1, l+1) are the weights for vehicle i,
    %                       and W(i+1, 1) corresponds to the virtual node (node 0).

    num_vehicles = size(graph, 1);
    num_nodes = num_vehicles + 1; % Including virtual node 0
    W = zeros(num_nodes, num_nodes); % Initialize weight matrix

    for i = 1:num_vehicles
        % Get trusted neighbors of vehicle i
        N_i_t = trusted_neighbors{i}; % Indices of trusted neighbors (1 to n)
        % Compute n_w_i(t) = max(kappa, |N_i(t)| + 1)
        n_w_i = max(kappa, length(N_i_t) + 1); % +1 for self (vehicle i)
        weight = 1 / n_w_i;

        % Set self-weight: W(i+1, i+1) = weight
        W(i+1, i+1) = weight;

        % Set weights for trusted neighbors: W(i+1, l+1) = weight for l in N_i(t)
        for l = N_i_t
            W(i+1, l+1) = weight;
        end

        % Set weight to virtual node (node 0): W(i+1, 1) = 1 - |N_i(t)| * weight
        W(i+1, 1) = 1 - length(N_i_t) * weight;
    end

    % Note: Row 1 (virtual node 0) is left as zeros, handled separately in observer update
end