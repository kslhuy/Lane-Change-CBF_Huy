function trusted_neighbors = get_trusted_neighbors(graph, trust_scores, threshold)
    % Compute trusted neighbors for each vehicle based on trust scores.
    %
    % Parameters:
    %   graph           - Adjacency matrix of the original graph (n x n).
    %   trust_scores    - Cell array where trust_scores{i} contains trust scores 
    %                     beta_i(j) for j in neighbors of i.
    %   threshold       - Trust score threshold (e.g., 0.5) to determine trusted neighbors.
    %
    % Returns:
    %   trusted_neighbors - Cell array where trusted_neighbors{i} contains the indices
    %                       (1 to n) of trusted neighbors of vehicle i.

    num_vehicles = size(graph, 1);
    trusted_neighbors = cell(num_vehicles, 1);

    for i = 1:num_vehicles
        % Get neighbors of vehicle i
        neighbors = find(graph(i, :));
        % Get trust scores for these neighbors
        trust_i = trust_scores{i}; % Vector of trust scores for neighbors
        % Check which neighbors have trust score > threshold
        trusted_mask = trust_i > threshold;
        % Get indices of trusted neighbors
        trusted_indices = neighbors(trusted_mask);
        trusted_neighbors{i} = trusted_indices;
    end
end