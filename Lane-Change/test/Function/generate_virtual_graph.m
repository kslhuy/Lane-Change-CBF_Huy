function Vj = generate_virtual_graph(graph, vehicle_index)
    
    %{
    % This function generates a virtual graph by adding an extra node (node 0)
    % and connecting it to the specified vehicle and its neighbors.
    % The input 'graph' is the adjacency matrix of the original graph.
    % The input 'vehicle_index' is the index of the vehicle to which node 0 will be connected.
    %}

    % Function to generate virtual graph for a given vehicle
    num_vehicles = size(graph, 1);
    Vj = zeros(num_vehicles + 1); % Initialize virtual graph with an extra node

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