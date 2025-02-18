function W = calculate_weights_Defaut(Vj)
    % Function to calculate consensus weights for a given virtual graph
    %
    % Parameters:
    %   Vj - Adjacency matrix representing the virtual graph. It is a square 
    %        matrix where Vj(i, j) = 1 if there is an edge between node i and 
    %        node j, and 0 otherwise.
    %
    % Returns:
    %   W - Weight matrix where W(i, j) represents the weight assigned to the 
    %       edge between node i and node j. The weights are calculated based 
    %       on the degree of each node.
    %
    % The function initializes a weight matrix W with zeros. For each node i 
    % (starting from the second node), it calculates the degree d_i of the node. 
    % Then, for each node l, if there is an edge between node i and node l or 
    % if i equals l (self-loop), it assigns a weight of 1 / (d_i + 1) to W(i, l).

    % https://discord.com/channels/1123389035713400902/1326223031667789885/1328426310963564564

    num_nodes = size(Vj, 1);
    W = zeros(num_nodes); % Initialize weights matrix

    for i = 2:num_nodes % Include node 0 (index 1 in MATLAB)
        d_i = sum(Vj(i, :)); % Degree of node i
        for l = 1:num_nodes % Include all nodes in the virtual graph
            if Vj(i, l) == 1 || i == l % Neighbor or self
                W(i, l) = 1 / (d_i + 1);
            end
        end
    end
end