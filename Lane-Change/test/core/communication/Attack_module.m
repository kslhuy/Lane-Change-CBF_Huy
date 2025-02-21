function [x_hat_i_j , x_bar_j] = Attack_module(j , x_hat_i_j, x_bar_j, attacker_id,attack_type, fault_intensity)
    % SIMULATE_ATTACK Simulates a cyber-attack on vehicle communication data.
    % 
    % INPUTS:
    
    %   attacker_id       - ID of the vehicle simulating the attack.
    %   attack_type       - Type of attack ('faulty', 'delay', 'dos').
    %   attacked_data     - Data category to attack ('position', 'acceleration', 'all').
    %   fault_intensity   - Intensity of faulty data attack.
    %   delay_time        - Delay time for the delay attack (seconds).
    %   simulation_start_time - Time when simulation started (for delay/DoS attacks).
    %   current_time      - Current time in the simulation.
    %
    % OUTPUT:
    %   x_hat_i_j         - Modified data matrix including the simulated attack.
    
    
    % Apply the chosen attack
    switch attack_type
        case 'faulty'
            % Faulty Data Attack: Modify data from vehicle 2
            x_hat_i_j(:, 2) = x_hat_i_j(:, 2) + randn(size(x_hat_i_j(:, 2))).*fault_intensity; % Add noise or incorrect values
            if j == attacker_id 
                x_bar_j = x_bar_j  + randn(size(x_hat_i_j(:, 2))).*fault_intensity; % Add noise or incorrect values
            end
        
        case 'delay'
            % Simulate delayed data by comparing timestamps
            % Delay/DoS Attack: Add delay to data from vehicle 2
                current_time = toc(simulation_start_time); % Current simulation time
                delay_time = 2; % Delay in seconds
                if (current_time - x_hat_i_j(4, 2)) < delay_time
                    x_hat_i_j(:, 2) = NaN; % Invalidate data if within delay period
                end
    
        
        case 'dos'
            % Simulate a Denial-of-Service attack by invalidating all data
            x_hat_i_j(attack_rows, attacker_id) = NaN;
            
        otherwise
            error('Unknown attack type. Choose "faulty", "delay", or "dos".');
    end
    end
    