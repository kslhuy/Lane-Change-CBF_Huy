%% TEST RANDOM FAULTY ATTACKS
%% This script demonstrates the new random faulty attack behavior

clear; clc; close all;

% Load configuration
Config

fprintf('=== Testing Random Faulty Attacks ===\n\n');

% Test different faulty attack scenarios
attack_scenarios = [
    struct('name', 'Always Faulty (Old)', 'type', 'Bogus', 'case', 8, 'intensity', 2.5)
    struct('name', 'High Random Fault', 'type', 'Mix_test', 'case', 7, 'intensity', struct('intensity', 10, 'probability', 0.5))
    struct('name', 'Medium Random Fault', 'type', 'Mix_test', 'case', 8, 'intensity', struct('intensity', 2.5, 'probability', 0.3))
    struct('name', 'Low Random Fault', 'type', 'Mix_test', 'case', 9, 'intensity', struct('intensity', 0.5, 'probability', 0.2))
];

% Test parameters
t_star = 5;
t_end = 10;
simulation_time = 15;
attacker_id = 2;
victim_id = -1;
data_type = "local";

% Create figure for comparison
figure('Position', [100, 100, 1200, 800]);

for scenario_idx = 1:length(attack_scenarios)
    scenario = attack_scenarios(scenario_idx);
    
    fprintf('Testing: %s\n', scenario.name);
    
    % Set up attack
    attack_module = Attack_module(Scenarios_config.dt);
    attack_module = Atk_Scenarios(attack_module, scenario.type, data_type, scenario.case, t_star, t_end, attacker_id, victim_id);
    center_communication = CenterCommunication(attack_module);
    
    % Create vehicles (simplified setup)
    initial_lane_id = 1;
    direction_flag = 0;
    
    car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
    car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
    car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
    car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
    
    platoon_vehicles = [car1; car2; car3; car4];
    
    % Assign neighbors
    car1.assign_neighbor_vehicle(platoon_vehicles, [], "None", center_communication, graph);
    car2.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
    car3.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
    car4.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
    
    % Run simulation
    simulator = Simulator(straightLanes, [], platoon_vehicles, Scenarios_config.dt, false);
    [state_log, input_log] = simulator.startSimulation(simulation_time, t_star, t_end, attacker_id);
    
    % Plot attack values
    subplot(2, 2, scenario_idx);
    try
        attack_module.plotAttackValues('vehicle_id', attacker_id, 'data_plot', "local");
        title(sprintf('%s\nAttacker: V%d', scenario.name, attacker_id));
        xlabel('Time (s)');
        ylabel('Attack Perturbation');
        grid on;
        
        % Add attack period indication
        hold on;
        ylims = ylim;
        fill([t_star t_end t_end t_star], [ylims(1) ylims(1) ylims(2) ylims(2)], 'red', 'Alpha', 0.1, 'EdgeColor', 'none');
        text((t_star + t_end)/2, ylims(2)*0.9, 'Attack Period', 'HorizontalAlignment', 'center', 'Color', 'red');
        hold off;
        
        % Count number of actual faults applied
        attack_values = attack_module.attack_values_local;
        if ~isempty(attack_values)
            non_zero_attacks = sum([attack_values.perturbation] ~= 0);
            total_possible = length(attack_values);
            fault_rate = non_zero_attacks / total_possible;
            
            text(0.02, 0.98, sprintf('Actual Fault Rate: %.2f%%', fault_rate*100), ...
                'Units', 'normalized', 'VerticalAlignment', 'top', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');
        end
        
    catch ME
        text(0.5, 0.5, sprintf('No attack data\n%s', ME.message), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        title(scenario.name);
    end
    
    fprintf('  Completed simulation for %s\n', scenario.name);
end

sgtitle('Comparison of Random Faulty Attack Behaviors', 'FontSize', 14, 'FontWeight', 'bold');

% Print summary
fprintf('\n=== SUMMARY ===\n');
fprintf('Random faulty attacks now have configurable probability:\n');
fprintf('- High Random (50%% chance): Frequent but unpredictable faults\n');
fprintf('- Medium Random (30%% chance): Moderate intermittent faults\n'); 
fprintf('- Low Random (20%% chance): Rare, stealthy faults\n');
fprintf('- Backward compatible with old constant fault format\n\n');

fprintf('This makes attacks more realistic and harder to detect!\n');
fprintf('The trust system will need to handle intermittent behavior.\n');