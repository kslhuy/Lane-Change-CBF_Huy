
function attack_module = Atk_Scenarios(attack_module , attack_type ,data_type, case_num , t_star,t_end , attacker_id , victim_id, intensity)

% Default scenario
scenario = struct();

switch attack_type

    %% Scenario I: Bogus Messages
    case "Mix_test"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -5, data_type, {'X'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 5, data_type, {'X'});
            case 3
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -2, data_type, {'velocity'});
            case 4
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 2, data_type, {'velocity'});
                    % Random faulty acceleration - low fault probability (stealthy)
            case 5
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 1, 'probability', 0.2), data_type, {'acceleration'});
            case 6
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 2.5, 'probability', 0.3), data_type, {'velocity'});
            % Random faulty position - high fault probability (frequent faults)
            case 7
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 10, 'probability', 0.5), data_type, {'X'});
            case 8
            % Random faulty velocity - medium fault probability 
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',5,'frequency',0.5), data_type, {'X'});
            case 9
                % scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 2, data_type, {'velocity'});
                scenario = struct('attacker_id', attacker_id, ...
                'victim_id', victim_id, ...
                'start_time', t_star, ...
                'end_time', t_end, ...
                'attack_type', 'drop', ...
                'fault_intensity', 0.5, ... %  percentage to drop no data
                'data_type', data_type, ...
                'attack_row', {'all'}); % Block all data
            otherwise
                disp("Invalid Mix_test case number"); return;
        end


    %% Scenario I: Bogus Messages
    case "Bogus"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', -0.5, data_type, {'velocity'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', 0.5, data_type, {'velocity'});
            case 3
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -10, data_type, {'X'});
            case 4
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 20, data_type, {'X'});
            case 5
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', 0.1, data_type, {'velocity'});
            case 6
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',5,'frequency',0.5), data_type, {'X'});
            case 7
                % Random faulty position - high fault probability (frequent faults)
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 10, 'probability', 0.5), data_type, {'X'});
            case 8
                % Random faulty velocity - medium fault probability 
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 2.5, 'probability', 0.3), data_type, {'velocity'});
            case 9
                % Random faulty acceleration - low fault probability (stealthy)
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 0.5, 'probability', 0.2), data_type, {'acceleration'});
            case 10
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 2, data_type, {'velocity'});
            otherwise
                disp("Invalid Bogus case number"); return;
        end


        %% Scenario I: Bogus Messages Position
    case "POS"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -5, data_type, {'X'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 5, data_type, {'X'});
            case 3
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', -0.5, data_type, {'X'});
            case 4
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', 0.5, data_type, {'X'});
            case 5
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',-5,'frequency',0.5), data_type, {'X'});
            case 6
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',5,'frequency',0.5), data_type, {'X'});
            case 7
                % Random faulty position - intermittent fault
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 5, 'probability', 0.4), data_type, {'X'});
            otherwise
                disp("Invalid Bogus case number"); return;
        end

        %% Scenario I: Bogus Messages Velocity
    case "VEL"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -2, data_type, {'velocity'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 2, data_type, {'velocity'});
            case 3
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', -0.1, data_type, {'velocity'});
            case 4
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', 0.1, data_type, {'velocity'});
            case 5
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',-2.5,'frequency',0.5), data_type, {'velocity'});
            case 6
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',2.5,'frequency',0.5), data_type, {'velocity'});
            case 7
                % Random faulty velocity - intermittent fault
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 2, 'probability', 0.35), data_type, {'velocity'});
            case 8
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', -0.5, data_type, {'velocity'});
            case 9
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', 0.5, data_type, {'velocity'});
            otherwise
                disp("Invalid Bogus case number"); return;
        end

        %% Scenario I: Bogus Messages Acceleration
    case "ACC"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', -5, data_type, {'acceleration'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'bias', 5, data_type, {'acceleration'});
            case 3
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', -0.5, data_type, {'acceleration'});
            case 4
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'linear', 0.5, data_type, {'acceleration'});
            case 5
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',-1,'frequency',0.5), data_type, {'acceleration'});
            case 6
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'sinusoidal', struct('amplitude',1,'frequency',0.5), data_type, {'acceleration'});
            case 7
                % Random faulty acceleration - intermittent fault
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'faulty', struct('intensity', 1, 'probability', 0.25), data_type, {'acceleration'});
            case 8
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', -0.5, data_type, {'acceleration'});
            case 9
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'scaling', 0.5, data_type, {'acceleration'});
            otherwise
                disp("Invalid Bogus case number"); return;
        end


        %% Scenario II: Replay/Delay Attacks
    case "Replay/Delay"
        switch case_num
            case 1
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'replay', -1, data_type, {'acceleration'});
            case 2
                scenario = makeScenario(attacker_id, victim_id, t_star, t_end, 'replay', -1, data_type, {'velocity'});
            otherwise
                disp("Invalid Replay case number"); return;
        end

        %% Scenario III: Collusion Attacks
    case "Collusion"
        switch case_num
            case 1
                scenario = makeScenario([1, 3], victim_id, t_star, t_end, 'scaling', -0.5, data_type, {'velocity'});
            case 2
                scenario = makeScenario([3, 4], victim_id, t_star, t_end, 'scaling', 0.5, data_type, {'velocity'});
            case 3
                scenario = makeScenario([3, 4], victim_id, t_star, t_end, 'bias', -15, data_type, {'X'});
            case 4
                scenario = makeScenario([1, 3], victim_id, t_star, t_end, 'bias', 33, data_type, {'X'});
            case 5
                % Coordinated Acc-Vel attack maintaining kinematics
                scenario = makeScenario([2, 4], victim_id, t_star, t_end, 'coordinated', struct('a_bias', 0.005), data_type, {'acceleration', 'velocity'});
            otherwise
                disp("Invalid Collusion case number"); return;
        end

        %% Scenario IV: Denial-of-Service (DoS) Attacks
    case "DoS"
        scenario = struct('attacker_id', attacker_id, ...
            'victim_id', victim_id, ...
            'start_time', t_star, ...
            'end_time', t_end, ...
            'attack_type', 'drop', ...
            'fault_intensity', 0.5, ... %  percentage to drop no data
            'data_type', data_type, ...
            'attack_row', {'all'}); % Block all data

    otherwise
        disp("Unknown attack type");
        attack_module.setScenario('none', []);
        return;
end

disp(scenario);
attack_module.setScenario('time_based', scenario);
end

function scenario = makeScenario(attacker_id, victim_id, start_time, end_time, attack_type, fault_intensity, data_type, attack_row)
scenario = struct('attacker_id', attacker_id, ...
    'victim_id', victim_id, ...
    'start_time', start_time, ...
    'end_time', end_time, ...
    'attack_type', attack_type, ...
    'fault_intensity', fault_intensity, ...
    'data_type', data_type, ...
    'attack_row', attack_row);
end



% function attack_module =  Atk_Scenarios(attack_module , attack_type ,data_type, case_num , t_star,t_end , attacker_id , victim_id)


%     if (attack_type == "Bogus")
%     %% Scenario I: Bogus Messages
%     % Attack Type: False information on velocity or position
%     % Effect: Vehicles react to incorrect velocity or position values.

%         switch case_num
%             case 1
%                 % Case 1: Underestimated velocity (0.5 * real velocity)
%                 scenario1_Case1 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'scaling', ...
%                                         'fault_intensity', -0.5, ... % 50% reduction
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario1_Case1;
%             case 2
%                 % Case 2: Overestimated velocity (1.5 * real velocity)
%                 scenario1_Case2 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'scaling', ...
%                                         'fault_intensity', 0.5, ... % 50% increase
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario1_Case2;
%             case 3
%                 % Case 3: Underestimated position (real position - 15m)
%                 scenario1_Case3 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', -15, ... % Position decrease
%                                         'data_type', data_type, ...
%                                         'attack_row', 'X');
%                 scenario = scenario1_Case3;
%             case 4
%                 % Case 4: Overestimated position (real position + 33m)
%                 scenario1_Case4 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', 33, ... % Position increase
%                                         'data_type', data_type, ...
%                                         'attack_row', 'X');
%                 scenario = scenario1_Case4;
%             otherwise
%                 disp("Invalid case number");
%         end


%     elseif (attack_type == "Replay/Delay")
%     %% Scenario II: Replay/Delay Attacks
%     % Attack Type: Replaying acceleration/deceleration values
%     % Effect: Vehicles keep outdated speed values, causing instability.

%         switch case_num
%             case 1
%                 % Case 1: Replay acceleration (use previous acceleration)
%                 scenario2_Case1 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', 0.5, ... % Replay acceleration
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario2_Case1;
%             case 2
%                 % Case 2: Replay deceleration (use previous deceleration)
%                 scenario2_Case2 = struct('attacker_id', attacker_id, ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', -0.5, ... % Replay deceleration
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario2_Case2;
%             otherwise
%                 disp("Invalid case number");
%         end




%     elseif (attack_type == "Collusion")
%     %% Scenario III: Collusion Attacks
%     % Attack Type: Three vehicles broadcasting false messages at the same time
%     % Effect: Severe platoon instability due to multiple compromised vehicles.
%         switch case_num
%             case 1
%                 % Case 1: Collusion with underestimated velocity (0.5 * real velocity)
%                 scenario3_Case1 = struct('attacker_id', [1,3], ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'scaling', ...
%                                         'fault_intensity', -0.5, ...
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario3_Case1;
%             case 2
%                 % Case 2: Collusion with overestimated velocity (1.5 * real velocity)
%                 scenario3_Case2 = struct('attacker_id', [3,4], ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'scaling', ...
%                                         'fault_intensity', 0.5, ...
%                                         'data_type', data_type, ...
%                                         'attack_row', 'velocity');
%                 scenario = scenario3_Case2;
%             case 3
%                 % Case 3: Collusion with underestimated position (real position - 15m)
%                 scenario3_Case3 = struct('attacker_id', [3,4], ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', -15, ...
%                                         'data_type', data_type, ...
%                                         'attack_row', 'X');
%                 scenario = scenario3_Case3;
%             case 4
%                 % Case 4: Collusion with overestimated position (real position + 33m)
%                 scenario3_Case4 = struct('attacker_id', [1, 3], ...
%                                         'victim_id', victim_id, ...
%                                         'start_time', t_star, ...
%                                         'end_time', t_end, ...
%                                         'attack_type', 'bias', ...
%                                         'fault_intensity', 33, ...
%                                         'data_type', data_type, ...
%                                         'attack_row', 'X');
%                 scenario = scenario3_Case4;
%             otherwise
%                 disp("Invalid case number");
%         end


%     else
%         attack_module.setScenario('none', []);
%         return;
%     end

%     disp(scenario);
%     attack_module.setScenario('time_based', scenario);

% end