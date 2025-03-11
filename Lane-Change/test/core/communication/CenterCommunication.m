classdef CenterCommunication < handle
    properties
        vehicles; % A container to store all vehicles in the platoon
        state_logs; % A container to store the state logs of all vehicles
        input_logs; % A container to store the input logs of all vehicles
        
        global_state_storage; % A container to store the global state of all vehicles
        global_state_atk_storage; % A container to store the global state of all vehicles



        local_state_storage; % A container to store the local state of all vehicles
        local_state_atk_storage; % A container to store the local state of all vehicles

        trust_score_storage; % A container to store the local state of all vehicles
        
        attack_module; % A container to store the local state of all vehicles
        
        
        
    end
    
    methods
        function self = CenterCommunication(attack_module)
            self.vehicles = [];

            self.input_logs = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            
            self.global_state_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.local_state_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.trust_score_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');

            self.global_state_atk_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.local_state_atk_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            
            self.attack_module = attack_module;
        end
        
        function register_vehicle(self, vehicle)
            % Register a vehicle with the central communication hub
            self.vehicles = [self.vehicles; vehicle];

            self.input_logs(vehicle.vehicle_number) = vehicle.input_log;
            % raw
            self.global_state_storage(vehicle.vehicle_number) = vehicle.observer.est_global_state_current;
            self.local_state_storage(vehicle.vehicle_number) = vehicle.observer.est_local_state_current;
            % attack
            self.global_state_atk_storage(vehicle.vehicle_number) = vehicle.observer.est_global_state_current;
            self.local_state_atk_storage(vehicle.vehicle_number) = vehicle.observer.est_local_state_current;

            self.trust_score_storage(vehicle.vehicle_number) = vehicle.trust_log(1,1,:);
            
        end
        
        
        %% SETTER
        
        function update_trust(self, vehicle_number, trust_score)
            % Update the state of a specific vehicle
            self.trust_score_storage(vehicle_number) = trust_score;
        end
        

        
        %% TODO : Need add attack here
        function update_global_state(self, vehicle_number, global_state,instant_idx)

            % instant_idx is the index of the current time step
            [global_state_atk, global_state_raw] = self.attack_module.SetGlobalAttack(vehicle_number,global_state, instant_idx);

            % Update the global state of a specific vehicle
            self.global_state_storage(vehicle_number) = global_state_raw;
            self.global_state_atk_storage(vehicle_number) = global_state_atk;
            
        end
        
        function update_local_state(self, vehicle_number, local_state , instant_idx)
            
            [local_state_atk , local_state_raw] = self.attack_module.SetLocalAttack(vehicle_number , local_state, instant_idx);

            % Update the local state of a specific vehicle
            self.local_state_storage(vehicle_number) = local_state_raw;
            self.local_state_atk_storage(vehicle_number) = local_state_atk;
        end
        
        
        function update_input(self, vehicle_number, input)
            % Update the input of a specific vehicle
            self.input_logs(vehicle_number) = input;
        end
        
        %% GETTER
        %% TODO : can implement attack module here to simulate the attack
        
        function global_state = get_global_state(self, vehicle_number)
            % Get the global state of a specific vehicle
            global_state = self.global_state_atk_storage(vehicle_number);
        end
        
        function local_state = get_local_state(self, vehicle_number)
            % Get the local state of a specific vehicle
            
            local_state = self.local_state_atk_storage(vehicle_number);
            
        end
        
        function input = get_input(self, vehicle_number)
            % Get the input of a specific vehicle
            input = self.input_logs(vehicle_number);
        end
        
        function trust_score = get_trust_score(self, vehicle_number)
            % Get the trust score of a specific vehicle
            trust_score = self.trust_score_storage(vehicle_number);
        end
        
        %% Get all current state and input of all vehicles
        function states = get_all_states(self)
            % Get the states of all vehicles
            states = values(self.state_logs);
        end
        
        function inputs = get_all_inputs(self)
            % Get the inputs of all vehicles
            inputs = values(self.input_logs);
        end
    end
end