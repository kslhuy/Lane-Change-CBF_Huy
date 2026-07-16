classdef CenterCommunication < handle
    properties
        vehicles; % A container to store all vehicles in the platoon
        state_logs; % A container to store the state logs of all vehicles
        input_logs; % A container to store the input logs of all vehicles
        
        global_state_storage; % A container to store the global state of all vehicles
        global_state_atk_storage; % A container to store the global state of all vehicles
        global_state_timestamp_storage; % Source timestamp for each global-state packet



        local_state_storage; % A container to store the local state of all vehicles
        local_state_atk_storage; % A container to store the local state of all vehicles
        local_state_timestamp_storage; % Source timestamp for each local-state packet

        input_timestamp_storage; % Source timestamp for each input packet

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

            self.global_state_timestamp_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.local_state_timestamp_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.input_timestamp_storage = containers.Map('KeyType', 'int32', 'ValueType', 'any');

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

            % Registered values are the time-zero source samples.
            self.global_state_timestamp_storage(vehicle.vehicle_number) = 0;
            self.local_state_timestamp_storage(vehicle.vehicle_number) = 0;
            self.input_timestamp_storage(vehicle.vehicle_number) = 0;

            self.trust_score_storage(vehicle.vehicle_number) = vehicle.trust_log(1,1,:);
            
        end
        
        
        %% SETTER
        
        function update_trust(self, vehicle_number, trust_score)
            % Update the state of a specific vehicle
            self.trust_score_storage(vehicle_number) = trust_score;
        end
        

        
        %% TODO : Need add attack here
        function update_global_state(self, vehicle_number, global_state, instant_idx, source_timestamp)

            % instant_idx is the index of the current time step
            if nargin < 5 || isempty(source_timestamp)
                source_timestamp = instant_idx;
            end
            source_timestamp = self.validate_source_timestamp(source_timestamp);
            [global_state_atk, global_state_raw] = self.attack_module.SetGlobalAttack(vehicle_number,global_state, instant_idx);

            % Update the global state of a specific vehicle
            self.global_state_storage(vehicle_number) = global_state_raw;
            self.global_state_atk_storage(vehicle_number) = global_state_atk;
            self.global_state_timestamp_storage(vehicle_number) = source_timestamp;
            
        end
        
        function update_local_state(self, vehicle_number, local_state, instant_idx, source_timestamp)
            if nargin < 5 || isempty(source_timestamp)
                source_timestamp = instant_idx;
            end
            source_timestamp = self.validate_source_timestamp(source_timestamp);

            [local_state_atk , local_state_raw] = self.attack_module.SetLocalAttack(vehicle_number , local_state, instant_idx);

            % Update the local state of a specific vehicle
            self.local_state_storage(vehicle_number) = local_state_raw;
            self.local_state_atk_storage(vehicle_number) = local_state_atk;
            self.local_state_timestamp_storage(vehicle_number) = source_timestamp;
        end
        
        
        function update_input(self, vehicle_number, input, source_timestamp)
            % Update the input of a specific vehicle
            if nargin < 4 || isempty(source_timestamp)
                % Legacy send_data() publishes local/global state immediately
                % before the matching input. Reuse that packet timestamp so old
                % callers gain timestamp retention without an API change.
                source_timestamp = self.latest_source_timestamp(vehicle_number);
            end
            source_timestamp = self.validate_source_timestamp(source_timestamp);
            self.input_logs(vehicle_number) = input;
            self.input_timestamp_storage(vehicle_number) = source_timestamp;
        end
        
        %% GETTER
        %% TODO : can implement attack module here to simulate the attack
        
        function [global_state, source_timestamp] = get_global_state(self, vehicle_number,host_id)
            % Get the global state of a specific vehicle
            if(host_id == vehicle_number)
                global_state = self.global_state_storage(vehicle_number);
            else
                global_state = self.global_state_atk_storage(vehicle_number);
            end
            if nargout > 1
                source_timestamp = self.global_state_timestamp_storage(vehicle_number);
            end
        end
        
        function [local_state, source_timestamp] = get_local_state(self, vehicle_number,host_id)
            % Get the local state of a specific vehicle
            % if we are the host, we get the local state raw data
            if(host_id == vehicle_number)
                local_state = self.local_state_storage(vehicle_number);
            else
                local_state = self.local_state_atk_storage(vehicle_number);
            end
            if nargout > 1
                source_timestamp = self.local_state_timestamp_storage(vehicle_number);
            end
        end

        function [local_state, source_timestamp] = get_clean_local_state(self, vehicle_number)
            % Return the unattacked source sample for trust/anchor checks.
            local_state = self.local_state_storage(vehicle_number);
            if nargout > 1
                source_timestamp = self.local_state_timestamp_storage(vehicle_number);
            end
        end
        
        function [input, source_timestamp] = get_input(self, vehicle_number)
            % Get the input of a specific vehicle
            input = self.input_logs(vehicle_number);
            if nargout > 1
                source_timestamp = self.input_timestamp_storage(vehicle_number);
            end
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

        function source_timestamp = latest_source_timestamp(self, vehicle_number)
            % Prefer the local-state timestamp because send_data() publishes it
            % first, then fall back to global state or the registered time zero.
            if isKey(self.local_state_timestamp_storage, vehicle_number)
                source_timestamp = self.local_state_timestamp_storage(vehicle_number);
            elseif isKey(self.global_state_timestamp_storage, vehicle_number)
                source_timestamp = self.global_state_timestamp_storage(vehicle_number);
            else
                source_timestamp = 0;
            end
        end

        function source_timestamp = validate_source_timestamp(~, source_timestamp)
            if ~isnumeric(source_timestamp) || ~isscalar(source_timestamp) || ...
                    ~isreal(source_timestamp) || ~isfinite(source_timestamp)
                error('CenterCommunication:InvalidSourceTimestamp', ...
                    'Source timestamp must be a finite real numeric scalar.');
            end
        end
    end
end
