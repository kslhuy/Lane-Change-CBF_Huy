classdef CenterCommunication < handle
    properties
        global_state_storage; % Stores global states of all vehicles
        local_state_storage; % Stores local states of all vehicles
    end
    
    methods
        function self = CenterCommunication(num_vehicles, num_states, Nt)
            self.global_state_storage = zeros(num_states, Nt, num_vehicles);
        end
        
        function update_global_state(self, vehicle_id, global_state, instant_index)
            % Update the global state for a specific vehicle at a specific time index
            self.global_state_storage(:, instant_index, vehicle_id) = global_state;
        end
        
        function global_state = get_global_states(self, requester_id)
            % Return the global states of all vehicles for a given requester
            global_state = self.global_state_storage(:, :, :);
        end
    end
end
