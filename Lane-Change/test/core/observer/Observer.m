classdef Observer < handle
    properties
        type; % type of the controller,
        param_sys; % parameter for vehicle
        straightlane;

        est_local_state_current;
        est_global_state_current;

        est_global_state_log;
        est_local_state_log;
        vehicle;
        
    end
    methods
        function self = Observer( vehicle , veh_param, inital_global_state, inital_local_state)
            self.vehicle = vehicle;
            self.param_sys = veh_param; % Set the vehicle system parameters

            self.est_local_state_current = inital_local_state;
            
            self.est_global_state_current = inital_global_state;
            
            num_states = 4 ; % X , Y , theta , V
            Nt = self.vehicle.total_time_step;
            num_vehicles = length(self.vehicle.other_vehicles);

            self.est_global_state_log = zeros(num_states, Nt, num_vehicles);
            self.est_global_state_log(:,1,:) = inital_global_state;

            self.est_local_state_log = inital_local_state;


        end
        
        
        function  Distributed_Observer(self,instant_index , weights)
            % Get the number of other vehicles
            num_vehicles = length(self.vehicle.other_vehicles);

            Big_X_hat_1_tempo = zeros(size(self.est_global_state_current)); % Initialize the variable to store the results
            x_hat_i_j = zeros(4, num_vehicles); % Initialize the variable to store the global state of other vehicles

            for j = 1:num_vehicles
                if(self.vehicle.trust_log(1, instant_index, j) < 0.5)
                    weights(j) = 0;
                end

                %% Get local of j vehicle
                x_bar_j = self.vehicle.center_communication.get_local_state(j);
                
                % ------- To get global state of other vehicles 
                % Go through all the other vehicles , start from the second vehicle
                for k = 1:num_vehicles
                    % Go in car number "k" , Get car "j" in global estimat of "k" 
                    x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k);
                    x_hat_i_j(:,k) =  x_hat_i_j_full(:,j);
                end
                % ------- To get global state of other vehicles 
                
                % Hiện tại đang tính thằng j , so vào thằng j lấy control input của nó
                % TODO : Since vehicle 1 dont have connection with vehicle 4 , so we need to remove the last control input
                % Need to calculate the control input for each vehicle locally by the estimated state of other vehicle
                
                % u_j = self.vehicle.other_vehicles(j).input; % Control input of the current vehicle
                u_j = self.vehicle.input; % Control input of the current vehicle


                
                output = distributed_Observer_each( self , j , x_bar_j , x_hat_i_j, u_j, weights );
                Big_X_hat_1_tempo(:,j) = output; % Append the result
            end
            self.est_global_state_current = Big_X_hat_1_tempo;
            self.est_global_state_log(:, instant_index, :) = Big_X_hat_1_tempo;
        end
        
        function output = distributed_Observer_each( self , j , x_bar_j , x_hat_i_j, u_j ,weights )
            

            Sig = zeros(4,1); % Initialize consensus term
            [A , B]  = self.matrix();
            % Calculate the consensus term
            % Start from 2 , because the first element is the local state of the vehicle
            for L = 2:length(weights)
                % Why x_hat_i_j(: , l-1) because L start from 2 , but we need to start from 1 for vehicle
                Sig = weights(L)*( x_hat_i_j(: , L-1) - x_hat_i_j(: , self.vehicle.vehicle_number) ) + Sig;
            end
            %% Some situation
            % Only if we try to estimated j = i , so we have real local state that was estimated by Local observer .
            % But if we try to estimate j != i , we have (fake) the local state of j , that by vehicle j send to i vehicle

            % Estimate the j vehicle , in i vehicle
            % So the observer is implement in i vehicle
            w_i0 = weights(j); % Weight of the local state

            %% Original paper (TRUE WORK)
            output = A*( x_hat_i_j(: , j) + Sig + w_i0 * (x_bar_j - x_hat_i_j(: , j)) ) + B*u_j ;

            
        end
        



        function Local_observer(self , state)
            self.est_local_state_current = state;
        end

        % This model use in obesrver , so need to be here, in the observer class
        function  [A , B] = matrix(self)
            theta = self.vehicle.state(3);
            Ts = self.param_sys.dt;
            v = self.vehicle.state(4);
            % Here in continuous time
            % A = [ 0 0 0 cos(theta);
            %      0 0 0 sin(theta);
            %      0 0 1 0;
            %      0 0 0 0];

            % B = [ 0 0;
            %       0 0;
            %       0 0;
            %       1 0];

            % Here in discrete time

            A = [ 1 0 -v*sin(theta)*Ts cos(theta)*Ts;
                  0 1 v*cos(theta)*Ts sin(theta)*Ts;
                  0 0 1 0;
                  0 0 0 1];

            B = [0 0
                 0 0
                 0 Ts
                 Ts 0];
        

        end

        function plot_global_state_log(self)
            % Ensure est_global_state_log is not empty
            if isempty(self.est_global_state_log)
                error('est_global_state_log is empty. No data to plot.');
            end
        
            num_vehicles = size(self.est_global_state_log, 3); % Number of vehicles
            num_states = 4; % Assuming X, Y, theta, and V as states
        
            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity'};
        
            figure("Name", "Global Position Estimates " );
            % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

            for state_idx = 1:num_states
                subplot(4, 1, state_idx);
                hold on;
                for v = 1:num_vehicles
                    plot(squeeze(self.est_global_state_log(state_idx, 1:end-1, v)), 'DisplayName', ['Vehicle ', num2str(v)]);
                end
                title(state_labels{state_idx});
                xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end


        
    end
end
