%% Function for plot
function every_plot( collected_car)
    nb_case_simulation = 2;
    nb_vehicles = size(collected_car , 2); 
    figure("Name", "Error global est " + num2str(nb_vehicles), "NumberTitle", "off");

    %% Plot error X position
    subplot(4,1,1);

    % vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:nb_vehicles, 'UniformOutput', false);

    % Compare 2 Vehicle 1 for the error of global state for position X
    plot( collected_car(1,1).observer.est_global_state_log(1, 1:end, i) - vehicles(i).state_log(1, 1:end-1), 'LineWidth', 1);
    plot( collected_car(2,1).observer.est_global_state_log(1, 1:end, i) - vehicles(i).state_log(1, 1:end-1), 'LineWidth', 1);
    hold on;


    for i = 1:nb_vehicles
        plot( self.observer.est_global_state_log(1, 1:end, i) - vehicles(i).state_log(1, 1:end-1), 'LineWidth', 1);
        plot( self.observer.est_global_state_log(1, 1:end, i) - vehicles(i).state_log(1, 1:end-1), 'LineWidth', 1);
        hold on;
    end

    % % Assuming Is_ok is a logical array with the same time steps as the logs
    % is_not_ok = ~self.observer.Is_ok_log;
    % bad_time_steps = find(is_not_ok); % Indices where Is_ok is true

    legend(vehicle_labels);

    title(['Comparison global est state with local true platoon car ' num2str(self.vehicle_number)]);

    % % Add vertical lines where Is_ok is false
    % for t = bad_time_steps
    %     xline(t, '--r', 'LineWidth', 1.2, 'Alpha', 0.3); % Red dashed line
    % end


    %% Plot error Y position 
    subplot(4,1,2);

    for i = 1:nb_vehicles
        plot( self.observer.est_global_state_log(2, 1:end, i) - vehicles(i).state_log(2, 1:end-1), 'LineWidth', 1);
        hold on;
    end
    % legend('error Y');

    %% Plot error Theta angle 
    subplot(4,1,3);

    for i = 1:nb_vehicles
        plot( self.observer.est_global_state_log(3, 1:end, i) - vehicles(i).state_log(3, 1:end-1), 'LineWidth', 1);
        hold on;
    end

    %% Plot error velocity angle
    subplot(4,1,4);

    for i = 1:nb_vehicles
        plot( self.observer.est_global_state_log(4, 1:end, i) - vehicles(i).state_log(4, 1:end-1), 'LineWidth', 1);
        hold on;
    end

    xlabel('Time (s)');
end



%%% Plotting Functions
function plot_error_global_state_log(global_state_log_1, global_state_log_2)


    
    
    % Ensure est_global_state_log is not empty
    if isempty(global_state_log_1) || isempty(global_state_log_2)
        error('Global state logs are empty. No data to plot.');
    end


    num_vehicles = size(global_state_log_1, 3); % Number of vehicles
    % num_states = 4; % Assuming X, Y, theta, and V as states
    num_states = size(global_state_log_1, 1); % Number of states

    state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};

    figure("Name", "Global Position Estimates " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
    % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

    for state_idx = 1:num_states
        subplot(num_states, 1, state_idx);
        hold on;
        for v = 1:num_vehicles
            plot(squeeze(global_state_log_1(state_idx, 1:end-1, v) - global_state_log_2(state_idx, 1:end-1, v)), 'DisplayName', ['Vehicle ', num2str(v)]);
        end
        title(state_labels{state_idx});
        % xlabel('Time (s)');
        ylabel(state_labels{state_idx});
        legend;
        grid on;
    end
end


%% Function for plot
function plot_ground_error_global_est(self , collected_car)
    figure("Name", "Error global est " + num2str(self.vehicle_number), "NumberTitle", "off");
    nb_vehicles = length(collected_car);
    state_labels = {'Error Position X', 'Error Velocity','Error Acc'};
    num_states = size(self.observer.est_global_state_log, 1); % Number of states

    for state_idx = [1, 4, 5] % Assuming we want to plot only X, V, and Acc
        for v_subplot = 1:nb_vehicles
            vehicles = collected_car(:, state_idx);
            subplot(num_states, v_subplot, state_idx);
            for v_idx = 1:nb_vehicles
                plot( vehicles(v_idx).observer.est_global_state_log(state_idx, 1:end, v_idx) - vehicles(v_idx).state_log(state_idx, 1:end-1), 'LineWidth', 1 , 'DisplayName', ['Vehicle ', num2str(v_idx)]);
                hold on;
                % plot(self.observer.est_global_state_log(state_idx, 1:end, v), 'DisplayName', ['Vehicle ', num2str(v)]);
            end
        end
        
        title(state_labels{state_idx});
        ylabel(state_labels{state_idx});
        legend;
        grid on;
    end

    xlabel('Time (s)');
end


