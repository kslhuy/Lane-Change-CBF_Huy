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
