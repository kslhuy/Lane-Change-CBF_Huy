classdef Simulator
    properties
        lanes; % Lane configuration for the simulation
        ego_vehicle; % The ego vehicle in the simulation
        other_vehicles; % List of other vehicles in the simulation
        dt; % Time step for the simulation
    end
    methods
        % Constructor to initialize the Simulator object
        function self = Simulator(lanes, ego_vehicle, other_vehicles, dt)
            self.lanes = lanes; % Set lanes
            self.ego_vehicle = ego_vehicle; % Set ego vehicle
            self.other_vehicles = other_vehicles; % Set other vehicles
            self.dt = dt; % Set time step
        end

        % Method to start the simulation
        function [state_log, input_log] = startSimulation(self, simulation_time)
            num = simulation_time / self.dt; % Calculate number of simulation steps
            plot_interval = 5; % Plot every 20 iterations

            % Set up the figure once
            figure(1);
            set(gcf, 'Position', [200, 100, 1000, 1000]); % Larger figure window

            grid on;
            xlabel('X Position (m)', 'FontSize', 12);
            ylabel('Y Position (m)', 'FontSize', 12);


            % Set the x and y axis limits
            xlim([0, 500]); % Set x-axis limits
            ylim([-15, 15]); % Adjusted for better visibility
            % Start the timer
            tic;

            for i = 1:num
                % Update other vehicles' positions
                num_car = size(self.other_vehicles, 1);
                if num_car >= 1
                    for k = 1:num_car
                        self.other_vehicles(k).update; % Update vehicle state
                    end
                end

                if ~isempty(self.ego_vehicle)
                    % Update ego vehicle's position
                    self.ego_vehicle.update; % Update ego vehicle state
                
                end


                % Plot at specified intervals
                if mod(i, plot_interval) == 0
                    clf; % Clear the current figure
                    set(gcf, 'Position', [100, 100, 1200, 800]); % Larger figure window

                    self.lanes.plot_lanes; % Plot lanes
                    hold on; % Hold the plot for adding more elements

                    % Plot other vehicles
                    if num_car >= 1
                        for k = 1:num_car
                            self.other_vehicles(k).plot_vehicle; % Plot vehicles
                        end
                    end
                    
                    if ~isempty(self.ego_vehicle)
                        self.ego_vehicle.plot_vehicle; % Plot ego vehicle
                    end
                    

                    % % Set the x and y axis limits
                    xlim([0, 500]); % Set x-axis limits
                    ylim([-15, 15]); % Adjusted for better visibility

                    grid on;
                    % Add grid and labels for better visualization
                    xlabel('X Position (m)', 'FontSize', 12);
                    ylabel('Y Position (m)', 'FontSize', 12);

                    % Update the plot
                    drawnow; % Ensures smooth rendering of the animation
                end

                % Synchronize with real-time
                elapsed_time = toc;
                expected_time = i * self.dt;
                if elapsed_time < expected_time
                    % disp("pause")
                    pause(expected_time - elapsed_time - self.dt);
                end
            end
            % Log the state and input history of the ego vehicle
            if ~isempty(self.ego_vehicle)
                state_log = self.ego_vehicle.state_log; % Update states history
                input_log = self.ego_vehicle.input_log; % Update inputs history
            else
                state_log = [];
                input_log = [];
            end
        end
    end
end