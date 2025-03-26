classdef Simulator
    properties
        lanes; % Lane configuration for the simulation
        ego_vehicle; % The ego vehicle in the simulation
        other_vehicles; % List of other vehicles in the simulation
        dt; % Time step for the simulation
        show_animation;
    end
    methods
        % Constructor to initialize the Simulator object
        function self = Simulator(lanes, ego_vehicle, other_vehicles, dt,show_animation)
            self.lanes = lanes; % Set lanes
            self.ego_vehicle = ego_vehicle; % Set ego vehicle
            self.other_vehicles = other_vehicles; % Set other vehicles
            self.dt = dt; % Set time step
            self.show_animation = show_animation;
        end

        % Method to start the simulation
        function [state_log, input_log] = startSimulation(self, simulation_time)
            num = simulation_time / self.dt; % Calculate number of simulation steps
            plot_interval = 5; % Plot every 20 iterations

            % Set up the figure once
            figure(1);
            set(gcf, 'Position', [200, 100, 720, 600]); % Larger figure window

            % grid on;
            % xlabel('X Position (m)', 'FontSize', 12);
            % ylabel('Y Position (m)', 'FontSize', 12);


            % % Set the x and y axis limits
            % xlim([0, 300]); % Set x-axis limits
            % ylim([-30, 30]); % Adjusted for better visibility

            % Start the timer
            tic;

            for i = 1:num
                % Update other vehicles' positions
                num_car = size(self.other_vehicles, 1);
                if num_car >= 1
                    for k = 1:num_car
                        self.other_vehicles(k).update(i); % Update vehicle state
                    end
                    % for k = 1:num_car
                    %     self.other_vehicles(k).send_data(i); % Update vehicle state
                    % end
                    
                end

                if ~isempty(self.ego_vehicle)
                    % Update ego vehicle's position
                    self.ego_vehicle.update; % Update ego vehicle state

                end

                if (self.show_animation)
                    % Plot at specified intervals
                    if (mod(i, plot_interval) == 0)
                        clf; % Clear the current figure
    
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
    
    
                        % Set the x and y axis limits
                        % xlim([0, 500]); % Set x-axis limits
                        % % Adjust xlim dynamically to follow the ego vehicle


                        offset_x = self.other_vehicles(1).state(1); % Assuming state(1) is the X position
                        xlim([offset_x  - 200, offset_x + 100]); % Keep the ego vehicle in the center
                

                        ylim([-30, 30]); % Adjusted for better visibility
    
                        grid on;
                        % Add grid and labels for better visualization
                        xlabel('X Position (m)', 'FontSize', 14);
                        ylabel('Y Position (m)', 'FontSize', 14);
    
                        % Update the plot
                        drawnow; % Ensures smooth rendering of the animation
                    end
    
                    % Synchronize with real-time
                    elapsed_time = toc
                    expected_time = i * self.dt;
                    if elapsed_time < expected_time
                        % disp("pause")
                        pause(expected_time - elapsed_time - self.dt);
                    end
        
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

        %% Function for plot

        function [] = plot_movement_log(self,vehicles, scenarios_config, num_vehicles)
            figure;

            % Define vehicle labels for the legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:num_vehicles, 'UniformOutput', false);

            % Plot position X history
            subplot(6, 1, 1)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt :scenarios_config.simulation_time, vehicles(i).state_log(1, :),"LineWidth",1)
            end
            title('Position X history')
            ylabel('m')
            % xlabel('s')
            % legend(vehicle_labels)
            hold off

            % Plot position Y history
            subplot(6, 1, 2)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(2, :),"LineWidth",1)
            end
            title('Position Y history')
            ylabel('m')
            % xlabel('s')
            % legend(vehicle_labels)
            hold off

            % Plot velocity history
            subplot(6, 1, 3)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(4, :),"LineWidth",1)
            end
            title('Velocity history')
            ylabel('m/s')
            % xlabel('s')
            if scenarios_config.where == "Highway"
                ylim([14, 36])
            elseif scenarios_config.where == "Urban"
                ylim([7, 17])
            end
            legend(vehicle_labels)
            hold off


            % Plot yaw history
            subplot(6, 1, 4)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(3, :),"LineWidth",1)
            end
            title('Yaw history')
            ylabel('rad')
            % xlabel('s')
            ylim([-0.3, 0.3])
            legend(vehicle_labels)
            hold off



            % Plot steering history
            subplot(6, 1, 5)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).input_log(1, :),"LineWidth",1)
            end
            title('Control acc history')
            ylabel('m/s^2')
            % xlabel('s')
            % ylim([-2, 2])
            % legend(vehicle_labels)
            hold off

            % Plot steering history
            subplot(6, 1, 6)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).input_log(2, :),"LineWidth",1)
            end
            title('Steering history')
            ylabel('rad')
            xlabel('s')
            ylim([-0.08, 0.08])
            % legend(vehicle_labels)


            hold off
        end

        %% Function for plotting relative position and speed history
        function [] = plot_relative_movement_log(self, vehicles, scenarios_config, num_vehicles)
            figure;
            title('Real relative State');

            % Define vehicle labels for legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d - Vehicle %d', i-1 ,i ), 2:num_vehicles, 'UniformOutput', false);

            % Plot relative position X history
            subplot(3, 1, 1)
            hold on
            for i = 2:num_vehicles
                rel_x = vehicles(i-1).state_log(1, :) - vehicles(i).state_log(1, :) ;
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_x,"LineWidth",1)
            end
            title('Relative Position X history')
            ylabel('m')
            legend(vehicle_labels)
            hold off

            % Plot relative position Y history
            subplot(3, 1, 2)
            hold on
            for i = 2:num_vehicles
                rel_y = vehicles(i).state_log(2, :) - vehicles(i-1).state_log(2, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_y,"LineWidth",1)
            end
            title('Relative Position Y history')
            ylabel('m')
            legend(vehicle_labels)
            hold off

            % Plot relative speed history
            subplot(3, 1, 3)
            hold on
            for i = 2:num_vehicles
                rel_speed = vehicles(i).state_log(4, :) - vehicles(i-1).state_log(4, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_speed,"LineWidth",1)
            end
            title('Relative Speed history')
            ylabel('m/s')
            xlabel('s')
            legend(vehicle_labels)
            hold off
        end

    end
end