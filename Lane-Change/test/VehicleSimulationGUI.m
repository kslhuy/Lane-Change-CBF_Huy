classdef VehicleSimulationGUI < matlab.apps.AppBase
    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                matlab.ui.Figure
        SimulationPanel         matlab.ui.container.Panel
        NumVehiclesSpinner      matlab.ui.control.Spinner
        NumVehiclesLabel        matlab.ui.control.Label
        ScenarioDropDown        matlab.ui.control.DropDown
        ScenarioLabel           matlab.ui.control.Label
        StartButton             matlab.ui.control.Button
        PlotResultsButton       matlab.ui.control.Button
        
        AttackConfigPanel       matlab.ui.container.Panel
        TargetVehicleSpinner    matlab.ui.control.Spinner
        TargetVehicleLabel      matlab.ui.control.Label
        StartTimeEdit           matlab.ui.control.NumericEditField
        StartTimeLabel          matlab.ui.control.Label
        EndTimeEdit             matlab.ui.control.NumericEditField
        EndTimeLabel            matlab.ui.control.Label
        AttackTypeDropDown      matlab.ui.control.DropDown
        AttackTypeLabel         matlab.ui.control.Label
        IntensityEdit           matlab.ui.control.NumericEditField
        IntensityLabel          matlab.ui.control.Label
        DataTypeDropDown        matlab.ui.control.DropDown
        DataTypeLabel           matlab.ui.control.Label
        AttackRowDropDown       matlab.ui.control.DropDown
        AttackRowLabel          matlab.ui.control.Label
        
        VehicleConfigPanel      matlab.ui.container.Panel
        VehicleGrid             matlab.ui.container.GridLayout
    end
    
    properties (Access = private)
        Vehicles            % Array of vehicle objects
        Simulator           % Simulator object
        StateLog            % Simulation state log
        InputLog            % Simulation input log
        ScenariosConfig     % Scenarios configuration
    end
    
    methods (Access = private)
        function setupVehicleControls(app)
            % Clear existing controls
            delete(app.VehicleGrid.Children)
            
            % Create controls for each vehicle
            numVehicles = app.NumVehiclesSpinner.Value;
            app.VehicleGrid.RowHeight = repmat({30}, 1, numVehicles + 1);
            app.VehicleGrid.ColumnWidth = {'1x', '1x', '1x', '1x'};
            
            % Header row
            % Position items explicitly in the grid layout using Layout.Row and Layout.Column
            header1 = uilabel(app.VehicleGrid, 'Text', 'Vehicle ID', 'FontWeight', 'bold');
            header1.Layout.Row = 1;
            header1.Layout.Column = 1;
            
            header2 = uilabel(app.VehicleGrid, 'Text', 'Control Type', 'FontWeight', 'bold');
            header2.Layout.Row = 1;
            header2.Layout.Column = 2;
            
            header3 = uilabel(app.VehicleGrid, 'Text', 'Initial X', 'FontWeight', 'bold');
            header3.Layout.Row = 1;
            header3.Layout.Column = 3;
            
            header4 = uilabel(app.VehicleGrid, 'Text', 'Initial Speed', 'FontWeight', 'bold');
            header4.Layout.Row = 1;
            header4.Layout.Column = 4;
            
            % Vehicle rows
            for i = 1:numVehicles
                % Vehicle ID
                label = uilabel(app.VehicleGrid, 'Text', sprintf('Car %d', i));
                label.Layout.Row = i + 1;
                label.Layout.Column = 1;
                
                % Control Type dropdown
                dd = uidropdown(app.VehicleGrid, ...
                    'Items', {'None', 'IDM'}, ...
                    'Value', 'IDM', ...
                    'Tag', sprintf('ControlType%d', i));
                dd.Layout.Row = i + 1;
                dd.Layout.Column = 2;
                
                % Initial X position
                ef1 = uieditfield(app.VehicleGrid, 'numeric', ...
                    'Value', 80 - (i-1)*20, ...
                    'Tag', sprintf('InitX%d', i));
                ef1.Layout.Row = i + 1;
                ef1.Layout.Column = 3;
                
                % Initial Speed
                ef2 = uieditfield(app.VehicleGrid, 'numeric', ...
                    'Value', 26 - (i-1)*3, ...
                    'Tag', sprintf('InitSpeed%d', i));
                ef2.Layout.Row = i + 1;
                ef2.Layout.Column = 4;
            end
        end
        
        function initializeSimulation(app)
            % Initialize scenario configuration
            app.ScenariosConfig = Scenarios_config(0.01, 15, app.ScenarioDropDown.Value);
            
            % Setup lanes
            lane_width = app.ScenariosConfig.getLaneWidth();
            straightLanes = StraightLane(3, lane_width, 750);
            
            % Create vehicles
            numVehicles = app.NumVehiclesSpinner.Value;
            app.Vehicles = [];
            param_sys = ParamVeh();
            initial_lane_id = 1;
            direction_flag = 0;
            
            % Setup adjacency matrix
            graph = ones(numVehicles) - eye(numVehicles);
            weightTrust = Weight_Trust_module(graph, 0.5, 1);
            
            for i = 1:numVehicles
                controlDropdown = findobj(app.UIFigure, 'Tag', sprintf('ControlType%d', i));
                if ~isempty(controlDropdown) && isprop(controlDropdown, 'Value')
                    controlType = controlDropdown.Value;
                else
                    error('Dropdown for ControlType%d not found or has no Value property.', i);
                end
                initXField = findobj(app.UIFigure, 'Tag', sprintf('InitX%d', i));
                if ~isempty(initXField) && isprop(initXField, 'Value')
                    initX = initXField.Value;
                else
                    error('Edit field for InitX%d not found or has no Value property.', i);
                end
                
                initSpeedField = findobj(app.UIFigure, 'Tag', sprintf('InitSpeed%d', i));
                if ~isempty(initSpeedField) && isprop(initSpeedField, 'Value')
                    initSpeed = initSpeedField.Value;
                else
                    error('Edit field for InitSpeed%d not found or has no Value property.', i);
                end
                
                newVehicle = Vehicle(i, controlType, param_sys, ...
                    [initX; 0.5 * lane_width; 0; initSpeed], ...
                    initial_lane_id, straightLanes, direction_flag, 0, ...
                    app.ScenariosConfig, weightTrust);
                app.Vehicles = [app.Vehicles; newVehicle];
            end
            
            % Setup attack scenario
            attack_module = Attack_module(app.ScenariosConfig.dt);
            scenario_Attack_params = struct(...
                'target_vehicle_id', app.TargetVehicleSpinner.Value, ...
                'start_time', app.StartTimeEdit.Value, ...
                'end_time', app.EndTimeEdit.Value, ...
                'attack_type', app.AttackTypeDropDown.Value, ...
                'fault_intensity', app.IntensityEdit.Value, ...
                'data_type', app.DataTypeDropDown.Value, ...
                'attack_row', app.AttackRowDropDown.Value);
            
            attack_module.setScenario('time_based', scenario_Attack_params);
            center_comm = CenterCommunication(attack_module);
            
            % Assign neighbors
            for i = 1:numVehicles
                app.Vehicles(i).assign_neighbor_vehicle(app.Vehicles, [], center_comm, graph);
            end
            
            % Initialize simulator
            app.Simulator = Simulator(straightLanes, [], app.Vehicles, app.ScenariosConfig.dt);
        end
    end
    
    % Callbacks
    methods (Access = private)
        function NumVehiclesSpinnerValueChanged(app)
            app.setupVehicleControls();
        end
        
        function StartButtonPushed(app)
            app.initializeSimulation();
            [app.StateLog, app.InputLog] = app.Simulator.startSimulation(app.ScenariosConfig.simulation_time);
            app.PlotResultsButton.Enable = 'on';
        end
        
        function PlotResultsButtonPushed(app, event)
            % Plot movement
            figure;
            app.Simulator.plot_movement_log(app.Vehicles, app.ScenariosConfig, length(app.Vehicles));
            
            % Plot trust logs
            figure;
            for i = 1:length(app.Vehicles)
                subplot(length(app.Vehicles), 1, i);
                app.Vehicles(i).plot_trust_log();
                title(sprintf('Vehicle %d Trust Log', i));
            end
            
            % Plot observer states for vehicle 2
            figure;
            app.Vehicles(2).observer.plot_global_state_log();
            
            figure;
            app.Vehicles(2).plot_ground_truth_vs_estimated();
        end
    end
    
    % App creation and deletion
    methods (Access = public)
        function app = VehicleSimulationGUI
            % Create UIFigure and components
            app.UIFigure = uifigure('Name', 'Vehicle Simulation GUI');
            app.UIFigure.Position = [100 100 800 600];
            
            % Simulation Panel
            app.SimulationPanel = uipanel(app.UIFigure, 'Title', 'Simulation Settings', ...
                'Position', [20 400 760 180]);
            
            app.NumVehiclesLabel = uilabel(app.SimulationPanel, 'Text', 'Number of Vehicles:', ...
                'Position', [20 120 120 22]);
            app.NumVehiclesSpinner = uispinner(app.SimulationPanel, ...
                'Limits', [1 10], 'Value', 4, ...
                'Position', [150 120 60 22], ...
                'ValueChangedFcn', @(src, event)app.NumVehiclesSpinnerValueChanged());
            
            app.ScenarioLabel = uilabel(app.SimulationPanel, 'Text', 'Scenario:', ...
                'Position', [20 80 60 22]);
            app.ScenarioDropDown = uidropdown(app.SimulationPanel, ...
                'Items', {'Highway', 'Urban'}, 'Value', 'Highway', ...
                'Position', [150 80 100 22]);
            
            app.StartButton = uibutton(app.SimulationPanel, 'push', ...
                'Text', 'Start Simulation', ...
                'Position', [20 20 120 30], ...
                'ButtonPushedFcn', @(src, event) app.StartButtonPushed());
            
            app.PlotResultsButton = uibutton(app.SimulationPanel, 'push', ...
                'Text', 'Plot Results', ...
                'Position', [150 20 120 30], ...
                'Enable', 'off', ...
                'ButtonPushedFcn', @(src, event) app.PlotResultsButtonPushed(src, event));
            % Attack Config Panel
            app.AttackConfigPanel = uipanel(app.UIFigure, 'Title', 'Attack Configuration', ...
                'Position', [20 20 380 360]);
            
            % Attack controls
            controls = {
                {'TargetVehicleLabel', 'Target Vehicle:', [20 300], 'TargetVehicleSpinner', [150 300], 1, [1 10]};
                {'StartTimeLabel', 'Start Time:', [20 260], 'StartTimeEdit', [150 260], 5, [0 15]};
                {'EndTimeLabel', 'End Time:', [20 220], 'EndTimeEdit', [150 220], 10, [0 15]};
                {'IntensityLabel', 'Intensity:', [20 180], 'IntensityEdit', [150 180], 0.5, [-10 10]};
                {'AttackTypeLabel', 'Attack Type:', [20 140], 'AttackTypeDropDown', [150 140], 'bias', {'bias'}};
                {'DataTypeLabel', 'Data Type:', [20 100], 'DataTypeDropDown', [150 100], 'local', {'local'}};
                {'AttackRowLabel', 'Attack Row:', [20 60], 'AttackRowDropDown', [150 60], 'velocity', {'velocity', 'X', 'Y', 'all'}}
                };
            
            for i = 1:length(controls)
                c = controls{i};
                uilabel(app.AttackConfigPanel, 'Text', c{2}, 'Position', [c{3} 100 22]);
                if contains(c{4}, 'Spinner')
                    uispinner(app.AttackConfigPanel, 'Value', c{6}, 'Limits', c{7}, ...
                        'Position', [c{5} 60 22]);
                elseif contains(c{4}, 'Edit')
                    uieditfield(app.AttackConfigPanel, 'numeric', 'Value', c{6}, 'Limits', c{7}, ...
                        'Position', [c{5} 60 22]);
                else
                    uidropdown(app.AttackConfigPanel, 'Items', c{7}, 'Value', c{6}, ...
                        'Position', [c{5} 100 22]);
                end
            end
            
            % Vehicle Config Panel
            app.VehicleConfigPanel = uipanel(app.UIFigure, 'Title', 'Vehicle Configuration', ...
                'Position', [420 20 360 360]);
            app.VehicleGrid = uigridlayout(app.VehicleConfigPanel, ...
                'ColumnWidth', {'1x', '1x', '1x', '1x'}, ...
                'RowHeight', repmat({30}, 1, 5));
            
            % Initial setup
            app.setupVehicleControls();
        end
    end
    
    % App launcher
    methods (Static)
        function run()
            app = VehicleSimulationGUI;
        end
    end
end