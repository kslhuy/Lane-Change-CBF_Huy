classdef Scenarios_config < handle
    properties
        dt ;
        simulation_time ;
        where ; % 1 indicates highway, 2 indicates urban road
        model_vehicle_type = "normal"; % "delay_v" , "delay_a" , "normal"

        lead_senario = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"
        
        lead_input = 0; % lead input
        debug_mode = false; % debug mode

        %%%% Observer related

        Use_predict_observer = true; % if use predict observer
        predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"
        Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"
        Is_noise_mesurement = false; % if the noise is in the mesurement
        noise_probability = 0.3; % Probability of adding measurement noise
        Use_smooth_filter = true; % if using smooth filtering for noise mesurement 
        Use_smooth_filter_in_local_observer = true; % if using smooth filtering in local observer

        Dichiret_type = "Single"; % "Single" , "Dual"
        Monitor_sudden_change = false; % if the sudden change is monitored
        use_local_data_from_other = true; % if the local data from other vehicles is used

        %%%% Attack related

        attacker_update_locally = true; % if the attacker is not updated from the others , only use local data


        %% ------------- Trust related
        using_weight_trust_observer = true; % if using weight trust
        Use_weight_local_trust = true; % if using weight trust for local data
        Use_weight_global_trust = true; % if using weight trust for global data
        
        is_know_data_not_nearby = true ; % just for test purpose, Use that we have better Trust score , meaning that we know the data all of the other vehicles
        
        % New validation controls
        Use_physical_constraints_check = false; % Enable/disable physical constraints validation
        Use_temporal_consistency_check = false; % Enable/disable temporal consistency evaluation 
        

        acceleration_trust_score_method = "vrel_dis_adjusted"; % 'mathematical' - Use exact mathematical formula from paper
                                        % 'enhanced' - Use enhanced implementation with reduced sensitivity
                                        % 'default' - Use default implementation
                                        % vrel_dis_adjusted - Use adjusted relative distance
                                        % 'vrel_dis_real' - Use real relative distance
                                        % 'hybrid' - Combine both methods
        opinion_type = "trust"; % opinion type " distance" , " trust" , " both"

        %%% Controller related
        gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean "
        controller_type = "local"; % "local" , "coop" , "mix"
        data_type_for_u2 = "true"; % "est" , "true"

        control_use_accel = false; % Will override distributed observer with a prediction model
        CACC_bidirectional = false; % If true, the CACC controller will consider both leading and following vehicles in the control law
    

        %% ---- PREDICTION in observer PARAMETERS
        MAX_PREDICT_ONLY_TIME = 5; % seconds
        N_good = 3; % Number of consecutive good steps to exit predict_only
        blend_thresh = 10; % You can tune this threshold

    end
    methods
        function self = Scenarios_config(dt , simulation_time , scenario_where  )
            self.dt = dt;
            self.simulation_time = simulation_time;
            self.where = scenario_where;

        end

        function [ulim,llim] = getLimitSpeed(self)

            if self.where == "Highway"
                ulim = 33.33;
                llim = 16.67;
            elseif self.where == "Urban"
                ulim = 16.67;
                llim = 12;
            end
        end

        function lane_width = getLaneWidth(self)
            if self.where == "Highway"
                lane_width = 3.6; % Highway lane width in meters
            elseif self.where == "Urban"
                lane_width = 3;
            end
        end

        function set_usecontrol_accel(self, control_use_accel)
            self.control_use_accel = control_use_accel;
        end

        function set_Trip_Dichiret(self, dichiret_type)
            self.Dichiret_type = dichiret_type;
        end

        function set_Lead_Senarios(self , lead_senario )
            self.lead_senario = lead_senario;
        end

        function Is_attacker_not_update(self , attacker_update_locally)
            self.attacker_update_locally = attacker_update_locally;
        end

        function lead_input = get_LeadInput(self , instant_index)
            if self.lead_senario == "constant"
                lead_input = 0;

                % time = self.dt * instant_index;
                % lead_input = 1*sin(4 * pi * time / 10) ; % Example sinusoidal input for testing

            else
                time = self.dt * instant_index;
                if time >= 10 && time < 15
                    if self.lead_senario == "Acceleration"
                        lead_input = 2;
                    elseif self.lead_senario == "Deceleration"
                        lead_input = -5;
                    elseif self.lead_senario == "Lane_change"
                        lead_input = 0;
                    end
                else
                    lead_input = 0;
                    % lead_input = 2*sin(2 * pi * time / 10) ; % Example sinusoidal input for testing
                end
            end

        % if (self.is_lead_input_change)
        %     time = self.dt * instant_index;
        %     if (time >= 0 && time < 5)
        %         lead_input = 0;
        %     elseif (time >= 5 && time < 9)
        %         lead_input = 2;
        %     elseif (time >= 10 && time < 15)
        %         lead_input = -5;
        %     else
        %         lead_input = 0;
        %     end
        % else
        %     lead_input = 0;
        % end
        end

        function set_CACC_bidirectional(self, CACC_bidirectional)
            self.CACC_bidirectional = CACC_bidirectional;
        end




        function set_Test_better_trust(self , is_know_data_not_nearby)
            self.is_know_data_not_nearby = is_know_data_not_nearby;
        end


        function set_Use_local_data_from_other(self , use_local_data_from_other)
            self.use_local_data_from_other = use_local_data_from_other;
        end

        function set_Use_predict_observer(self, use_predict_observer)
            self.Use_predict_observer = use_predict_observer;
        end

        function  set_predict_controller_type(self ,predict_controller_type )
            self.predict_controller_type = predict_controller_type;
        end

        function  set_Local_observer_type(self ,Local_observer_type )
            self.Local_observer_type = Local_observer_type;
        end
        function set_Is_noise_mesurement(self , Is_noise_mesurement)
            self.Is_noise_mesurement = Is_noise_mesurement;
        end

        function set_Use_smooth_filter(self, Use_smooth_filter)
            self.Use_smooth_filter = Use_smooth_filter;
        end

        function set_monitor_sudden_change(self, monitor_sudden_change)
            self.Monitor_sudden_change = monitor_sudden_change;
        end

        function set_parmeter_prediction_switch_observer(self, MAX_PREDICT_ONLY_TIME, N_good , blend_thresh)
            % Set parameters for prediction in observer
            self.MAX_PREDICT_ONLY_TIME = MAX_PREDICT_ONLY_TIME; % seconds
            self.N_good = N_good; % Number of consecutive good steps to exit predict_only
            self.blend_thresh = blend_thresh; % Blending threshold
        end
    end
end
