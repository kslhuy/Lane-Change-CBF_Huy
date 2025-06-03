classdef Scenarios_config < handle
    properties
        dt ;
        simulation_time ;
        where ; % 1 indicates highway, 2 indicates urban road
        gamma_type; % type gamma for switching control = " min" , " max " , " mean "
        opinion_type; % opinion type " distance" , " trust" , " both"
        controller_type; % "local" , "coop" , "mix"
        model_vehicle_type = "normal"; % "delay_v" , "delay_a" , "normal"
        data_type_for_u2 = "est"; % "est" , "true"

        lead_senario; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"

        lead_input = 0; % lead input
        debug_mode = false; % debug mode
        Use_predict_observer = false; % if use predict observer
        predict_controller_type = "self"; % "self" , "true_other" , "predict_other"
        Local_observer_type = "mesurement"; % "mesurement" , "kalman" , "observer"
        Is_noise_mesurement = false; % if the noise is in the mesurement
        Dichiret_type = "Single"; % "Single" , "Dual"
        Monitor_sudden_change = false; % if the sudden change is monitored
        use_local_data_from_other = false; % if the local data from other vehicles is used

        attacker_update_locally = true; % if the attacker is not updated from the others , only use local data
    end
    methods
        function self = Scenarios_config(dt , simulation_time , scenario_where , controller_type ,data_type_for_u2, gamma_type , opinion_type, model_vehicle_type, debug_mode )
            self.dt = dt;
            self.simulation_time = simulation_time;
            self.where = scenario_where;

            %% Control
            self.controller_type = controller_type; % "local" , "coop" , "mix"
            self.data_type_for_u2 = data_type_for_u2; % "est" , "true"
            % type gamma for switching control = " min" , " max " , " mean "
            self.gamma_type =  gamma_type;

            % opinion type " distance" , " neighbor" , " both"
            self.opinion_type = opinion_type;

            self.model_vehicle_type = model_vehicle_type; % "delay_v" , "delay_a" , "normal"

            self.debug_mode = debug_mode;
            % self.Dichiret_type = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"
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

        function set_monitor_sudden_change(self, monitor_sudden_change)
            self.Monitor_sudden_change = monitor_sudden_change;
        end
    end
end
