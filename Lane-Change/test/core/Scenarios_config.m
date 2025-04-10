classdef Scenarios_config < handle
    properties
        dt ;
        simulation_time ;
        where ; % 1 indicates highway, 2 indicates urban road
        gamma_type; % type gamma for switching control = " min" , " max " , " mean "
        opinion_type; % opinion type " distance" , " trust" , " both"
        controller_type; % "local" , "coop" , "mix"
        model_vehicle_type; % "delay_v" , "delay_a" , "normal"
        data_type_for_u2; % "est" , "true"

        is_lead_input_change;

        lead_input = 0; % lead input
        debug_mode = false; % debug mode
        Use_predict_observer = false; % if use predict observer
        predict_controller_type = "self"; % "self" , "true_other" , "predict_other"
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
        end

        function [ulim,llim] = getLimitSpeed(self)

            if self.where == "Highway"
                ulim = 33.33;
                llim = 23;
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

        
        function set_LeadInput_change(self , is_lead_input_change)
            self.is_lead_input_change = is_lead_input_change;
        end

        function lead_input = get_LeadInput(self , instant_index)
            if (self.is_lead_input_change)
                time = self.dt * instant_index;
                if (time >= 0 && time < 5)
                    lead_input = 0;
                elseif (time >= 5 && time < 9)
                    lead_input = 2;
                elseif (time >= 10 && time < 15)
                    lead_input = -5;
                else
                    lead_input = 0;
                end
            else
                lead_input = 0;
            end

        end



        function set_Use_predict_observer(self, use_predict_observer)
            self.Use_predict_observer = use_predict_observer;
        end
        
        function  set_predict_controller_type(self ,predict_controller_type )
            self.predict_controller_type = predict_controller_type;
        end
    end
end
