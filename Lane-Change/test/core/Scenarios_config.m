classdef Scenarios_config
    properties
        dt ;
        simulation_time ;
        where ; % 1 indicates highway, 2 indicates urban road
    end
    methods
        function self = Scenarios_config(dt , simulation_time , scenario_where)
            self.dt = dt;
            self.simulation_time = simulation_time;
            self.where = scenario_where;
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
                lane_width = 3.6;% Highway lane width in meters
            elseif self.where == "Urban"
                lane_width = 3;
            end
        end
        
        
    end
end
