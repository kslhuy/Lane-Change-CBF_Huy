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
    end
end
