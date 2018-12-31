classdef MODEL_Template
    properties
        pos;
    end
    methods
        function obj = MODEL_Template() % Constructor
            obj.pos = 1.0;
            
        end
        
        function obj = step()
            % Write dynmaics here
        end
        
        function [pos] = getstate(obj)            
            % Parse state
        end
        function visualize(obj)
            % Enter Visualization here
        end
    end
end