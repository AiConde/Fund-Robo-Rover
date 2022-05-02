classdef Command < handle
    properties (Access = private) 
        rover_handle;
    end

    methods
        function obj = Command(rover_obj)
            if (nargin == 0)
                error("Error: commands must be instantiated with a reference to the rover object!");
            else
                obj.rover_handle = rover_obj;
            end
        end
    end

    methods (Abstract)
        initialize(obj);
        execute(obj);
        done = is_done(obj);
        cmd_end(obj);
    end
end