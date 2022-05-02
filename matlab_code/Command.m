classdef Command < handle

    methods (Abstract)

        initialize(obj, rover);
        execute(obj, rover);
        is_done(obj, rover);
        cmd_end(obj, rover);

    end

end