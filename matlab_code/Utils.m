classdef Utils
    methods (Static)
        %% static methods

        %% Returns time since epoch, as a floating point number. Useful in computing delta times between loops.
        function unix_timestamp = get_current_time()
            unix_timestamp = posixtime(datetime(now,'ConvertFrom','datenum'));  % Horrible hack to get POSIX timestamp
        end

        %% Maps a value from one range (specified by low and high ranges) to another range (specified by low+high as well)
        function x_out = map(val, in_min, in_max, out_min, out_max)
            x_out = (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        end

        %% Clamps a value between val_min and val_max
        function x_out = clamp(val, val_min, val_max)
            if (val < val_min) 
                x_out = val_min;
            elseif  (val > val_max)
                x_out = val_max;
            else
                x_out = val;
            end
        end

    end % End static methods
end

