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

    end % End static methods
end

