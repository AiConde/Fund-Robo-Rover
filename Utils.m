classdef Utils
    methods (Static)
        %% static methods

        %% Returns time since epoch, as a floating point number. Useful in computing delta times between loops.
        function unix_timestamp = get_current_time()
            unix_timestamp = posixtime(datetime(now,'ConvertFrom','datenum'));  % Horrible hack to get POSIX timestamp
        end
    end % End static methods
end

