classdef HoldHorses < Command

    properties (Constant)
    end

    properties (Access = private)
        wait_time; % How long we want to wait for
        start_time; % Time that we start running the command
    end

    methods
        function obj = HoldHorses(rover_handle, wait_time)

            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor

            if (nargin == 1) % If we don't get wait_time specifically passed in
                obj.wait_time = 0.5; % Default 0.5 seconds
            else % If we get wait_time passed in
                obj.wait_time = wait_time;
            end
        end

        function initialize(obj)
            obj.start_time = obj.rover_handle.system_time; % We're starting the command running at the current system time
            disp(strcat("HoldHorses: waiting ", num2str(obj.wait_time), " seconds"));
        end

        function execute(obj)
            %Hold horses
        end

        function done = is_done(obj)
            done = obj.rover_handle.system_time > obj.start_time + obj.wait_time; % Has at least wait_time seconds elapsed since start_time?
        end

        function done = cmd_end(obj)
            %disp("HoldHorses: done!");
        end

    end
end