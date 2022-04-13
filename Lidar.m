classdef Lidar  < handle
    % Lidar class object. Represents a Hokuyo lidar.

    properties
        %% class properties
        last_scan_angles % Last set of scanned angles we got
        last_scan_angles_tstamp % Timestep of last scan
    end % End class properties

    properties (Dependent)
        %% getter properties
    end % End getter properties

    methods
        %% getter methods
    end % End getter methods

    methods (Static)
        %% static methods
        function angles_filtered = filter_angles(angles)
            % TODO
        end

        function cartesian_points = polar_to_cartesian(angles)
            % TODO
        end
    end % End static methods

    properties  (Access = private)
        %% private class properties
        lidar_obj
    end % End clas

    properties(Constant)
        %% Constant properties
        lidar_angles = (-120:240/682:120-240/682)*pi/180;
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Lidar(com_port)
            serial_objs = instrfind;
            if ~isempty(serial_objs)
                fclose(serial_objs);
                delete(serial_objs);
            end

            obj.lidar_obj = serial(com_port, 'baudrate', 115200);
            set(obj.lidar_obj, 'Timeout', 0.1); % TODO test with 0.1
            set(obj.lidar_obj, 'InputBufferSize', 40000); % TODO test with 40000
            set(obj.lidar_obj, 'Terminator', 'CR'); % TODO test with CR

            fopen(obj.lidar_obj);
            pause(0.1);

            fprintf(obj.lidar_obj, 'SCIP2.0');
            pause(0.1);

            fscanf(obj.lidar_obj);
            fprintf(obj.lidar_obj, 'VV');
            pause(0.1);

            fscanf(obj.lidar_obj);
            fprintf(obj.lidar_obj, 'BM');
            pause(0.1);

            %fscanf(obj.lidar_obj);
            %fprintf(obj.lidar_obj, 'MD0044072500'); %TODO this may be unnecessary
            %pause(0.3);

            fscanf(obj.lidar_obj);
        end

        %% Class destructor
        function delete(obj)
            fprintf(obj.lidar_obj, 'QT');
            fclose(obj.lidar_obj);
            clear obj.lidar_obj;
        end

        function angles = lidar_scan(obj)
            [A] = LidarScan(obj.lidar_obj);
            obj.last_scan_angles_tstamp = Utils.get_current_time();
            angles = A;
            obj.last_scan_angles = A;
        end

        function rangeval=decodeSCIP(rangeenc)
            % Check for 2 or 3 Character Encoding
            if rangeenc(1)=='0' && rangeenc(2)=='0' && rangeenc(3)=='0'
                rangeval=0;
                return;
            end
            if rangeenc(1)=='0'
                dig1=((rangeenc(2)-'!')+33);
                dig2=((rangeenc(3)-'!')+33);
                dig1sub=dig1-48;
                dig2sub=dig2-48;
                dig1bin=dec2bin(dig1sub,6);
                dig2bin=dec2bin(dig2sub,6);
                rangeval=bin2dec([dig1bin dig2bin]);
                return;
            else
                dig1=((rangeenc(1)-'!')+33);
                dig2=((rangeenc(2)-'!')+33);
                dig3=((rangeenc(3)-'!')+33);
                dig1sub=dig1-48;
                dig2sub=dig2-48;
                dig3sub=dig3-48;
                dig1bin=dec2bin(dig1sub,6);
                dig2bin=dec2bin(dig2sub,6);
                dig3bin=dec2bin(dig3sub,6);
                rangeval=bin2dec([dig1bin dig2bin dig3bin]);
                return;

            end

        end


        function [rangescan]=FunRoboLidarScan(lidar)
            proceed=0;
            fprintf(lidar,'GD0044072500');
            while (proceed==0)

                if lidar.BytesAvailable >= 2134
                    data = fscanf(lidar,'%c',2134);
                    proceed = 1;
                end

                % data=fscanf(lidar);
                % if numel(data)==2134
                %     proceed=1;
                % end

            end

            i = find(data == data(13));
            rangedata=data(i(3)+1:end-1);
            for j=0:31
                onlyrangedata((64*j)+1:(64*j)+64)=rangedata(1+(66*j):64+(66*j));
            end
            j=0;
            for i=1:floor(numel(onlyrangedata)/3)
                encodeddist(i,:)=[onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
                j=j+1;
            end
            for k=1:size(encodeddist,1)
                rangescan(k)=decodeSCIP(encodeddist(k,:));
            end
        end

        function [rangescan]=LidarScan(lidar)
            proceed=0;
            while (proceed==0)
                fprintf(lidar,'GD0044072500');
                pause(0.01);
                data=fscanf(lidar);
                if numel(data)==2134
                    proceed=1;
                end
            end
            i=find(data==data(13));
            rangedata=data(i(3)+1:end-1);
            for j=0:31
                onlyrangedata((64*j)+1:(64*j)+64)=rangedata(1+(66*j):64+(66*j));
            end
            j=0;
            for i=1:floor(numel(onlyrangedata)/3)
                encodeddist(i,:)=[onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
                j=j+1;
            end
            for k=1:size(encodeddist,1)
                rangescan(k)=decodeSCIP(encodeddist(k,:));
            end
        end

    end % End classmethods



end % End class

