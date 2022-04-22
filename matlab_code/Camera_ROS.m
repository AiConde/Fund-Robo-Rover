classdef Camera_ROS < handle
    % Camera ROS interface class object

    properties
        %% class properties
    end % End class properties

    properties (Dependent)
        %% getter properties
    end % End getter properties

    methods
        %% getter methods
    end % End getter methods

    methods (Static)
        %% static methods
    end % End static methods

    properties  (Access = private)
        %% private class properties
        camera_sub % MATLAB camera object
        last_image_msg % Last image we captured

        new_image_available % Update whenever we recieve a new image
        img_cache % Last decoded image
    end % End clas
    properties (SetAccess = private)
        camera_intrinsics % Calibrated camera intrinsics - read only from outside the class
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Camera_ROS(camera_intrinsics)
		    obj.camera_sub = rossubscriber("/usb_cam/image_raw/compressed", @obj.Callback_Image, "DataFormat", "struct");
            obj.new_image_available = false;
            obj.camera_intrinsics = camera_intrinsics;
        end

        function delete(obj)
            clear obj.camera_sub;
        end

	    function Callback_Image(obj, sub, imagedata)
            obj.new_image_available = true;
            obj.last_image_msg = imagedata;
		    %obj.last_image = rosReadImage(imagedata);
	    end

        function is_new_image = is_new_image_available(obj)
            is_new_image = obj.new_image_available;
        end

        function img = get_image_raw(obj)
            if (obj.new_image_available)
                obj.new_image_available = false;
                img = rosReadImage(obj.last_image_msg);
                img_cache = img;
            else
                img = img_cache;
            end
        end


        function [img_undistorted, new_image_origin] = undistort_image(obj, img)
            [img_undistorted, new_image_origin] = undistortImage(img, obj.camera_intrinsics, "OutputView","same");
        end

    end % End classmethods

end % End class

