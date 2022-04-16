classdef Camera_ROS < handle
    % Camera ROS interface class object

    properties
        %% class properties
        last_image % Last image we captured
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
    end % End clas
    properties (SetAccess = private)
        camera_intrinsics % Calibrated camera intrinsics - read only from outside the class
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Camera_ROS(camera_intrinsics)
		obj.camera_sub = rossubscriber("/usb_cam/image_raw", @obj.Callback_Image, "DataFormat", "struct");
            obj.camera_intrinsics = camera_intrinsics;
        end

        function delete(obj)
            clear obj.camera_sub;
        end

	    function Callback_Image(obj, sub, imagedata)
		    obj.last_image = rosReadImage(imagedata);
	    end


        function [img_undistorted, new_image_origin] = undistort_image(obj, img)
            [img_undistorted, new_image_origin] = undistortImage(img, obj.camera_intrinsics, "OutputView","same");
        end

    end % End classmethods

end % End class

