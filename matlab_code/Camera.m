classdef Camera < handle
    % Camera class object. Represents a single Camera.

    properties
        %% class properties
        last_image_acq % Last image we captured
        last_image_acq_tstamp % Timestep of last image
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
        camera_obj % MATLAB camera object
    end % End clas
    properties (SetAccess = private)
        camera_intrinsics % Calibrated camera intrinsics - read only from outside the class
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Camera(webcam_idx, camera_intrinsics)
            imaqreset;
            obj.camera_obj = webcam(webcam_idx);
            obj.camera_intrinsics = camera_intrinsics;
        end

        function delete(obj)
            clear obj.camera_obj;
            imaqreset;
        end

        function config_resolution(obj, res)
            assert (any(strcmp(obj.camera_obj.AvailableResolutions, res)))
            obj.camera_obj.Resolution = res;
        end

        function config_exposuremode(obj, exposuremode)
            assert(strcmp('auto', exposuremode) || strcmp('manual', exposuremode));
            obj.camera_obj.ExposureMode = exposuremode;
        end
        function config_exposure(obj, exposure)
            obj.camera_obj.Exposure = exposure;
        end

        function config_brightness(obj, brightness)
            obj.camera_obj.Brightness = brightness;
        end

        function config_whitebalancemode(obj, whitebalancemode)
             assert(strcmp('auto', whitebalancemode) || strcmp('manual', whitebalancemode));
             obj.camera_obj.WhiteBalanceMode = whitebalancemode;
        end
        function config_whitebalance(obj, whitebalance)
            obj.camera_obj.WhiteBalance = whitebalance;
        end

        function flush_buffer(obj)
            flushdata(obj.camera_obj);
        end

        function img = cap_img(obj)
            obj.last_image_acq = snapshot(obj.camera_obj);
            img = obj.last_image_acq;
            obj.last_image_acq_tstamp = Utils.get_current_time();
        end

        function [img_undistorted, new_image_origin] = undistort_image(obj, img)
            [img_undistorted, new_image_origin] = undistortImage(img, obj.camera_intrinsics, "OutputView","same");
        end

    end % End classmethods

end % End class

