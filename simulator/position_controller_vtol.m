classdef position_controller_vtol < pid_controller

    properties

        MC_position_controller position_controller
        FW_position_controller position_controller_fw_tecs
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2

    end

    properties (SetAccess = private, GetAccess = public)
        AttitudeStrategy attitude_strategies = attitude_strategies.FullTilt;
    end

    methods

        function [lin_accel,rpy_des] = CalculateControlCommand(obj, mult, pos_des,vel_des, yaw_des,acc_des, time)

            obj.MC_position_controller = position_controller;
            obj.FW_position_controller = position_controller_fw_tecs;

            obj.MC_position_controller.SetPID(obj.P,obj.I,obj.D);
            obj.FW_position_controller.SetPID(obj.P,obj.I,obj.D);

            [lin_accel,rpy_des] = obj.MC_position_controller.CalculateControlCommand(mult, pos_des,vel_des, yaw_des,acc_des, time);
            
            

            %             if         mult.Servos{1}.CurrentAngle >= 0 & mult.Servos{1}.CurrentAngle <= 27
            %                 [lin_accel,rpy_des] = obj.MC_position_controller.CalculateControlCommand(mult, pos_des,vel_des, yaw_des,acc_des, time);
            %
            %             elseif     mult.Servos{1}.CurrentAngle > 27 & mult.Servos{1}.CurrentAngle <= 90
            %                 [lin_accel,rpy_des] = obj.FW_position_controller.CalculateControlCommand(mult, pos_des,vel_des, yaw_des,acc_des, time);
            %
            %             end
            %


        end
    end
end
