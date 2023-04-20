classdef attitude_controller_vtol < pid_controller
    
    
    
    
    properties

        MC_attitude_controller attitude_controller
        FW_attitude_controller attitude_controller_fw
        RateLimits = [70; 70; 30]; % in deg/s
        OutputMax = [1; 1; 1]; % in rad/s^2
        blending_speed_xy = 14.4;
        Transition_air_speed_xy = 18;
        weight_MC = 1;
        weight_FW = 0;
        vtol_mode = 'FW mode';
    end
    methods
        function euler_accel = CalculateControlCommand(obj,mult, rpy_des, rpy_dot_des, eul_acc_des, time)
            obj.MC_attitude_controller = attitude_controller;
            obj.FW_attitude_controller = attitude_controller_fw;
            obj.MC_attitude_controller.SetPID(obj.P,obj.I,obj.D);
            obj.FW_attitude_controller.SetPID(obj.P,obj.I,obj.D);
            air_speed_xy = norm(mult.State.AirVelocity);
  
            if isempty(rpy_dot_des)
                rpy_dot_des = zeros(3, 1);
            end
            if isempty(eul_acc_des)
                eul_acc_des = zeros(3, 1);
            end
            dt = time - obj.LastTime;
            % initial start from MC
            obj.weight_MC = 1;
            obj.weight_FW = 0;
            %if is vtol_front_transition
            if air_speed_xy >= obj.blending_speed_xy
                if obj.vtol_mode == 'Phase_1'
                    obj.weight_MC = min((air_speed_xy-obj.blending_speed_xy)/(obj.Transition_air_speed_xy-obj.blending_speed_xy),1);
                    obj.weight_FW = 1 - obj.weight_MC;
                elseif obj.vtol_mode == 'MC_mode'
                    obj.weight_MC = 1;
                    obj.weight_FW = 0;
                else
                    obj.weight_MC = 0;
                    obj.weight_FW = 1;
                end
            end
            if air_speed_xy < obj.blending_speed_xy
                obj.vtol_mode = 'MC_mode';
                euler_accel = obj.weight_MC * obj.MC_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des, dt) + obj.weight_FW * obj.FW_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des,dt);
            elseif air_speed_xy >= obj.blending_speed_xy && air_speed_xy < obj.Transition_air_speed_xy
                obj.vtol_mode = 'Phase_1';
                euler_accel = obj.weight_MC * obj.MC_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des, dt) + obj.weight_FW * obj.FW_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des,dt);
            elseif air_speed_xy > obj.Transition_air_speed_xy
                obj.vtol_mode = 'Phase_2';
                euler_accel = obj.weight_MC * obj.MC_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des, dt) + obj.weight_FW * obj.FW_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des,dt);
            else
                obj.vtol_mode = 'FW_mode';
                euler_accel = obj.weight_MC * obj.MC_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des, dt) + obj.weight_FW * obj.FW_attitude_controller.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des,dt);
            end
            euler_accel = obj.LimitOutput(euler_accel);
            obj.LastTime = time;
        end
    end
end