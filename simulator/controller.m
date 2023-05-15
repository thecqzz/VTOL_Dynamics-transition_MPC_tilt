classdef controller < handle
    properties
        ControlAllocation control_allocation_vtol
        AttitudeController attitude_controller
        PositionController position_controller_fw_MPC_new
        HMFController hmf_controller

        Max_speed_sq

        mode = "MC_location_initial"
    end

    methods
        function obj = controller(mult)
            obj.ControlAllocation = control_allocation_vtol(mult);
            obj.AttitudeController = attitude_controller;
            obj.PositionController = position_controller_fw_MPC_new;
            obj.HMFController = hmf_controller;
        end

        function [rotor_speeds_squared, deflections, saturated] = ControlAcceleration(obj, mult, lin_acc_des, euler_acc_des)
            [rotor_speeds_squared, deflections,saturated] = obj.ControlAllocation.CalcActuators(mult, lin_acc_des, euler_acc_des);

            obj.Max_speed_sq = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));

            rotor_speeds_squared = rotor_speeds_squared .* obj.Max_speed_sq;

        end

        function euler_accel = ControlAttitude(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, dt)

            euler_accel = obj.AttitudeController.CalculateControlCommand(mult, rpy_des, [], [], dt);

            %euler_accel = [0,0,0]';
        end


        function [lin_accel, rpy_des, tilt] = ControlPosition(obj, mult, pos_des, yaw_des, vel_des, acc_des, dt)


            %                 [lin_accel,rpy_des, tilt] = obj.PositionController.CalculateControlCommand(mult, pos_des, vel_des, yaw_des, acc_des, dt);




            %% begin PID decoy

            %% initialize position

            current_tilt = mult.Servos{1}.CurrentAngle;
            current_velocity  = mult.State.Velocity;
            current_acceleration = mult.State.Acceleration;
            speed_norm = norm(mult.State.Velocity);
            transition_tilt = 27;

            RPY = mult.State.RPY;
            Rbi = GetRotationMatrix(RPY(1),RPY(2),RPY(3));
            hover = Rbi * physics.Gravity;

            lin_accel_MC = -hover;
            lin_accel_FW = [1,0,0];

            lin_accel_transition_P1 = [-tand(current_tilt)*lin_accel_MC(3);0;lin_accel_MC(3)];
            lin_accel_transition_P2 = [lin_accel_FW(1);0;lin_accel_FW(1)/(-tand(current_tilt))];

            blending_speed = 16;
            transition_speed = 23;

            if obj.mode == "MC_location_initial"
                tilt = 0;

                P = 20;
                D = 10;
                %%%%%%%%%%%%%%%%%%%%%%%
                transition_z = -200.5;
                %%%%%%%%%%%%%%%%%%%%%%%
                position_err = transition_z - mult.State.Position(3);
                speed_err = 0 - mult.State.Velocity(3);

                lin_accel_z = position_err * P + speed_err * D;

                if lin_accel_z <= -13.8066

                    lin_accel_z = -13.8066;

                end

                lin_accel = [0,0,lin_accel_z]';
                rpy_des = [0,0,0]';

                if mult.State.Position(3) <= -200
                    obj.mode = "MC_track_vel_des";
                end

            end

            if obj.mode == "MC_track_vel_des"

                tilt = 0;

                current_acceleration = mult.State.Acceleration;

                vel_des = [10.1,0,0]';
                acceleration_des = [0,0,0]';

                P = 0.6;
                D = 0.1;

                velocity_err_x = vel_des(1) - mult.State.Velocity(1);
                acceleration_err_x = acceleration_des(1) - current_acceleration(1);
                lin_accel_x = velocity_err_x * P + acceleration_err_x * D;

                velocity_err_z = vel_des(3) - mult.State.Velocity(3);
                acceleration_err_z = acceleration_des(3) - current_acceleration(3);
                lin_accel_z = velocity_err_z * P + acceleration_err_z * D - physics.Gravity(3);

                lin_accel_body_z = -(lin_accel_x^2 + lin_accel_z^2)^0.5;

                if lin_accel_body_z <= -12.8066

                    lin_accel_body_z = -12.8066;

                end


                lin_accel = [0,0,lin_accel_body_z]';

                pitch_angle = atand(lin_accel_x/lin_accel_z);

                rpy_des = [0,pitch_angle,0]';


                if norm(current_velocity - vel_des) <= 0.1

                    obj.mode = "reset_RPY";
                end



            end

            if obj.mode == "reset_RPY"

                tilt = 0;
                rpy_des = [0,0,0]';
                lin_accel = [0,0,-physics.Gravity(3)]';

                if norm(RPY - rpy_des) <= 0.001

                    obj.mode = "transition_Phase1";
                end
            end

            if obj.mode == "transition_Phase1"

                rpy_des = [0,0,0]';
                tilt = transition_tilt;
                lin_accel = lin_accel_transition_P1;

                if speed_norm >blending_speed && speed_norm <= transition_speed

                    obj.mode = "transition_Phase2";
                end


            end

            if obj.mode == "transition_Phase2"

                rpy_des = [0,0,0]';
                tilt = transition_tilt;

                ratio = (speed_norm - blending_speed)/(transition_speed-blending_speed);

                lin_accel = lin_accel_transition_P2 * ratio + lin_accel_transition_P1 * (1 - ratio);

                disp("speed_norm")
                disp(speed_norm)
                disp("ratio")
                disp(ratio)

                if speed_norm >= transition_speed

                    obj.mode = "FW_tilt";

                end
            end

            if obj.mode == "FW_tilt"

                rpy_des = [0,0,0]';
                tilt = 90.5;
                lin_accel = lin_accel_transition_P2;

                if current_tilt >= 89.9

                    obj.mode = "FW";

                end
            end

            if obj.mode == "FW"


                P_FW = 1.5;
                I_FW = 0.1;
                D_FW = 0.3;

                %vel_des = [27.7425, 0, 0]';

                vel_des_FW = [27.7425, 0, -1]';
                acceleration_des_FW = [0,0,0]';

                velocity_err_x_FW = vel_des_FW(1) - mult.State.Velocity(1);

                obj.ErrorIntegral_x_FW = obj.ErrorIntegral_x_FW + velocity_err_x_FW * dt;

                acceleration_err_x_FW = acceleration_des_FW(1) - current_acceleration(1);
                lin_accel_x_FW = velocity_err_x_FW * P_FW + obj.ErrorIntegral_x_FW * I_FW + acceleration_err_x_FW * D_FW;

                if lin_accel_x_FW <= 0
                    lin_accel_x_FW = 1e-10;
                end

                PP_FW = 1.5;
                II_FW = 0.001;
                DD_FW = 0.3;

                velocity_err_z_FW = vel_des_FW(3) - mult.State.Velocity(3);
                obj.ErrorIntegral_z_FW = obj.ErrorIntegral_z_FW + velocity_err_z_FW * dt;
                acceleration_err_z_FW = acceleration_des_FW(3) - current_acceleration(3);
                lin_accel_z_FW = velocity_err_z_FW * PP_FW + obj.ErrorIntegral_z_FW * II_FW + acceleration_err_z_FW * DD_FW;

                lin_accel_z_FW = -lin_accel_z_FW;
                %
                %             lin_accel_z = 0;
                %
                lin_accel_body_x_FW = (lin_accel_x_FW^2 + lin_accel_z_FW^2)^0.5;

                %             if lin_accel_body_x >= 12.8066
                %
                %                lin_accel_body_x = 12.8066;
                %
                %             end

                lin_accel = [lin_accel_body_x_FW, 0, 0 ]';

                pitch_angle_FW = atand(lin_accel_z_FW/lin_accel_x_FW);

                if pitch_angle_FW >= 8
                    pitch_angle_FW = 8;

                elseif pitch_angle_FW <= -8
                    pitch_angle_FW = -8;

                end


                rpy_des = [0,pitch_angle_FW,0]';

            end
           disp(tilt)
           disp(current_tilt)
         disp(obj.mode)

        end




        function Reset(obj)
            obj.AttitudeController.Reset();
            obj.PositionController.Reset();
        end


    end
end

function Rot_BI = GetRotationMatrix(roll, pitch, yaw)


s_ph = sin(roll);
s_th = sin(pitch);
s_ps = sin(yaw);
c_ph = cos(roll);
c_th = cos(pitch);
c_ps = cos(yaw);
Rot_BI = [ c_th * c_ps                      ,       c_th * s_ps                      ,          -s_th;
    s_ph * s_th * c_ps - c_ph * s_ps ,       s_ph * s_th * s_ps + c_ph * c_ps ,          s_ph * c_th;
    c_ph * s_th * c_ps + s_ph * s_ps ,       c_ph * s_th * s_ps - s_ph * c_ps ,          c_ph * c_th  ];
end