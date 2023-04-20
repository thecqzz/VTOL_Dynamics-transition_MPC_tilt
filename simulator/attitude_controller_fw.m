classdef attitude_controller_fw < attitude_controller_vtol

    properties
%         RateLimits = [70; 70; 30]; % in deg/s
%         OutputMax = [8; 8; 8]; % in rad/s^2
        IntegratorMax = [1; 1 ; 1];
        roll_rate_sp = 0;
        pitch_rate_sp = 0;
        yaw_rate_sp = 0;
        
        roll_lastoutput = 0;
        pitch_lastoutput = 0;
        yaw_lastoutput = 0;
        scaler = 1;
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, dt)
%             if isempty(rpy_dot_des)
%                 rpy_dot_des = zeros(3, 1);
%             end
%             if isempty(eul_acc_des)
%                 eul_acc_des = zeros(3, 1);
%             end
% 
%             % Calculate time step
%             dt = time - obj.LastTime;

            disp(dt)
            % Update input data for rate controller
            YawControlAttitude(obj,mult,rpy_des);
            % Run attitude rate controller
            RollControlBodyrate(obj,mult,dt,rpy_des);
            PitchControlBodyrate(obj,mult,dt,rpy_des);
            YawControlBodyrate(obj,mult,rpy_des);
            
            euler_accel = [obj.roll_lastoutput   obj.pitch_lastoutput    obj.yaw_lastoutput]';
            
            % Apply the euler rate limits
%             euler_accel = min(max(euler_accel,-deg2rad(obj.RateLimits)),deg2rad(obj.RateLimits));
            
            % Limit the output
            euler_accel = obj.LimitOutput(euler_accel);
%             fprintf('euler accel (1) %d\n',euler_accel(1))
%             fprintf('euler accel (2) %d\n',euler_accel(2))
%             fprintf('euler accel (3) %d\n',euler_accel(3))
            % Update the time of the last call                      
%             obj.LastTime = time;      
        end
        
        function YawControlAttitude(obj,mult,rpy_des)
            inverted = false;
            
            if abs(mult.State.RPY(1)) < 90
                constrained_roll = min(max(mult.State.RPY(1), -80), 80);
            else
                inverted = true;
                if mult.State.RPY(1) > 0
                    constrained_roll = min(max(mult.State.RPY(1),100),180);
                else
                    constrained_roll = min(max(mult.State.RPY(1),-180),-100);
                end
            end
            constrained_roll = min(max(mult.State.RPY(1), -abs(rpy_des(1))), abs(rpy_des(1)));
            
            if ~inverted
                obj.yaw_rate_sp = tand(constrained_roll) * cosd(mult.State.RPY(1)) * physics.Gravity(3) / norm(mult.State.AirVelocity);
            end
            if ~isfinite(obj.yaw_rate_sp)
                warning('yaw rate setpoint not finite');
            end
        end
        
        function RollControlBodyrate(obj,mult,dt,rpy_des)
            roll_err = deg2rad(rpy_des(1) - mult.State.RPY(1));
            obj.roll_rate_sp = roll_err / dt;
            roll_bodyrate_sp = obj.roll_rate_sp - sind(mult.State.RPY(1))*obj.yaw_rate_sp;
            if ~isfinite(roll_bodyrate_sp)
                obj.ErrorIntegral(1) = 0;
            end
            roll_rate_err = roll_bodyrate_sp - mult.State.EulerRate(1);
            if obj.I(1) > 0
%                 id = roll_rate_err * dt * obj.scaler * obj.scaler;
                id = roll_rate_err  * obj.scaler * obj.scaler;
                if obj.roll_lastoutput < -1
                    id = max(id,0);
                elseif obj.roll_lastoutput > -1
                    id = min(id,0);
                end 
                obj.ErrorIntegral(1) = min(max(obj.ErrorIntegral(1)+ id * obj.I(1),-1.2*obj.OutputMax(1)),1.2*obj.OutputMax(1));
            end
            obj.roll_lastoutput = roll_bodyrate_sp * obj.k_ff * obj.scaler + roll_rate_err * obj.P(1) *...
                obj.scaler * obj.scaler + obj.I(1) * obj.ErrorIntegral(1);
            obj.roll_lastoutput = min(max(obj.roll_lastoutput,-obj.OutputMax(1)),obj.OutputMax(1));
        end
        
        function PitchControlBodyrate(obj,mult,dt,rpy_des)
            pitch_err = deg2rad(rpy_des(2) - mult.State.RPY(2));
            obj.pitch_rate_sp = pitch_err / dt;

            pitch_bodyrate_sp = cosd(mult.State.RPY(1)) * obj.pitch_rate_sp + cosd(mult.State.RPY(2)) * ...
                sind(mult.State.RPY(1)) * obj.yaw_rate_sp;
            if ~isfinite(pitch_bodyrate_sp)
                obj.ErrorIntegral(2) = 0;
            end
            pitch_rate_err = pitch_bodyrate_sp - mult.State.EulerRate(2);
%            fprintf('pitch bodyrate sp %d\n',pitch_bodyrate_sp)
%            fprintf('pitch rate err %d\n',pitch_rate_err)
            if obj.I(2,2) > 0
                
%                 id = pitch_rate_err * dt * obj.scaler * obj.scaler;
                id = pitch_rate_err * obj.scaler * obj.scaler;
                if obj.pitch_lastoutput < -1
                    id = max(id,0);
                elseif obj.pitch_lastoutput > -1
                    id = min(id,0);
                end
                obj.ErrorIntegral(2) = min(max(obj.ErrorIntegral(2)+ id * obj.I(2,2),-1.2*obj.IntegratorMax(2)),1.2*obj.IntegratorMax(2));
            end
            obj.pitch_lastoutput = pitch_bodyrate_sp * obj.k_ff * obj.scaler + pitch_rate_err * obj.P(2,2) *...
                obj.scaler * obj.scaler +  obj.ErrorIntegral(2);
            obj.pitch_lastoutput = min(max(obj.pitch_lastoutput,-obj.OutputMax(2)),obj.OutputMax(2));
        end
        
        function YawControlBodyrate(obj,mult,dt)
            yaw_bodyrate_sp = -sind(mult.State.RPY(1)) * obj.pitch_rate_sp + cosd(mult.State.RPY(1)) * ...
                cosd(mult.State.RPY(2)) * obj.yaw_rate_sp;
            yaw_rate_err = yaw_bodyrate_sp - mult.State.EulerRate(3);
            if obj.I(3,3) > 0
%                 id = yaw_rate_err * dt * obj.scaler * obj.scaler;
                id = yaw_rate_err * obj.scaler * obj.scaler;
                if obj.yaw_lastoutput < -1
                    id = max(id,0);
                elseif obj.yaw_lastoutput > -1
                    id = min(id,0);
                end
                obj.ErrorIntegral(3) = min(max(obj.ErrorIntegral(3)+ id * obj.I(3,3),-1.2*obj.OutputMax(3)),1.2*obj.OutputMax(3));
            end
            obj.yaw_lastoutput = yaw_bodyrate_sp * obj.k_ff * obj.scaler + yaw_rate_err * obj.P(3,3) *...
                obj.scaler * obj.scaler + obj.I(3,3) * obj.ErrorIntegral(3);
            obj.yaw_lastoutput = min(max(obj.yaw_lastoutput,-1),1);
        end
    end
end

%% Helper functions
%% Below Part Implemented from PX4 Open Source attitude_fw > ecl_yaw_controller.cpp %%%%

