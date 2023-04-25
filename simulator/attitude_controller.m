classdef attitude_controller < pid_controller

    properties

        RateLimits = deg2rad([100; 100; 50]); % in rad/s
        OutputMax = deg2rad([15; 15; 15]); % in rad/s^2


    end

    methods

        function euler_accel = CalculateControlCommand(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, time)%time

            %             obj.P = 30.*eye(3);
            %             obj.I = 1.*eye(3);
            %             obj.D = 10.*eye(3);

            if isempty(rpy_dot_des)
                rpy_dot_des = zeros(3, 1);
            end
            if isempty(eul_acc_des)
                eul_acc_des = zeros(3, 1);
            end
            dt = time - obj.LastTime;
            % Calculate the error in radians
            rpy_err = deg2rad(rpy_des) - deg2rad(mult.State.RPY);

            for i = 1:3
                if rpy_err(i) < 1.0e-6 & rpy_err(i) > -1.0e-6
                    rpy_err(i) = 0;
                end
            end

            %% guard
            
            for i = 1:3
            if rpy_err(i) > 90
                disp("error check att controller")
            end
            end

            % Calculate the rate in radians
            rpy_dot_err = deg2rad(rpy_dot_des - mult.State.EulerRate);

            for i = 1:3
                if rpy_dot_err(i) < 1.0e-6 & rpy_dot_err(i) > -1.0e-6
                    rpy_dot_err(i) = 0;
                end
            end

            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + rpy_err * dt;

            for i = 1:3
                if obj.ErrorIntegral(i) < 1.0e-6 & obj.ErrorIntegral(i) > -1.0e-6
                    obj.ErrorIntegral(i) = 0;
                end
            end

            % Calculate the PID result
            euler_accel = eul_acc_des + obj.P * rpy_err + ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;

            % Apply the euler rate limits
            euler_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, rpy_err, ...
                deg2rad(mult.State.EulerRate), euler_accel, obj.RateLimits, true);

            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(euler_accel, rpy_err, rpy_dot_err)

            % Limit the output
            obj.LastTime = time;
        end

    end
end
