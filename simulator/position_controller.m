classdef position_controller < pid_controller

    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
    end

    properties (SetAccess = private, GetAccess = public)
        AttitudeStrategy attitude_strategies = attitude_strategies.FullTilt;
    end

    methods

        function [lin_accel,rpy_des] = CalculateControlCommand(obj, mult, pos_des, vel_des, yaw_des,acc_des, time)


            % Calculates PID response using this formula:
            % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            %disp("vtol")
            if isempty(vel_des)
                vel_des = zeros(3, 1);
            end
            if isempty(acc_des)
                acc_des = zeros(3, 1);
            end

            % Calculate time step
            dt = time - obj.LastTime;
            %disp(vel_des)
            % Calculate the error
            vel_err = vel_des - mult.State.Velocity;

            % Calculate the velocity error
%            disp('accel')
%             disp(mult.State.Velocity)
            acc_err = acc_des - mult.State.Acceleration;

            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + vel_err * dt;

            % Calculate the PID result
            lin_accel_unweight = acc_des + obj.P * vel_err + ...
                obj.D * acc_err + obj.I * obj.ErrorIntegral;

%           disp(vel_err)

            % Apply the velocity limits
            lin_accel_unweight = pid_controller.ApplyRateLimits(obj.P, obj.D, vel_err, ...
                mult.State.Velocity, lin_accel_unweight, obj.RateLimits, false);


            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(lin_accel_unweight, vel_err, acc_err);

            % Limit the output
            lin_accel_unweight = obj.LimitOutput(lin_accel_unweight);
%             if(mult.Wingspan == 2)
% 
%                 lin_accel_unweight = [lin_accel_unweight(1); -lin_accel_unweight(2); lin_accel_unweight(3)];
%             end
            % Update the time of the last call
            obj.LastTime = time;
            
            
            lin_accel = lin_accel_unweight;
           % disp('lin_acc')
           % disp(lin_accel)
            
            rpy_des = CalculateAttitude(obj,lin_accel,yaw_des);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             %if mult.Servos{1}.CurrentAngle >= 90 % & mult.Servos{1}.CurrentAngle <=90
%                    
%             rpy_des = [0;0;0];

              %lin_accel(1) = max(lin_accel_unweight(1),70);

%             lin_accel(1) = lin_accel_unweight(3)*sin(mult.Servos{1}.CurrentAngle);
%             lin_accel(2) = lin_accel_unweight(2);
%             lin_accel(3) = lin_accel_unweight(3)*cos(mult.Servos{1}.CurrentAngle);
%             lin_accel = [lin_accel(1); -lin_accel(2); lin_accel(3)];
% 
%             else 
%             lin_accel = lin_accel_unweight;


                %%%%%%%%%%

% 
             %end
%              disp(lin_accel)
%             disp(lin_accel_unweight(1))
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end

        function rpy_des = CalculateAttitude(obj, acc_cmd, yaw_des)
            % Calculate the desired attitude to achieve the input acceleration
            % The desired attitude aligns the required force direction
            % (acceleration - gravity) with the Z axis. FRD frame is used here.
            if obj.AttitudeStrategy == attitude_strategies.FullTilt
                rpy_des = position_controller.CalculateFullAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeStrategy == attitude_strategies.ZeroTilt
                rpy_des = position_controller.CalculateZeroTiltAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeStrategy == attitude_strategies.MinTilt
                rpy_des = position_controller.CalculateMinimumTiltAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeStrategy == attitude_strategies.FixedTilt
                rpy_des = position_controller.CalculateFixedTiltAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeStrategy == attitude_strategies.FixedAttitude
                rpy_des = position_controller.CalculateFixedOrientationAttitude(acc_cmd, yaw_des);
            end
        end

        function SetAttitudeStrategy(obj, attitude_strategy)
            if ~isa(attitude_strategy, 'attitude_strategies')
                error('Position Controller: Input attitude strategy must be from attitude_strategies enum.');
            end
            obj.AttitudeStrategy = attitude_strategy;
        end

    end

    methods(Static)

        function rpy_des = CalculateFullAttitude(acc_cmd, yaw_des)
            %rpy_des = task2_calc_attitude(acc_cmd, yaw_des);

            % Find the desired Z axis
            force = acc_cmd - physics.Gravity;
            z_axis = -force / norm(force);

            % Calculate the desired attitude
            rpy_des = rpy_from_z_and_yaw(z_axis, yaw_des);
        end

        function rpy_des = CalculateZeroTiltAttitude(~, yaw_des)
            rpy_des = [0; 0; yaw_des];
        end

        function rpy_des = CalculateFixedTiltAttitude(~, yaw_des)

            lambda_sp = options.AS_FixedTiltAngle;
            kappa_sp = options.AS_FixedTiltDirection;

            % The axis of rotation
            z_inertial = [0; 0; 1];
            yaw_c = [cosd(kappa_sp); sind(kappa_sp); 0];
            r_vec = cross(yaw_c, z_inertial);
            r_axis = r_vec / norm(r_vec);

            % Rotate Z around r axis using Rodrigues' formula
            z_axis = (1 - cosd(lambda_sp)) * dot(r_axis, z_inertial) * r_axis + ...
                z_inertial * cosd(lambda_sp) + cross(r_axis, z_inertial) * sind(lambda_sp);

            % Calculate the desired attitude
            rpy_des = rpy_from_z_and_yaw(z_axis, yaw_des);
        end

        function rpy_des = CalculateFixedOrientationAttitude(~, yaw_des)
            rpy_des = [ options.AS_FixedAttitudeRoll; ...
                options.AS_FixedAttitudePitch; ...
                yaw_des];
        end

        function rpy_des = CalculateMinimumTiltAttitude(acc_cmd, yaw_des)

            % Compensate for the gravity
            acc_g = acc_cmd - physics.Gravity;

            flmax = options.AS_MinTiltMaxLateralAcceleration;
            lat_acc = norm(acc_g(1 : 2));

            % Return the zero tilt if the lateral acceleration is less than
            % the maximum lateral acceleration
            if lat_acc <= flmax
                rpy_des = [0; 0; yaw_des];
                return;
            end

            acc_norm = norm(acc_g);

            % Calculate the minimum tilt
            lambda_sp = asind(lat_acc / acc_norm) - asind(flmax / acc_norm);

            % The axis of rotation
            z_inertial = [0; 0; 1];
            r_vec = cross(acc_g, z_inertial);
            r_axis = r_vec / norm(r_vec);

            % Rotate Z around r axis using Rodrigues' formula
            z_axis = (1 - cosd(lambda_sp)) * dot(r_axis, z_inertial) * r_axis + ...
                z_inertial * cosd(lambda_sp) + cross(r_axis, z_inertial) * sind(lambda_sp);

            % Calculate the desired attitude
            rpy_des = rpy_from_z_and_yaw(z_axis, yaw_des);
        end

    end
end

%% Helper functions

function rpy = rpy_from_z_and_yaw(z_axis, yaw)

% Initialize the output
rpy = [0; 0; yaw];

% Find the desired X axis
x_c = cross([-sind(yaw); cosd(yaw); 0], z_axis);
x_axis = x_c / norm(x_c);

% Find the desired Y axis
y_axis = cross(z_axis, x_axis);

% Calculate the roll and pitch
rpy(1) = atan2d(y_axis(3), z_axis(3));
rpy(2) = -asind(x_axis(3));
end
