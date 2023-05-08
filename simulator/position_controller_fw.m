classdef position_controller_fw < pid_controller
    
    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
        
    end
    
    properties (SetAccess = private, GetAccess = public)
        AttitudeStrategy attitude_strategies = attitude_strategies.FullTilt;
    end
    properties (SetAccess=protected, GetAccess=public)
        fw_p_lim_max = 45; % Positive pitch limit
        fw_p_lim_min = -45; % Negative pitch limit
        fw_thr_cruise = 0.6; % Cruise throttle
        nav_gpsf_r = 15;  % Fixed bank angle
        aspd_sp_slew_rate = 1; %slew rate limit for airspeed setpoint changes [m/s/S]
        land = false; % bool for landing 
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [lin_accel, rpy_des] = CalculateControlCommand(obj, mult, pos_des,...
            vel_des, acc_des, time)
        % Calculates PID response using this formula:
        % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            if isempty(vel_des)
                vel_des = zeros(3, 1);
            end
            if isempty(acc_des)
                acc_des = zeros(3, 1);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            curr_alt = - mult.State.Position(3);
            load_factor = 1 / cosd(mult.State.RPY(1));
            tecs.load_factor = load_factor;
            mode = 'fw_posctrl_mode_auto';
            mode = 'fw_posctrl_mode_auto_altitude';
            mode = 'fw_posctrl_mode_auto_climbrate';
            switch mode
                case 'fw_posctrl_mode_auto'
                    ControlAuto(obj);
                case 'fw_posctrl_mode_auto_altitude'
                    ControlAutoFixedbankalthold(obj);
                case 'fw_posctrl_mode_auto_climbrate'
                    COntrolAutoDescend(obj)
            end
            % Calculate time step
            dt = time - obj.LastTime;
                 
        end


        function SetAttitudeStrategy(obj, attitude_strategy)
            if ~isa(attitude_strategy, 'attitude_strategies')
                error('Position Controller: Input attitude strategy must be from attitude_strategies enum.');
            end
            obj.AttitudeStrategy = attitude_strategy;
        end
        function ControlAutoDescend(obj)
            descend_rate = -0.5;
            throttle_min = mult.Rotors{1}.LowerSpeedPercentage / 100;
            throttle_max = mult.Rotors{1}.UpperSpeedPercentage / 100;
            tecs_update_pitch_throttle(obj,dt,mult.State.Position(3),fw_airspd_trim,obj.fw_p_lim_min,obj.fw_p_lim_max,...
                throttle_min,throttle_max,obj.fw_thr_cruise,false,obj.fw_p_lim_min,descend_rate)
            roll_des = obj.nav_gpsf_r;
            yaw_des = 0;
            if obj.land
                thrust_body = throttle_min;
            else
                thrust_body = min(min(tecs.last_throttle_setpoint,1),throttle_max);
            end
            pitch_des = obj.last_pitch_setpoint;
            
        end
    end
    
    methods(Static)
        
        function tecs_update_pitch_throttle(dt,alt_sp,airspeed_sp,pitch_min_rad,pitch_max_rad,throttle_min,...
                throttle_max,throttle_cruise,climbout_mode,climbout_pitch_min_rad,hgt_rate_sp)
            tecs.update_vehicle_state_estimate(airspeed,body_acceleration(0),current_altitude,mult.State.Velocity(3))
            %% _param_fw_psp_off 
            %An airframe specific offset of the pitch setpoint in degrees, the 
            %value is added to the pitch setpoint and should correspond to the pitch at typical cruise speed of the airframe.
            %% scale_throttle
            %Scale throttle by pressure change
            scale_throttle = 0.1;
            baro_pressure = physics.StandardPressure;% TODO adding baro pressure later
            eas2tas = sqrt(physics.StandardPressure / baro_pressure);
            scale = constrain((eas2tas - 1) * scale_throttle + 1,1,2);
            throttle_max = constrain(throttle_max * scale,throttle_min,1)
            tecs.update_pitch_throttle(mult.State.RPY(2),mult.State.Position(3),alt_sp,airspeed_sp,air_velocity,eas2tas,...
            climbout_mode,climbout_pitch_min_rad,throttle_min,throttle_max,throttle_cruise,...
            pitch_min_rad,pitch_max_rad,param_climbrate_target,param_sinkrate_target,hgt_rate_sp)
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

function constrainoutput = constrain(x,minval,maxval)
    constrainoutput = min(max(x, minval), maxval);
end