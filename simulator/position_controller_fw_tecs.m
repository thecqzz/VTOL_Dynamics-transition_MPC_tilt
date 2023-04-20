classdef position_controller_fw_tecs < pid_controller
    
    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
        tecs;
        
    end
    
    properties (SetAccess = private, GetAccess = public)
        control_mode_current position_strategies;
        position_sp_types position_sp_types;
    end
    properties (SetAccess=protected, GetAccess=public)
        fw_thr_cruise = 1; % Cruise throttle
        nav_gpsf_r = 15;  % Fixed bank angle
        aspd_sp_slew_rate = 1; %slew rate limit for airspeed setpoin                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        t changes [m/s/S]
        land = false; % bool for landing 
        param_fw_psp_off = 0;
        Position_des;
        Airspeed_des = 35; %% hard code desire airspeed
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [rss, rpy_des] = CalculateControlCommand(obj, mult,pos_des,...
            yaw_des,vel_des, acc_des, time)
            obj.tecs = tecs();
            obj.Position_des = pos_des;
            obj.control_mode_current = position_strategies.FW_POSCTRL_MODE_AUTO;
            obj.position_sp_types = position_sp_types.SETPOINT_TYPE_POSITION;
        % Calculate time step
            dt = time - obj.LastTime;
        % Calculates PID response using this formula:
        % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            if isempty(vel_des)
                vel_des = zeros(3, 1);
            end
            if isempty(acc_des)
                acc_des = zeros(3, 1);
            end
            vel_des = norm(vel_des);

            switch obj.control_mode_current
                case 'FW_POSCTRL_MODE_AUTO'
                    ControlAuto(obj,mult,dt,vel_des);
                case 'FW_POSCTRL_MODE_AUTO_ALTITUDE'
                    ControlAutoFixedbankalthold(obj);
                case 'FW_POSCTRL_MODE_AUTO_CLIMBRATE'
                    ControlAutoDescend(obj);
                case 'FW_POSCTRL_MODE_AUTO_LANDING'
                    ControlAutolanding(obj);
                case 'FW_POSCTRL_MODE_AUTO_TAKEOFF'
                    ControlAutotakeoff(obj);
%                 case 'FW_POSCTRL_MODE_OTHER'
                    %TODO
            end
           rpy_des = zeros(3,1);
           rpy_des(2) = rad2deg(obj.get_tecs_pitch(obj));
           throttle = obj.get_tecs_thrust(obj);
           throttle = 0;
           
           rss = zeros(4,1);
           for i = 1 : mult.NumOfRotors   
                rss(i,1) = (throttle * mult.Rotors{i}.RPMLimit / 30 * pi)^2;
           end
        end


        function SetAttitudeStrategy(obj, attitude_strategy)
            if ~isa(attitude_strategy, 'attitude_strategies')
                error('Position Controller: Input attitude strategy must be from attitude_strategies enum.');
            end
            obj.AttitudeStrategy = attitude_strategy;
        end
        function ControlAuto(obj,mult,dt,vel_des)            
            switch obj.position_sp_types
                case 'SETPOINT_TYPE_IDLE'
                    mult.RPY(1) = 0;
                    mult.RPY(2) = obj.param_fw_psp_off;
                    % TODO
                    % thrust update
                    % flaps & spoilers
                    
                case 'SETPOINT_TYPE_POSITION'
                    control_auto_position(obj,mult,dt,vel_des);
                case 'SETPOINT_TYPE_VELOCITY'
                    control_auto_velocity(obj,mult);
                case 'SETPOINT_TYPE_LOITER'
                    contrl_auto_loiter(obj,mult);
            end
            
            
        end
        function control_auto_position(obj,mult,dt,vel_des)
            mission_throttle = obj.fw_thr_cruise;
%             if isfinite(cruising_throttle) && cruising_throttle >= 0  %TODO
%                 mission_throttle = cruising_throttle;
%             end
            if mission_throttle < 0
                obj.tecs.pitch_speed_weight = 2;
                tecs_fw_thr_min = 0;
                tecs_fw_thr_max = 0;
                tecs_fw_mission_throttle = 0;
            else
                tecs_fw_thr_min = 0;
                tecs_fw_thr_max = 1;
                tecs_fw_mission_throttle = mission_throttle;
            end
            obj.tecs_update_pitch_throttle(obj,mult,dt,obj.Position_des(3),vel_des,...
                tecs_fw_thr_min,tecs_fw_thr_max,tecs_fw_mission_throttle,false,mult.fw_p_lim_min)
        end
%         function ControlAutoDescend(obj,mult)
%             descend_rate = -0.5;
%             throttle_min = mult.Rotors{1}.LowerSpeedPercentage / 100;
%             throttle_max = mult.Rotors{1}.UpperSpeedPercentage / 100;
%             tecs_update_pitch_throttle(obj,mult,dt,mult.State.Position(3),fw_airspd_trim,obj.fw_p_lim_min,obj.fw_p_lim_max,...
%                 throttle_min,throttle_max,obj.fw_thr_cruise,false,obj.fw_p_lim_min,descend_rate)
%             %TODO
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             roll_des = radian(mult.RPY(1));
%             yaw_des = 0;
%             if obj.land
%                 thrust_body = throttle_min;
%             else
%                 thrust_body = min(min(tecs.last_throttle_setpoint,1),throttle_max);
%             end
%             pitch_des = obj.last_pitch_setpoint;
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         end
        
    end
    
    methods(Static)
        function tecs_update_pitch_throttle(obj,mult,dt,alt_sp,airspeed_sp,throttle_min,...
                throttle_max,throttle_cruise,climbout_mode,climbout_pitch_min_rad,disable_underspeed_detection,hgt_rate_sp)
            if nargin < 11
                disable_underspeed_detection = false;
            end
            if nargin < 12
                hgt_rate_sp = NaN;
            end
            
            air_speed = norm(mult.State.Velocity);
            RIB = mult.GetRotationMatrix();
            body_acceleration = RIB * mult.State.Acceleration;
            % update TECS vehicle state estimates
            obj.tecs.update_vehicle_state_estimate(obj.tecs,norm(mult.State.AirVelocity),body_acceleration(1),mult.State.Position(3),mult.State.Velocity(3));
            %% _param_fw_psp_off
            %An airframe specific offset of the pitch setpoint in degrees, the
            %value is added to the pitch setpoint and should correspond to the pitch at typical cruise speed of the airframe.
            %% scale_throttle
            %Scale throttle by pressure change
            scale_throttle = 0.1;
            baro_pressure = physics.StandardPressure;% TODO adding baro pressure later
            eas2tas = sqrt(physics.StandardPressure / baro_pressure);
            scale = constrain((eas2tas - 1) * scale_throttle + 1,1,2);
            
            throttle_max = constrain(1 * scale,0,1);
            obj.tecs.update_pitch_throttle(obj.tecs,dt,deg2rad(mult.State.RPY(2)),mult.State.Position(3),...
                                        alt_sp,airspeed_sp,air_speed,eas2tas,...
                                        climbout_mode,climbout_pitch_min_rad,...
                                        0,throttle_max,...
                                        throttle_cruise,deg2rad(mult.fw_p_lim_min),...
                                        deg2rad(mult.fw_p_lim_max),mult.fw_t_climb_r_sp,...
                                        mult.fw_t_sink_r_sp,hgt_rate_sp);
        
        end
        function pitch_des = get_tecs_pitch(obj)
            pitch_des = obj.tecs.last_pitch_setpoint;
        end
        function thrust_des = get_tecs_thrust(obj)
            thrust_des = min(obj.tecs.last_throttle_setpoint,1);
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