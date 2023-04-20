classdef tecs < handle
    
    properties (SetAccess=public, GetAccess=public)
        DT_MIN = 0.001; % minimum allowed value of _dt (sec)
        DT_MAX = 1.0;   % max value of _dt allowed before a filter state reset is performed (sec)

        % controller parameters
        hgt_estimate_freq = 0.0;                 %cross-over frequency of the height rate complementary filter (rad/sec)
        tas_estimate_freq = 0.0;                 %cross-over frequency of the true airspeed complementary filter (rad/sec)
        max_climb_rate = 2.0;                    %climb rate produced by max allowed throttle (m/sec)
        min_sink_rate = 1.0;                     %sink rate produced by min allowed throttle (m/sec)
        max_sink_rate = 2.0;                     %maximum safe sink rate (m/sec)
        pitch_time_constant = 5.0;               %control time constant used by the pitch demand calculation (sec)
        throttle_time_constant = 8.0;            %control time constant used by the throttle demand calculation (sec)
        pitch_damping_gain = 0.0;                %damping gain of the pitch demand calculation (sec)
        throttle_damping_gain = 0.0;             %damping gain of the throttle demand calculation (sec)
        integrator_gain_throttle = 0.0;          %integrator gain used by the throttle and pitch demand calculation
        integrator_gain_pitch = 0;               %integrator gain used by the pitch demand calculation
        vert_accel_limit = 0.0;                  %magnitude of the maximum vertical acceleration allowed (m/sec**2)
        load_factor = 0;
        load_factor_correction = 0.0;            %gain from normal load factor increase to total energy rate demand (m**2/sec**3)
        pitch_speed_weight = 1.0;                %speed control weighting used by pitch demand calculation
        height_error_gain = 0.0;                 %gain from height error to demanded climb rate (1/sec)
        height_setpoint_gain_ff = 0.0;           %gain from height demand derivative to demanded climb rate
        airspeed_error_gain = 0.1;                  %gain from speed error to demanded speed rate (1/sec)
        equivalent_airspeed_min = 3.0;            %equivalent airspeed demand lower limit (m/sec)
        equivalent_airspeed_max = 30.0;           %equivalent airspeed demand upper limit (m/sec)
        equivalent_airspeed_cruise = 30;
        throttle_slewrate = 0.0;                 %throttle demand slew rate limit (1/sec)
        SEB_rate_ff = 1;
        STE_rate_time_const = 0.1;
        speed_derivative_time_const = 0.01;
        %  complimentary filter states
        tas_innov = 0;                           % complimentary filter true airspeed innovation (m/sec)

        % speed demand calculations
        EAS = 0;                                 %equivalent airspeed (m/sec)
        TAS_max = 50.0;                          %true airpeed demand upper limit (m/sec)
        TAS_min=3.0;                             %true airpeed demand lower limit (m/sec)
        TAS_setpoint=0.0;                        %current airpeed demand (m/sec)
        TAS_setpoint_last=0.0;                   %previous true airpeed demand (m/sec)
        EAS_setpoint=0.0;                        %Equivalent airspeed demand (m/sec)
        TAS_setpoint_adj=0.0;                    %true airspeed demand tracked by the TECS algorithm (m/sec)
        TAS_rate_setpoint=0.0;                   %true airspeed rate demand tracked by the TECS algorithm (m/sec**2)
        tas_rate_raw = 0;
        
        % specific energy quantities
        SPE_setpoint =0.0;                       %specific potential energy demand (m**2/sec**2)
        SKE_setpoint =0.0;                    %specific kinetic energy demand (m**2/sec**2)
        SPE_rate_setpoint = 0.0;                 %specific potential energy rate demand (m**2/sec**3)
        SKE_rate_setpoint = 0.0;                 %specific kinetic energy rate demand (m**2/sec**3)
        SPE_estimate =0.0;                       %specific potential energy estimate (m**2/sec**2)
        SKE_estimate =0.0;                       %specific kinetic energy estimate (m**2/sec**2)
        SPE_rate =0.0;                           %specific potential energy rate estimate (m**2/sec**3)
        SKE_rate = 0.0;                          %specific kinetic energy rate estimate (m**2/sec**3)

        % specific energy error quantities
        STE_error = 0.0;                         %specific total energy error (m**2/sec**2)
        STE_rate_error = 0.0;                    %specific total energy rate error (m**2/sec**3)
        SEB_error = 0.0;                         %secific energy balance error (m**2/sec**2)
        SEB_rate_error =0.0;                     %specific energy balance rate error (m**2/sec**3)

        % controller outputs
        vert_accel_state = 0.0;                  %complimentary filter state - height second derivative (m/sec**2)
        vert_vel_state = 0.0;                    %complimentary filter state - height rate (m/sec)
        vert_pos_state = 0.0;                    %complimentary filter state - height (m)
        tas_rate_state = 0.0;                    %complimentary filter state - true airspeed first derivative (m/sec**2)
        tas_state = 0.0;                         %complimentary filter state - true airspeed (m/sec)

        % vehicle physical limits
        pitch_setpoint_unc = 0.0;                %pitch demand before limiting (rad)
        STE_rate_max = 0.0;                      %specific total energy rate upper limit achieved when throttle is at _throttle_setpoint_max (m**2/sec**3)
        STE_rate_min = 0.0;                      %specific total energy rate lower limit acheived when throttle is at _throttle_setpoint_min (m**2/sec**3)
        throttle_setpoint_max = 0;             %normalised throttle upper limit
        throttle_setpoint_min = 0.0;             %normalised throttle lower limit
        pitch_setpoint_max = 0.5;                %pitch demand upper limit (rad)
        pitch_setpoint_min = -0.5;               %pitch demand lower limit (rad)

        % height demand calculations
        hgt_setpoint = 0.0;                      %demanded height tracked by the TECS algorithm (m)
        hgt_setpoint_in_prev = 0.0;              %previous value of _hgt_setpoint after noise filtering (m)
        hgt_setpoint_prev = 0.0;                 %previous value of _hgt_setpoint after noise filtering and rate limiting (m)
        hgt_setpoint_adj = 0.0;                  %demanded height used by the control loops after all filtering has been applied (m)
        hgt_setpoint_adj_prev =0.0;              %value of _hgt_setpoint_adj from previous frame (m)
        hgt_rate_setpoint =0.0;                  %demanded climb rate tracked by the TECS algorithm

        %controller states
        throttle_integ_state = 0;				% throttle integrator state
        pitch_integ_state = 0;					% pitch integrator state (rad)
        last_throttle_setpoint = 0;			% throttle demand rate limiter state (1/sec)
        last_pitch_setpoint = 0;				% pitch demand rate limiter state (rad/sec)
        tas_rate_filtered = 0;
        % speed height weighting
        SPE_weighting = 1;
        SKE_weighting = 1;

        % controller mode logic
        climbout_mode_active = false;
        states_initialized = false;
        underspeed_detected = false;
        detect_underspeed_enabled = true;
        % Alpha filters
        STE_rate_error_filter alphafilter;
        TAS_rate_filter alphafilter; 
        
    end
    properties (SetAccess=protected, GetAccess=public)
        jerk_max = 1000;
        tecs_mode TECS_mode;
    end
    methods(Static)
        function obj = tecs()     
            obj.STE_rate_error_filter = alphafilter();
            obj.TAS_rate_filter = alphafilter();
        end
%         function handle_alt_step(obj,delta_alt,altitude)
%             %add height reset delta to all variables involved
%             %in filtering the demanded height
%             obj.hgt_setpoint_in_prev = obj.hgt_setpoint_in_prev + delta_alt;
%             obj.hgt_setpoint_prev = obj.hgt_setpoint_prev + delta_alt;
%             obj.hgt_setpoint_adj_prev = obj.hgt_setpoint_adj_prev + delta_alt;
%  0.
%             %reset height states
%             obj.vert_pos_state = altitude;
%             obj.vert_accel_state = 0.0;
%             obj.vert_vel_state = 0.0;
%         end
        function update_vehicle_state_estimate(obj,airspeed,accel_x,altitude,vz)
            obj.EAS = airspeed;
            obj.vert_vel_state = -vz;
            obj.vert_pos_state = altitude;
            
            if isfinite(airspeed)
                obj.tas_rate_raw = accel_x;
            else
                obj.tas_rate_raw = 0;
                obj.tas_rate_filtered = 0;
            end
        end
        
        
        function InitializeStates(obj,dt,pitch,throttle_cruise,altitude,pitch_min_climbout,eas2tas)
            if ~obj.states_initialized
                obj.vert_vel_state = 0;
                obj.vert_pos_state = 0;
                obj.tas_rate_state = 0;
                obj.tas_state = obj.EAS * eas2tas;
                obj.throttle_integ_state = 0;
                obj.pitch_integ_state = 0;
                obj.last_throttle_setpoint = throttle_cruise;
                obj.last_pitch_setpoint = constrain(pitch,obj.pitch_setpoint_min,obj.pitch_setpoint_max);
                obj.pitch_setpoint_unc = obj.last_pitch_setpoint;
                obj.TAS_setpoint_last = obj.EAS * eas2tas;
                obj.TAS_setpoint_adj = obj.TAS_setpoint_last;
                obj.STE_rate_error = 0;
                obj.hgt_setpoint = altitude;
            elseif obj.climbout_mode_active
                
                obj.pitch_setpoint_min = pitch_min_climbout;
                obj.throttle_setpoint_min = obj.throttle_setpoint_max - 0.01;
                obj.TAS_setpoint_last = obj.EAS * eas2tas;
                obj.TAS_setpoint_adj = obj.EAS * eas2tas;
                obj.hgt_setpoint = altitude;
                obj.underspeed_detected = false;
                
            end
            %% filter
            obj.STE_rate_error_filter.setparameter(dt,obj.STE_rate_time_const);
            obj.STE_rate_error_filter.reset(0);
            obj.TAS_rate_filter.setparameter(dt,obj.speed_derivative_time_const);
            obj.TAS_rate_filter.reset(0);
        end
        function update_pitch_throttle(obj,dt,pitch,altitude,hgt_setpoint,eas_setpoint,...
            air_velocity,eas2tas,climbout_mode,pitch_min_climbout,...
            throttle_min,throttle_max,throttle_cruise,pitch_lim_min,...
            pitch_lim_max, target_climbrate, target_sinkrate,hgt_rate_sp)
            obj.throttle_setpoint_max = throttle_max;
            obj.throttle_setpoint_min = throttle_min;
            obj.pitch_setpoint_max = pitch_lim_max;
            obj.pitch_setpoint_min = pitch_lim_min;
            obj.climbout_mode_active = climbout_mode;
            
           
            obj.InitializeStates(obj,dt,pitch,throttle_cruise,altitude,pitch_min_climbout,eas2tas);
            %% TODO
            %obj.update_trajectory_generationconstraints;
            obj.update_speed_states(obj,dt,eas_setpoint,air_velocity,eas2tas);
            obj.update_STE_rate_lim(obj);
            obj.detect_underspeed(obj);
            obj.update_speed_height_weights(obj);
            %TODO
            % obj.detect_uncommanded_descent();
            obj.update_speed_setpoint(obj);
            obj.calculateHeightRateSetpoint(obj,hgt_setpoint,hgt_rate_sp,target_climbrate,target_sinkrate,altitude);
            obj.update_energy_estimate(obj);
            obj.update_throttle_setpoint(obj,dt,throttle_cruise);
            obj.update_pitch_setpoint(obj,dt);
            if obj.underspeed_detected
                obj.tecs_mode = TECS_mode.ECL_TECS_MODE_UNDERSPEED;
            elseif obj.climbout_mode_active
                obj.tecs_mode = TECS_mode.ECL_TECS_MODE_CLIMBOUT;
            else
                obj.tecs_mode = TECS_mode.ECL_TECS_MODE_NORMAL;
            end
        end
        function update_speed_states(obj,dt,eas_setpoint,equivalent_airspeed,eas2tas)
            %  Convert equivalent airspeed quantities to true airspeed
            obj.EAS_setpoint = eas_setpoint;
            obj.TAS_setpoint = obj.EAS_setpoint * eas2tas;
            obj.TAS_max = obj.equivalent_airspeed_max * eas2tas;
            obj.TAS_min = obj.equivalent_airspeed_min * eas2tas;
            
            % If airspeed measurements are not being used, fix the airspeed estimate to the nominal cruise airspeed
            if ~isfinite(equivalent_airspeed) %| ~inair
                obj.EAS = obj.equivalent_airspeed_cruise;
            else
                obj.EAS = equivalent_airspeed;
            end
            % If first time through or not flying, reset airspeed states 
            %% TODO
            
            % Obtain a smoothed TAS estimate using a second order complementary filter
            obj.tas_innov = (obj.EAS * eas2tas) - obj.tas_state;
            tas_rate_state_input = obj.tas_innov * obj.tas_estimate_freq * obj.tas_estimate_freq;
            % limit integrator input to prevent windup
            if obj.tas_state < 3.1
                tas_rate_state_input = max(tas_rate_state_input,0);
            end
            obj.tas_rate_state = obj.tas_rate_state + tas_rate_state_input * dt;
            tas_state_input = obj.tas_rate_state + obj.tas_rate_raw + obj.tas_innov * obj.tas_estimate_freq * 1.4142;
            obj.tas_state = obj.tas_state + tas_state_input * dt;
            obj.tas_state = max(obj.tas_state,3);
           
            
        end
        function update_speed_setpoint(obj)
            %Set the airspeed demand to the minimum value if an underspeed or
            %or a uncontrolled descent condition exists to maximise climb rate
            if obj.underspeed_detected
                obj.TAS_setpoint = obj.TAS_min;
            end
            obj.TAS_setpoint = min(max(obj.TAS_setpoint,obj.TAS_min),obj.TAS_max);
            
            %Calculate limits for the demanded rate of change of speed based on physical performance limits
            %with a 50% margin to allow the total energy controller to correct for errors
            velRateMax = 0.5 * obj.STE_rate_max / obj.tas_state;
            velRateMin = 0.5 * obj.STE_rate_min / obj.tas_state;
            
            obj.TAS_setpoint_adj = min(max(obj.TAS_setpoint,obj.TAS_min),obj.TAS_max);
            
            %calculate the demanded rate of change of speed proportional to speed error
            %and apply performance limits
            obj.TAS_rate_setpoint = min(max((obj.TAS_setpoint_adj - obj.tas_state) * obj.airspeed_error_gain,velRateMin),velRateMax);
        end
%         hgt_setpoint,hgt_rate_sp,target_climbrate,target_sinkrate,altitude
        function calculateHeightRateSetpoint(obj,altitude_sp_amsl,height_rate_sp,target_climbrate,target_sinkrate,altitude)
            %TODO
            control_altitude = true;
            input_is_height_rate = isfinite(height_rate_sp);
            % velocity_control_traj_generator.setVelSpFeedback(_hgt_rate_setpoint);
            if input_is_height_rate
                % velocity_control_traj_generator.setCurrentPositionEstimate(altitude_amsl);
                % velocity_control_traj_generator.update(_dt, height_rate_sp);
%                 obj.hgt_rate_setpoint = velocity_control_traj_generator.getCurrentVelocity();
%                 altitude_sp_amsl = velocity_control_traj_generator.getCurrentPosition();
%                 control_altitude(altitude_sp_amsl);
            else
%                 velocity_control_traj_generator.reset(0, obj.hgt_rate_setpoint, obj.hgt_setpoint);
            end
            if control_altitude
%                 runAltitudeControllerSmoothVelocity(altitude_sp_amsl, target_climbrate, target_sinkrate, altitude_amsl);
            else
%                 alt_control_traj_generator.setCurrentVelocity(_hgt_rate_setpoint);
%                 alt_control_traj_generator.setCurrentPosition(altitude_amsl);
%                 hgt_setpoint = altitude_amsl;
            end
            
        end
            function update_height_setpoint(obj,desired,state)
            %Detect first time through and initialize previous value to demand
            if isfinite(desired) && abs(obj.hgt_setpoint_in_prev) < 0.1
                 obj.hgt_setpoint_in_prev = desired;
            end
            if isfinite(desired)
                obj.hgt_setpoint = 0.5 * (desired + obj.hgt_setpoint_in_prev)
            else
                 obj.hgt_setpoint = obj.hgt_setpoint_in_prev;
            end
            obj.hgt_setpoint_prev = obj.hgt_setpoint_in_prev;
            
            %Apply a rate limit to respect vehicle performance limitations
            if (obj.hgt_setpoint - obj.hgt_setpoint_prev) > (obj.max_climb_rate * dt) 
                obj.hgt_setpoint = obj.hgt_setpoint_prev + obk.max_climb_rate * dt;
 
            end
        end
        
        function update_pitch_setpoint(obj,dt)
            % Calculate the specific energy balance rate demand
            SEB_rate_setpoint = obj.SPE_rate_setpoint * obj.SPE_weighting - obj.SKE_rate_setpoint * obj.SKE_weighting;
            % Calculate the specific energy balance rate error
            obj.SEB_rate_error = SEB_rate_setpoint - (obj.SPE_rate * obj.SPE_weighting - obj.SKE_rate * obj.SKE_weighting);
            climb_angle_to_SEB_rate = obj.tas_state * physics.Gravity(3);
            
            if obj.integrator_gain_pitch > 0
                pitch_integ_input = obj.SEB_rate_error * obj.integrator_gain_pitch;
                if obj.pitch_setpoint_unc > obj.pitch_setpoint_max
                     pitch_integ_input = min(pitch_integ_input,0);
                else
                     pitch_integ_input = max(pitch_integ_input,0);
                end
                obj.pitch_integ_state =  obj.pitch_integ_state +  pitch_integ_input * dt;
            else
                obj.pitch_integ_state = 0;
            end
            SEB_rate_correction = obj.SEB_rate_error * obj.pitch_damping_gain + obj.pitch_integ_state + obj.SEB_rate_ff * SEB_rate_setpoint;
            if obj.climbout_mode_active
                SEB_rate_correction = SEB_rate_correction + obj.pitch_setpoint_min * climb_angle_to_SEB_rate;
            end
            obj.pitch_setpoint_unc = SEB_rate_correction / climb_angle_to_SEB_rate;
            pitch_setpoint = constrain(obj.pitch_setpoint_unc,obj.pitch_setpoint_min,obj.pitch_setpoint_max);
            ptchRateIncr = dt * obj.vert_accel_limit / obj.tas_state;
            obj.last_pitch_setpoint = constrain(pitch_setpoint,obj.last_pitch_setpoint-ptchRateIncr,obj.last_pitch_setpoint + ptchRateIncr);
            
        end

        function update_energy_estimate(obj)
            % potential energy
            obj.SPE_setpoint = obj.hgt_setpoint * physics.Gravity(3);
            % kinetic energy
            obj.SKE_setpoint = 0.5 * obj.TAS_setpoint_adj * obj.TAS_setpoint_adj;
            
            % calc total energy error
            obj.STE_error = obj.SPE_setpoint - obj.SPE_estimate + obj.SKE_setpoint - obj.SKE_estimate;
            % calc specific energy balance demand 
            SEB_setpoint = SEBSetpoint(obj.SPE_setpoint,obj.SPE_weighting,obj.SKE_estimate,obj.SKE_weighting);
            obj.SEB_error = SEB_setpoint - (obj.SPE_estimate * obj.SPE_weighting - obj.SKE_estimate * obj.SKE_weighting);
            % calc specific energy rate demands (m**2/sec**3)
            obj.SPE_rate_setpoint = obj.hgt_rate_setpoint * physics.Gravity(3);
            obj.SKE_rate_setpoint = obj.tas_state * obj.TAS_rate_setpoint;
            % calc specific energies in unit of (m**2/sec**2)
            obj.SPE_estimate = obj.vert_pos_state * physics.Gravity(3);
            obj.SKE_estimate = 0.5 * obj.tas_state * obj.tas_state;
            % calculate specific energy rates in units of (m**2/sec**3)
            obj.SPE_rate = obj.vert_vel_state * physics.Gravity(3);
            obj.SKE_rate = obj.tas_state * obj.tas_rate_filtered;
        end
        function update_throttle_setpoint(obj,dt,throttle_cruise)
            % Calculate demanded rate of change of total energy, respecting vehicle limits
            STE_rate_setpoint = obj.SPE_rate_setpoint + obj.SKE_rate_setpoint;
            
            % Calculate the total energy rate error, applying a first order IIR filter
            obj.STE_rate_error_filter.update(-obj.SPE_rate - obj.SKE_rate + obj.SPE_rate_setpoint + obj.SKE_rate_setpoint);
            obj.STE_rate_error = obj.STE_rate_error_filter.getstate();
            % Calculate the throttle demand
            
            STE_rate_setpoint = STE_rate_setpoint + obj.load_factor_correction * (obj.load_factor - 1); %MAY NEED TO CHANGE
            STE_rate_setpoint = constrain(STE_rate_setpoint,obj.STE_rate_min,obj.STE_rate_max);
            if obj.underspeed_detected
                throttle_setpoint = obj.throttle_setpoint_max;
            else
                STE_rate_setpoint = STE_rate_setpoint + obj.load_factor_correction * (obj.load_factor - 1);
                STE_rate_setpoint = constrain(STE_rate_setpoint,obj.STE_rate_min,obj.STE_rate_max);
                throttle_pred = 0;
                % Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle as the starting point
                if STE_rate_setpoint >= 0
                    throttle_pred = throttle_cruise + STE_rate_setpoint / obj.STE_rate_max * (obj.throttle_setpoint_max - throttle_cruise);
                else
                    throttle_pred = throttle_cruise + STE_rate_setpoint / obj.STE_rate_min * (obj.throttle_setpoint_max - throttle_cruise);
                end
                % Calculate gain scaler from specific energy rate error to throttle
                STE_rate_to_throttle = 1 / (obj.STE_rate_max - obj.STE_rate_min); 
                % Add proportional and derivative control feedback to the predicted throttle and constrain to throttle limits
                throttle_setpoint = (obj.STE_rate_error * obj.throttle_damping_gain) * STE_rate_to_throttle + throttle_pred;
                throttle_setpoint = constrain(throttle_setpoint,obj.throttle_setpoint_min,obj.throttle_setpoint_max);
            end
            % Rate limit the throttle demand
            if abs(obj.throttle_slewrate) > 0.01
                throttle_increment_limit = dt * (obj.throttle_setpoint_max - obj.throttle_setpoint_min) * obj.throttle_slewrate;
                throttle_setpoint = constrain(throttle_setpoint,obj.last_throttle_setpoint - throttle_increment_limit, ...
                    obj.last_throttle_setpoint + throttle_increment_limit);
            end
            
            obj.last_throttle_setpoint = constrain(throttle_setpoint,obj.throttle_setpoint_min,obj.throttle_setpoint_max);
        end
        
        
        function update_STE_rate_lim(obj)
            obj.STE_rate_max = obj.max_climb_rate * physics.Gravity(3);
            obj.STE_rate_min = obj.max_sink_rate * physics.Gravity(3);
        end
        
        function detect_underspeed(obj)
            if ~obj.detect_underspeed_enabled
                obj.underspeed_detected = false;
            end
            
            if ((obj.tas_state < (obj.TAS_min * 0.9)) && (obj.last_throttle_setpoint >= (obj.throttle_setpoint_max * 0.95)))...
                    || ((obj.vert_pos_state < obj.hgt_setpoint) && obj.underspeed_detected)
                obj.underspeed_detected = true;
            else
                obj.underspeed_detected = false;
               
            end
        end
        function update_speed_height_weights(obj)
            obj.SKE_weighting = constrain(obj.pitch_speed_weight,0,2);
            if (obj.underspeed_detected || obj.climbout_mode_active)
                obj.SKE_weighting = 2;
            else
                obj.SKE_weighting = 0;
            end
            obj.SPE_weighting = constrain(2-obj.SKE_weighting,0,1);
            obj.SKE_weighting = constrain(obj.SKE_weighting,0,1);
        end
        function handle_alt_step(obj,delta_alt,altitude) % MANUAL CONTROL
            obj.hgt_setpoint = obj.hgt_setpoint + delta_alt;
            obj.vert_pos_state = altitude;
            obj.vert_vel_state = 0;
        end
    end
end

%% Helper functions
function SEB_setpoint = SEBSetpoint(spesp,speweight,skeesp,skeweight)
    SEB_setpoint = spesp * speweight - skeesp * skeweight;
end

function constrainoutput = constrain(x,minval,maxval)
    constrainoutput = min(max(x, minval), maxval);
end