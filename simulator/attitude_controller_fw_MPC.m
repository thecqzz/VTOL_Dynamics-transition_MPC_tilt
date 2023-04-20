classdef attitude_controller_fw_MPC < handle

    properties
        RateLimits = [70; 70; 30]; % in deg/s
        OutputMax = [8; 8; 8]; % in rad/s^2
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, mult, time)
       
            dt = time - obj.LastTime;
            moment = [M L N]';
            new_omega = mult.State.omega + mult.State.I_inv * moment * dt;
            new_eulerrate = GetEulerRate(mult);
            new_rpy = wrapTo180(mult.State.rpy + new_eulerrate * dt)
            new_Inv = pinv(physics.EstimateInertia(mult));
            
        end
        
        
        function phi_dot = GetEulerRate(mult)
        % Returns the euler rates in degrees   
            
            sphi = sind(mult.State.RPY(1));
            cphi = cosd(mult.State.RPY(1));
            ttheta = tand(mult.State.RPY(2));
            ctheta = cosd(mult.State.RPY(2));
            eta = [1,   sphi*ttheta, cphi*ttheta;
                   0, cphi, -sphi;
                   0, sphi / ctheta, cphi / ctheta];
            phi_dot = rad2deg(eta * mult.State.Omega);
        end
    end
end