classdef controller < handle
    properties
        ControlAllocation control_allocation_vtol
        AttitudeController attitude_controller
        PositionController position_controller_fw_MPC_new
        HMFController hmf_controller

        Max_speed_sq

        mode = "transition"
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

%                  euler_accel = obj.AttitudeController.CalculateControlCommand(mult, rpy_des, [], [], dt);
                
                euler_accel = [0,0,0]';
        end


       function [lin_accel, rpy_des, tilt] = ControlPosition(obj, mult, pos_des, yaw_des, vel_des, acc_des, dt)

             
%                 [lin_accel,rpy_des, tilt] = obj.PositionController.CalculateControlCommand(mult, pos_des, vel_des, yaw_des, acc_des, dt);
                         
              lin_accel= [0,0,0]';
              rpy_des = [0,0,0]';
              tilt = 0;
              
%               RPY = mult.State.RPY;
%               Rbi = GetRotationMatrix(RPY(1),RPY(2),RPY(3));
%               hover = Rbi * physics.Gravity;
%               lin_accel_MC = -hover;
%               lin_accel = [-tand(current_tilt)*lin_accel_MC(3);0;lin_accel_MC(3)]; 

%% begin PID decoy

current_tilt = mult.Servos{1}.CurrentAngle;
speed_norm = norm(mult.State.Velocity) ;
RPY = mult.State.RPY;
Rbi = GetRotationMatrix(RPY(1),RPY(2),RPY(3));

hover = Rbi * physics.Gravity;

lin_accel_MC = -hover;
lin_accel_FW = [1,0,0];

transition_tilt = 27;

lin_accel_transition_P1 = [-tand(current_tilt)*lin_accel_MC(3);0;lin_accel_MC(3)]; 
lin_accel_transition_P2 = [lin_accel_FW(1);0;lin_accel_FW(1)/(-tand(current_tilt))];

transition_z = -200.5;

P = 20;
D = 10;

tilt = 0;


% mode:MC (full control)
if obj.mode == "MC"
    tilt = 0;

    position_err = transition_z - mult.State.Position(3);
    speed_err = 0 - mult.State.Velocity(3);

    lin_accel_z = position_err * P + speed_err * D;

    if lin_accel_z <= -13.8066

        lin_accel_z = -13.8066;

    end


    lin_accel = [0,0,lin_accel_z]';


    if mult.State.Position(3) <= -200
        obj.mode = "transition";
    end
end





% mode:transition (phase1: gaining V_x) 
if obj.mode == "transition"

tilt = transition_tilt;

if speed_norm <= 16

lin_accel = lin_accel_transition_P1;

% mode:transition (phase2: blending)

elseif speed_norm >16 && speed_norm <= 23


lin_accel = lin_accel_transition_P2*(speed_norm - 16)/(23-16) + lin_accel_transition_P1 * (23-speed_norm)/(23-16);

% mode:FW (full control
elseif speed_norm > 23

    obj.mode = "FW";

    tilt = 90;
    lin_accel = lin_accel_transition_P2;


end

end






            
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