classdef control_allocation_vtol_2 < handle
    properties
    end
    
    properties(SetAccess=protected, GetAccess=public)
        Method control_allocation_types % The control allocation method

%%%
        Max_speed_sq
%%%
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        NDI_L                 % L matrix (related to body-fixed thrust forces)
        NDI_M                 % M matrix (related to body-fixed thrust and reaction moments)
    end

    methods
        function obj = control_allocation_vtol_2(multirotor)
            obj.SetMethod(multirotor, control_allocation_types.NDI);
        end
        
        function SetMethod(obj, multirotor, method)
            obj.Method = method;
        end
        
        function [rotor_speeds_squared, deflections,saturated] = CalcActuators(obj, mult, lin_accel, ang_accel)
        % Calculate the rotor speeds from the desired linear and angular accelerations
            
            persistent lin_accel_last
            persistent ang_accel_last
            persistent rotor_speeds_squared_last
            persistent deflections_last
            persistent saturated_last
            if isempty(lin_accel_last)
                lin_accel_last = zeros(3, 1);
            end
            if isempty(deflections_last)
                deflections_last = zeros(3, 1);
            end
            if isempty(ang_accel_last)
                ang_accel_last = zeros(3, 1);
            end
            if isempty(rotor_speeds_squared_last)
                rotor_speeds_squared_last = zeros(mult.NumOfRotors, 1);
            end
            if isempty(saturated_last)
                saturated_last = false;
            end
            

            % store max speed
            obj.Max_speed_sq = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));
        
            if obj.Method == control_allocation_types.NDI
                [rotor_speeds_squared, deflections] = obj.ActuatorCommands(mult, lin_accel, ang_accel);
            elseif obj.Method == control_allocation_types.Unified_PseudoInv
                [rotor_speeds_squared, deflections] = obj.ActuatorCommands_PseudoInv(mult, lin_accel, ang_accel);
            end

            saturation_flag = false;
            max_rotor_speeds = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared > max_rotor_speeds)
                %mx = max((rotor_speeds_squared - max_rotor_speeds) ./ max_rotor_speeds);
                %rotor_speeds_squared = rotor_speeds_squared - mx * max_rotor_speeds - 1e-5;
                ind = rotor_speeds_squared > max_rotor_speeds;
                rotor_speeds_squared(ind) = max_rotor_speeds(ind);
                saturation_flag = true;
            end
            min_rotor_speeds = cell2mat(cellfun(@(s)s.MinSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared < min_rotor_speeds)
                ind = rotor_speeds_squared < min_rotor_speeds;
                rotor_speeds_squared(ind) = min_rotor_speeds(ind);
                saturation_flag = true;
            end
            
            if nargin > 1
                saturated = saturation_flag;
            end
            lin_accel_last = lin_accel;
            ang_accel_last = ang_accel;
            saturated_last = saturated;
            rotor_speeds_squared_last = rotor_speeds_squared;
        end
    end
    
    %% Private Methods
    methods(Access=protected)


        function rotor_speeds_squared = NDIRotorSpeeds(obj, multirotor, lin_accel, euler_accel)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Initialize the NDI method
        
            % Calculate L matrix (related to body thrust forces)
            obj.NDI_L = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               obj.NDI_L(:, i) = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
            end

%             disp("NDI_L")
%             disp(obj.NDI_L)

            % Calculate G matrix (related to body reaction moments)
            NDI_G = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               NDI_G(:, i) = multirotor.Rotors{i}.GetReactionMomentPerUnitInput();
            end
%             disp("NDI_G")
%             disp(NDI_G)
            
            % Calculate F matrix (related to body thrust moments)
            NDI_F = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
                r = multirotor.Rotors{i}.Position;
                F = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
                NDI_F(:, i) = cross(r, F);
            end

%             disp("NDI_F")
%             disp(NDI_F)
            
            obj.NDI_M = NDI_F + NDI_G;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



        % Calculate the rotor speeds from the desired linear and angular accelerations
        % using NDI method
            
            % Create the desired output matrix y


            y = [lin_accel; euler_accel];

            % Get the rotation matrix
            RBI = multirotor.GetRotationMatrix();
            
            % Calculate eta_dot
            phi = deg2rad(multirotor.State.RPY(1));
            theta = deg2rad(multirotor.State.RPY(2));
            phi_dot = deg2rad(multirotor.State.EulerRate(1));
            theta_dot = deg2rad(multirotor.State.EulerRate(2));
            eta_dot = calc_eta_dot(phi, theta, phi_dot, theta_dot);
            
            % Calculate eta
            eta = [1,   sin(phi)*tan(theta), cos(phi)*tan(theta);
                   0, cos(phi), -sin(phi);
                   0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

            % Calculate the A matrix in y = A + Bu
            NDI_M_Grav = zeros(3, 1);
            for i = 1 : multirotor.NumOfRotors
                r = multirotor.Rotors{i}.Position;
                G_motor = multirotor.Rotors{i}.MotorMass * physics.Gravity;
                G_motorB = RBI * G_motor;
                G_arm = multirotor.Rotors{i}.ArmMass * physics.Gravity;
                G_armB = RBI * G_arm;
                NDI_M_Grav = NDI_M_Grav + cross(r, G_motorB) + cross(r/2, G_armB);
            end
            if multirotor.HasEndEffector()
                r = multirotor.EndEffector.EndEffectorPosition;
                G_eeI = multirotor.EndEffector.EndEffectorMass * physics.Gravity;
                G_eeB = RBI * G_eeI;
                G_armI = multirotor.EndEffector.ArmMass * physics.Gravity;
                G_armB = RBI * G_armI;
                NDI_M_Grav = NDI_M_Grav + cross(r, G_eeB) + cross(r/2, G_armB);
            end
            
            A_moment = eta_dot * multirotor.State.Omega + eta * multirotor.I_inv * ...
                (NDI_M_Grav - cross(multirotor.State.Omega, multirotor.I * multirotor.State.Omega));
            A = [physics.Gravity; A_moment];

% do not counter gravity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            A = zeros(6,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            % Calculate the B matrix

            B_force = RBI' * obj.NDI_L / multirotor.TotalMass;
            B_moment = eta * multirotor.I_inv * obj.NDI_M;
            B = [B_force; B_moment];
            
            % Calculate the rotor speeds
            rotor_speeds_squared = pinv(B) * (y - A); 
        end

        function [rotor_speeds_squared, deflections] = ActuatorCommands(obj, multirotor, lin_accel, euler_accel)
            
          % Calculate the rotor speeds from the desired linear and angular accelerations
        % using NDI method
            
            % Create the desired output matrix y
            y = [lin_accel; euler_accel];
            M_des = (euler_accel' * multirotor.I)';
            F_des = lin_accel * multirotor.TotalMass;
%             [rbw, ~, ~] = multirotor.CalcWindToBodyRotation(multirotor.State.AirVelocity);
            Va_i = multirotor.State.AirVelocity;
            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;


            S_A = 0.0720; %aileron surface area in m^2
            S_E = 0.03; %elevator surface area in m^2
            S_R = 0.008; %rudder surface area in m^2
            Wingspan = 1;
            MeanChord = 1;


            del_a = M_des(1) / (multirotor.C_A * S_A * Wingspan * q_bar);
            del_e = M_des(2) / (multirotor.C_E * S_E * MeanChord * q_bar);
            del_r = M_des(3) / (multirotor.C_R * S_R * Wingspan * q_bar);

%             disp("multirotor.C_A")
%             disp(multirotor.C_A)
%             disp("multirotor.C_E")
%             disp(multirotor.C_E)
%             disp("multirotor.C_R")
%             disp(multirotor.C_R)
            
            del_a = min(max(del_a, -1), 1);
            del_e = min(max(del_e, -1), 1);
            del_r = min(max(del_r, -1), 1);
            
            a = 0.0185;
            b = 35.217;
            
            f1 = min(max(0, a*(q_bar - b) + 0.5), 1);
            del_a = del_a * f1;
            del_e = del_e * f1;
            del_r = del_r * f1;
            
            deflections = [del_a, del_e, del_r];
            
            M_def = [multirotor.C_A * S_A * Wingspan * q_bar * del_a;
                     multirotor.C_E * S_E * MeanChord * q_bar * del_e
                     multirotor.C_R * S_R * Wingspan * q_bar * del_r];
                 
            M_r = M_des - M_def;
            
%             y = [F_des(1); F_des(3);M_r];
%             
%             A1 = [sin(tiltangle), sin(tiltangle), sin(tiltangle), sin(tiltangle)];
%             A2 = [-cos(tiltangle), -cos(tiltangle), -cos(tiltangle), -cos(tiltangle)];
%             A3 = [-multirotor.L0 * cos(tiltangle) + Cq/Ct*sin(tiltangle), -multirotor.L0 * cos(tiltangle) - Cq/Ct*sin(tiltangle), multirotor.L0 * cos(tiltangle) + Cq/Ct*sin(tiltangle), multirotor.L0 * cos(tiltangle) - Cq/Ct*sin(tiltangle)]; 
%             A4 = [-multirotor.l3*cos(tiltangle), multirotor.l4*cos(tiltangle), multirotor.l4*cos(tiltangle), -multirotor.l3*cos(tiltangle)];
%             A5 = [-multirotor.L0 * sin(tiltangle) - Cq/Ct*cos(tiltangle), -multirotor.L0 * sin(tiltangle) + Cq/Ct*cos(tiltangle), multirotor.L0 * sin(tiltangle) - Cq/Ct*cos(tiltangle), multirotor.L0 * sin(tiltangle) + Cq/Ct*cos(tiltangle)];
%             A = [A1;A2;A3;A4;A5];
%             
%             rotor_speeds = y * pinv(A);
            angular_accel_r = (M_r' * multirotor.I_inv)';
            rotor_speeds_squared = NDIRotorSpeeds(obj, multirotor, lin_accel, angular_accel_r);

            for i = 1:4
            rotor_speeds_squared(i) = rotor_speeds_squared(i) / obj.Max_speed_sq(i);
            end

        end
    end
end




%% Other functions
function eta_dot = calc_eta_dot(phi, theta, phi_dot, theta_dot)
    eta_dot_11 = 0;
    eta_dot_12 = sin(phi)*(tan(theta)^2 + 1)*theta_dot + cos(phi)*tan(theta)*phi_dot;
    eta_dot_13 = cos(phi)*(tan(theta)^2 + 1)*theta_dot - sin(phi)*tan(theta)*phi_dot;

    eta_dot_21 = 0;
    eta_dot_22 = -phi_dot*sin(phi);
    eta_dot_23 = -phi_dot*cos(phi);

    eta_dot_31 = 0;
    eta_dot_32 = (cos(phi)*phi_dot)/cos(theta) + (sin(phi)*sin(theta)*theta_dot)/cos(theta)^2;
    eta_dot_33 = (cos(phi)*sin(theta)*theta_dot)/cos(theta)^2 - (sin(phi)*phi_dot)/cos(theta);

    eta_dot = [eta_dot_11 eta_dot_12 eta_dot_13;
               eta_dot_21 eta_dot_22 eta_dot_23;
               eta_dot_31 eta_dot_32 eta_dot_33];
end



