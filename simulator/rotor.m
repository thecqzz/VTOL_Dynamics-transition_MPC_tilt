classdef rotor < handle

    properties
        InwardAngle = 0;                        % in degrees
        SidewardAngle = 0;                      % in degrees
        DihedralAngle = 0;                      % in degrees
        ArmAngle = 0;                           % in degrees
        ArmLength = 1;                        % in meters
        Diameter = 13;                          % in inches (used in visualization only)

        TimeConstant = 0.01;                    % for motor in secs
        ConstantGain = 1;                       % for motor
        Kv = 475;                               % in RPM/V (for KDE3510XF-475)
        BatteryVoltage = 22.2;                  % in Volts (for 6-cell lipo)
        
        ThrustConstant = 1.08105e-4 / 5;    
        TorqueConstant = (1.08105e-4 / 5) * 0.05;
        
        PropellerMass = 0.015;                  % For T-Motor 14" * 4.8 propellers
        MotorMass = (0.015) + 0.175;            % in Kilograms (for KDE3510XF-475)
        ESCMass = 0.084;                        % in Kilograms (for KDE-UAS35HVC)
        ArmMass = 0.10 + (0.084);               % in Kilograms
        RotationDirection = 1;                  % -1 for CW, 1 for CCW around Z
                                                % Remember that Z is downward
                                                
        UpperSpeedPercentage = 100;             % The percentage of the maximum 
                                                % rpm that sets the upper speed
                                                % limit (0..100)
        
        LowerSpeedPercentage = 0;               % The percentage of the minimum 
                                                % rpm that sets the lower speed
                                                % limit (0..100)                                           
    end
    
    properties (GetAccess = public, SetAccess = private)
        R_BR                                    % Rotation matrix R_BR
        Position                                % Position of the rotor in B
        MaxSpeedSquared                         % Maximum rotation speed squared in Hz^2
        MinSpeedSquared                         % Minimum rotation speed squared in Hz^2
        RPMLimit                                % Maximum RPM for the motor
    end
    
    methods
        function obj = rotor(rot)
            if nargin < 1
                obj.UpdateStructure();
            else
                obj = rotor();
                obj.CopyFrom(rot);
            end
        end

        function M = GetReactionMoment(obj, rotor_speed_squared)
            rotor_speed_squared = obj.LimitRotorSpeed(rotor_speed_squared);
            M = obj.R_BR * [0; 0; obj.RotationDirection * obj.TorqueConstant ...
                * rotor_speed_squared];
        end

        function M = GetReactionMomentPerUnitInput(obj)
            M = obj.R_BR * [0; 0; obj.RotationDirection * obj.TorqueConstant];
        end

        function F = GetThrustForce(obj, rotor_speed_squared)
            rotor_speed_squared = obj.LimitRotorSpeed(rotor_speed_squared);
            F = obj.R_BR * [0; 0; -obj.ThrustConstant * rotor_speed_squared];
        end

        function F = GetThrustForcePerUnitInput(obj)
            F = obj.R_BR * [0; 0; -obj.ThrustConstant];
        end
        function Throttle = GetCurrentThrottle(obj,rss)
            obj.RPMLimit = obj.Kv * obj.BatteryVoltage;
            ThrottlePercentage = 3000 * sqrt(rss) / (obj.RPMLimit * pi);
            Throttle = ThrottlePercentage * 0.01;
        end
        function [rotor_speed_squared, saturated] = LimitRotorSpeed(obj, rotor_speed_squared)
            flag = false;

            if rotor_speed_squared > obj.MaxSpeedSquared
                rotor_speed_squared = obj.MaxSpeedSquared;
                flag = true;
            end
            if rotor_speed_squared < obj.MinSpeedSquared
                rotor_speed_squared = obj.MinSpeedSquared;
                flag = true;
            end

            if nargout > 1
                saturated = flag;
            end
        end
        
        function set.ArmAngle(obj, value)
            obj.ArmAngle = value;
            obj.UpdateStructure();
        end

        function set.ArmLength(obj, value)
            obj.ArmLength = value;
            obj.UpdateStructure();
        end

        function set.DihedralAngle(obj, value)
            obj.DihedralAngle = value;
            obj.UpdateStructure();
        end

        function set.InwardAngle(obj, value)
            obj.InwardAngle = value;
            obj.UpdateStructure();
        end

        function set.SidewardAngle(obj, value)
            obj.SidewardAngle = value;
            obj.UpdateStructure();
        end

        function set.LowerSpeedPercentage(obj, value)
            obj.LowerSpeedPercentage = value;
            obj.UpdateStructure();
        end

        function set.UpperSpeedPercentage(obj, value)
            obj.UpperSpeedPercentage = value;
            obj.UpdateStructure();
        end

        function set.Kv(obj, value)
            obj.Kv = value;
            obj.UpdateStructure();
        end

        function set.BatteryVoltage(obj, value)
            obj.BatteryVoltage = value;
            obj.UpdateStructure();
        end

        function UpdateStructure(obj)
            obj.R_BR = obj.CalcRotorationMatrix(obj.ArmAngle, obj.InwardAngle, obj.SidewardAngle);
            obj.RPMLimit = obj.Kv * obj.BatteryVoltage;
            obj.MaxSpeedSquared = (obj.UpperSpeedPercentage / 100 * obj.RPMLimit / 30 * pi).^2;
            obj.MinSpeedSquared = (obj.LowerSpeedPercentage / 100 * obj.RPMLimit / 30 * pi).^2;
            obj.Position = obj.GetPosition();
        end
        
        function CopyFrom(obj, rot)
            obj.InwardAngle = rot.InwardAngle;
            obj.SidewardAngle = rot.SidewardAngle;
            obj.DihedralAngle = rot.DihedralAngle;
            obj.ArmAngle = rot.ArmAngle;
            obj.ArmLength = rot.ArmLength;
            obj.TimeConstant = rot.TimeConstant;
            obj.ConstantGain = rot.ConstantGain;
            obj.Kv = rot.Kv;
            obj.BatteryVoltage = rot.BatteryVoltage;
            obj.RPMLimit = rot.RPMLimit;
            obj.ThrustConstant = rot.ThrustConstant;
            obj.TorqueConstant = rot.TorqueConstant;
            obj.PropellerMass = rot.PropellerMass;
            obj.MotorMass = rot.MotorMass;
            obj.ESCMass = rot.ESCMass;
            obj.ArmMass = rot.ArmMass;
            obj.RotationDirection = rot.RotationDirection;
            obj.R_BR = rot.R_BR;
            obj.Position = rot.Position;
            obj.MaxSpeedSquared = rot.MaxSpeedSquared;
            obj.MinSpeedSquared = rot.MinSpeedSquared;
            obj.LowerSpeedPercentage = rot.LowerSpeedPercentage;
            obj.UpperSpeedPercentage = rot.UpperSpeedPercentage;
        end
        
        function SetR_BR(obj, R_BR)
            R = rotz(-obj.ArmAngle) * R_BR;
            y_axis = cross(R(:, 3)', [0, 1, 0]);
            inward_angle = 0;
            if any(y_axis)
                inward_angle = asind(y_axis(3) / norm(y_axis));
            end
            R = roty(-inward_angle) * R;
            sideward_angle = asind(R(2, 3));
            obj.InwardAngle = inward_angle;
            obj.SidewardAngle = sideward_angle;
            
            if obj.R_BR(:, 3)' * R_BR(:, 3) < 0
                obj.SidewardAngle = wrapTo180(obj.SidewardAngle + 180);
            end
        end
        
        function SetPosition(obj, position)
            rbr = obj.R_BR;
            obj.ArmLength = norm(position);
            obj.DihedralAngle = asind(-position(3) / obj.ArmLength);
            obj.ArmAngle = atan2d(position(2), position(1));
            obj.SetR_BR(rbr);
        end
        
        function SetRotorAxis(obj, axis)
            rot_mat = vrrotvec2mat(vrrotvec([0; 0; 1], axis));
            obj.SetR_BR(rot_mat);
        end
    end

    methods (Static)
        function R_BR = CalcRotorationMatrix(arm_angle, inward_angle, sideward_angle)
            rotorZB = rotz(arm_angle + 90);
            rotorXp = rotx(inward_angle);
            rotorYpp = roty(sideward_angle);

            R_BR = rotorZB * rotorXp * rotorYpp;
        end
    end
    
    methods (Access = private)

        function r = GetPosition(obj)
            rx = obj.ArmLength * cosd(obj.DihedralAngle) * cosd(obj.ArmAngle);
            ry = obj.ArmLength * cosd(obj.DihedralAngle) * sind(obj.ArmAngle);
            rz = -obj.ArmLength * sind(obj.DihedralAngle);
            r = [rx; ry; rz];

        end
        
    end
end