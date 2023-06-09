function m = vtol_custom(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [30, 150, 210, 330];
    RotorRotationDirections = [-1, 1, -1, 1];
    RotorDihedralAngle = 0;
    % %                   RotorSidewardAngle = [30, 30, -30, -30];
    % %                   RotorInwardAngle = [-90, 90, 90, -90];



    RotorSidewardAngle = 0;
    RotorInwardAngle = 0;

    m = vtol(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    for i = 1 : m.NumOfRotors
        m.Rotors{i}.ArmLength = 1; % in meters
    end
   
    %%%% 40 max 30 exit speed

    m.VelocityLimits = [40; 40; 40];
    m.TotalSpeedLimit = 50;
    
    m.PayloadRadius = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%this needs to be change to 90 for FW, 0 for MC !!!!!%%%%%%
    m.AddServo([1, 4], [0; -1; 0], 0);
    m.AddServo([2, 3], [0; -1; 0], 0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1 : m.NumOfServos
        m.Servos{i}.MaxRate = 10; % deg/s
        m.Servos{i}.SetMaxAngle();
        m.Servos{i}.SetTiltAngle(90);
    end
    
    
    m.WingDirections{1} = rotz(10) * [0; 1; 0];
    m.WingDirections{2} = rotz(-10) * [0; -1; 0];
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
