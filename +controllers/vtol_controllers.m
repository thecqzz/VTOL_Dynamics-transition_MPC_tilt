function c = vtol_controllers(mult)

    c = controller(mult);
    
%     c.AttitudeController.SetPID(1000, 5, 600);
%     c.PositionController.SetPID(3, 0, 7);

    c.AttitudeController.SetPID(10, 0, 0.1);
    c.PositionController.SetPID(10, 0, 1);


    c.HMFController.ForceController.SetPID(1, 0, 3);
    c.HMFController.PositionController.SetPID(7, 1, 7);
end
