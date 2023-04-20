function [Air_speed,phi,yaw_rate] = get_vel_phi(a)
    %% Properties
    m = 7.427;
    g = 9.81;
    R = 200;
    y = 0;   % yaw
    b = 0;   % sideslip angle
    c_z = 0.35 + 0.11 * deg2rad(a); %C
    c_d = 0.01 + 0.2 * deg2rad(a) * deg2rad(a); %C_D,a from paper 
    WingSurfaceArea = 0.44; % in m^2
    air_density = 1.229;
    ThrustConstant = 1.08105e-4 / 5;    
    %% Calculation
    syms phi Air_speed T
    assume(phi > -30 & phi < 30)
    assume(Air_speed > 0 & Air_speed < 30)
    %% Calculations
    drag = air_density * Air_speed ^ 2 / 2 * WingSurfaceArea * c_d;
    lift = air_density * Air_speed ^ 2 / 2 * WingSurfaceArea * c_z;
    %% Rotation Matrices
    RBW  = [cosd(a) * cosd(b),    sind(b),     sind(a)*cosd(b);
        -sind(b) * cosd(a),   cosd(b),     -sind(a)*sind(b);
        -sind(a)         ,   0  ,        cosd(a)];
    RWB = RBW';
    
    RIB = [cosd(a)*cosd(y)  cosd(a)*sind(y)  -sind(a);
           sind(phi)*sind(a)*cosd(y)-cosd(phi)*sind(y)  sind(phi)*sind(a)*sind(y)+cosd(phi)*cosd(y)   sind(phi)*cosd(a);
           cosd(phi)*sind(a)*cosd(y)+sind(phi)*sind(y)  cosd(phi)*sind(a)*sind(y)-sind(phi)*cosd(y)   cosd(phi)*cosd(a)]; %inertial to body frame
    RBI = RIB';
    BFweight =  RIB * [0;0;m*g];
%     disp(BFweight)
    BFlift = RWB * [0;0;-lift];
    BFdrag = RWB * [-drag;0;0];
    IFlift = RBI * BFlift;
    IFdrag = RBI * BFdrag;
    WBforce = RWB * [-drag;0;-lift];
    BFthrust = [T;0;0];
    IFthrust = RBI * BFthrust;
%     IFT = abs(IFdrag(1)) - abs(IFlift(1));  % NEEDS TO FIX
%     T = RIB * IFT;
    rss = T / (4 * ThrustConstant); % check rotor speed squared

    Bforce = BFthrust + WBforce;

    IFforce = RBI * Bforce; 
    
    %% Solver
    eqn1 = IFforce(3) + m*g == 0;
%     eqn2 = tand(phi) == Air_speed ^ 2 / (g * R * cosd(a));
    eqn2 = IFforce(2) == m * Air_speed ^ 2 / R;
    eqn3 = IFforce(1) == 0;
    sol = solve([eqn1, eqn2, eqn3], [phi,Air_speed,T]);
    Air_speed = double(sol.Air_speed);
    %     Air_speed = Air_speed(Air_speed > 0);
    phi = double(sol.phi);
    T = double(sol.T)

    yaw_rate = Air_speed / R;
    %% Check values
%     IFlift = RBI * BFlift;
%     Iforce = double(subs(BFlift));
%     disp(double(subs(rss)))
%     disp(double(subs(IFforce)))
    disp(double(subs(IFlift)))
%     disp(double(subs(rss)))
%     disp(double(subs(BFthrust)))
%     disp(Iforce)
%     disp(double(subs(RIB)));
%     yaw_rate = Air_speed / R;