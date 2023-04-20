function [Air_speed,phi,yaw_rate] = get_vel(alpha)
    phi = 0;
    yaw_rate = 0;
    %% Properties
    m = 7.427;
%     m = 4;
    g = 9.81;
    c_z = 0.35 + 0.11 * deg2rad(alpha); %C
    c_d = 0.01 + 0.2 * deg2rad(alpha) * deg2rad(alpha) %C_D,a from paper 
    b = 0;
    y = 0;
    phi = 0;
    WingSurfaceArea = 0.44; % in m^2
    ThrustConstant = 1.08105e-4 / 5; 
    air_density = 1.225;
    a = alpha;
    RBW  = [cosd(a) * cosd(b),    sind(b),     sind(a)*cosd(b);
        -sind(b) * cosd(a),   cosd(b),     -sind(a)*sind(b);
        -sind(a)         ,   0  ,        cosd(a)];
    RWB = RBW';
    RIB = [cosd(a)*cosd(y)  cosd(a)*sind(y)  -sind(a);
           sind(phi)*sind(a)*cosd(y)-cosd(phi)*sind(y)  sind(phi)*sind(a)*sind(y)+cosd(phi)*cosd(y)   sind(phi)*cosd(a);
           cosd(phi)*sind(a)*cosd(y)+sind(phi)*sind(y)  cosd(phi)*sind(a)*sind(y)-sind(phi)*cosd(y)   cosd(phi)*cosd(a)]; %inertial to body frame
    RBI = RIB';
    %% Calculation
    syms Air_speed T
    drag = air_density * Air_speed ^ 2 / 2 * WingSurfaceArea * c_d;
    lift = air_density * Air_speed ^ 2 / 2 * WingSurfaceArea * c_z;
    BFlift = RWB * [0;0;-lift];
    BFdrag = RWB * [-drag;0;0];
    IFlift = RBI * BFlift;
    IFdrag = RBI * BFdrag;
    BFthrust = [T;0;0];
    WBforce = RWB * [-drag;0;-lift];
    Bforce = BFthrust + WBforce;
%     IFthrust = RBI * BFthrust;
    IFforce = RBI * Bforce;
    
    %% Solver
    eqn1 = IFforce(3) + m*g == 0;
    eqn2 = IFforce(1) == 0;
    sol = solve([eqn1, eqn2], [Air_speed,T]);
    Air_speed = double(sol.Air_speed);
    Air_speed = Air_speed(Air_speed > 0);
    T = double(sol.T);
    T = T(1)
    double(subs(IFdrag))
    double(subs(IFlift))
    rss = T / (4 * ThrustConstant);
end