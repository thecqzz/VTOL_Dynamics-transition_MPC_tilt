function Air_speed = get_vel(alpha)
    %% Properties
    m = 7.427;
    g = 9.81;
    c_z = 0.35 + 0.11 * deg2rad(alpha); %C
    c_d = 0.01 + 0.2 * deg2rad(alpha) * deg2rad(alpha); %C_D,a from paper 
    WingSurfaceArea = 0.44; % in m^2
    air_density = 1.229;
    %% Calculation
    q_bar = m * g *cosd(alpha) / (WingSurfaceArea* (c_d * sind(alpha) + c_z * cosd(alpha)));
    Air_speed = abs(sqrt(2*q_bar/air_density));
    F_ver = q_bar * c_z * WingSurfaceArea * cosd(alpha) + q_bar * c_d *WingSurfaceArea * sind(alpha) - cosd(alpha) * m*g;
   
    F_hor = q_bar * c_z * WingSurfaceArea * sind(alpha) - q_bar * c_d *WingSurfaceArea * cosd(alpha);
    
