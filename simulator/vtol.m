classdef vtol < multirotor
    properties
        WingDirections = {[0; 1; 0], [0; -1; 0]};
        WingLengths = [1; 1]; % in meters
        WingSurfaceArea = 0.44; % in m^2
        AngleOfIncident = 0; % in degrees
        Wingspan = 2; % in meters
        MeanChord = 0.22; % in meters
        %         C_A = 0.03; %Aileron constant
        %         C_E = 0.03; %Elevator constant
        %         C_R = 0.03; %Rudder constant
        C_A = 0.11730; %Aileron constant
        C_E = 0.55604; %Elevator constant
        C_R = 0.08810; %Rudder constant
        r_w = 0.0625; %wing center of lift wrt, center of gravity in meters
        r_t = 0.6385; %tail center of lift wrt, center of gravity in meters
        S_A = 0.0720; %aileron surface area in m^2
        S_E = 0.03; %elevator surface area in m^2
        S_R = 0.008; %rudder surface area in m^2
        l3 = 0.5189; %front props delta x from center of mass in meters
        l4 = 0.4574; %back props delta x from center of mass in meters
        L0 = 0.3642; %tilting rotors y offset
        fw_p_lim_min = -45;
        fw_p_lim_max = 45;
        fw_t_climb_r_sp = 3;
        fw_t_sink_r_sp = 2;
        is_transition = false;

        % steady fw speed  =  27.7425;
        blending_air_speed = 10;   
        Transition_air_speed = 23; 

%         tilt = [0,0,0,0]';

    end

    properties (SetAccess=protected, GetAccess=public)
        % Add your properties that can only be set internally
    end

    properties (SetAccess=protected, GetAccess=protected)
        % Add your properties that can only be seen internally
    end

    %% Public methods
    methods
        function obj = vtol(ArmAngles, RotationDirections)
            obj = obj@multirotor(ArmAngles, RotationDirections);

        end

        function [wrench, aeromoments] = CalcGeneratedWrench(obj, plantinput)

            wrench = CalcGeneratedWrench@multirotor(obj, plantinput);

            
             u = [obj.State.RotorSpeeds.^2/1.2194e+6;plantinput.AileronLeftRate;plantinput.ElevatorRate;plantinput.RudderRate];
            

%             disp('u')
%             disp(u)
            tilt = obj.State.ServoAngles;

            if( isempty(tilt) )
            tilt = [0,0,0,0]';
            end
%             disp('tilt')
%             disp(tilt)


            Va_i = obj.State.AirVelocity;
            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;

            

           effectiveness_matrix = calc_eff_mat(q_bar, deg2rad(tilt));

%            disp('effectiveness_matrix1')
%            disp(effectiveness_matrix)
            
            test = effectiveness_matrix * u;

%             disp('test')
%             disp(test)

            aeromoments = obj.CalcAerodynamicMoment(obj.State.Velocity);


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% start new lift here %%%%%%%%%%%%%%%%%%

            % add moment

%             wrench(1:3) = wrench(1:3) + obj.CalcDeflectionMoment(obj.State.Velocity, plantinput);

%             F_x = [obj.GetGravityMoment(RBI) - cross(obj.State.Omega, zeros(3,3) * obj.State.Omega);
%                    obj.TotalMass * physics.Gravity];
            

            rbi = obj.GetRotationMatrix();
            rib = rbi';
            cal_force = rib*test(4:6);

            Grav_f = [0; 0; 9.80665] * 7.427;


            wrench = [test(1:3);cal_force+Grav_f];
% 
%              wrench = [0,1,0,0,0,0]';
         
            % add force
            force = obj.CalcAerodynamicForce(obj.State.Velocity);
            %full areo
            wrench(4:6) = wrench(4:6) + force(1:3);

%             disp('calculated wrench')
%             disp(wrench)



        end

        function new_state = CalcNextState(obj, wrench, Thrust_Body,tf_sensor_wrench, ...
                wind_force, plantinput, dt, is_collision, collision_normal, ...
                air_velocity)

            new_state = CalcNextState@multirotor(obj, wrench, Thrust_Body,tf_sensor_wrench, ...
                wind_force, plantinput, dt, is_collision, collision_normal, air_velocity);
            [~, alpha, beta] = CalcWindToBodyRotation(obj, air_velocity);
            new_state.AngleOfAttack = alpha;
            new_state.SideSlipAngle = beta;
            da = min(0.1, abs(plantinput.AileronLeftRate));
            if(plantinput.AileronLeftRate < 0)
                da = -da;
            end
            new_state.AileronLeftPos = obj.State.AileronLeftPos + da;
            new_state.AileronRightPos = -new_state.AileronLeftPos;
            new_state.ElevatorPos = plantinput.ElevatorRate;
            new_state.RudderPos = plantinput.RudderRate;
            new_state.AeroMoments = plantinput.Aeromoment;
        end

    end
    %% Private methods
    methods (Access = private)

        function force = CalcAerodynamicForce(obj, Va_i)

            [rbw, alpha, ~] = obj.CalcWindToBodyRotation(Va_i);

            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;

            c_y = 0;



            C_Z0 = 0.35;
            C_Za = 0.11;
            C_D0 = 0.03;
            C_Da = 0.2;

            c_z = C_Z0 + C_Za * alpha;
            c_x = C_D0 + C_Da * alpha * alpha;

            drag = q_bar * obj.WingSurfaceArea * c_x;
            lateral = q_bar * obj.WingSurfaceArea * c_y;
            lift = q_bar * obj.WingSurfaceArea *c_z;

            %% blending begin

            air_speed_norm = norm(obj.State.Velocity);

            coeff = aero_blending(obj, air_speed_norm);

            drag = coeff * drag;
            lateral = coeff * lateral;
            lift = coeff * lift;

            %% frame change
            rbi = obj.GetRotationMatrix();
            rib = rbi';

            lift = 0;
            drag = 0;

            force = rib * rbw * [-drag; lateral; -lift];


    
        end


        %         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function moment = CalcAerodynamicMoment(obj, Va_i) %TODO
            [rwb, ~, ~] = obj.CalcWindToBodyRotation(Va_i);

            Va_b = rwb*Va_i;

            q_bar = (Va_b' * Va_b) * physics.AirDensity / 2;

            p = obj.State.Omega(1);
            q = obj.State.Omega(2);
            r = obj.State.Omega(3);

            %dimensionless angular rate
            p_tilde = (obj.Wingspan * p) / (2 * norm(Va_b)^2); % why there's a square for Va_b
            q_tilde = (obj.MeanChord * q) / (2 * norm(Va_b)^2);
            r_tilde = (obj.Wingspan * r) / (2 * norm(Va_b)^2);

            roll_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * p_tilde;
            pitch_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * q_tilde;
            yaw_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * r_tilde;
            %             c_i = get_ci(alpha);
            %             c_m = get_cm(alpha);
            %             c_n = get_cn(alpha);
            %
            %             roll_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_i;
            %             pitch_moment = q_bar * obj.WingSurfaceArea * obj.MeanChord * c_m;
            %             yaw_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_n;

            moment = [roll_moment; pitch_moment; yaw_moment];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function moment = CalcDeflectionMoment(obj, Va_i, plantinput)
            [rwb, ~, ~] = obj.CalcWindToBodyRotation(Va_i);
            Va_b = rwb*Va_i;

            q_bar = (Va_b' * Va_b) * physics.AirDensity / 2;
            roll_moment = q_bar * obj.S_A * obj.C_A * plantinput.AileronLeftRate;
            pitch_moment = q_bar * obj.S_E * obj.C_E * plantinput.ElevatorRate;
            yaw_moment = q_bar * obj.S_R * obj.C_R * plantinput.RudderRate;
             
%             disp('1')
%             disp(obj.S_A)
%             disp('2')
%             disp(obj.S_E)
%             disp('3')
%             disp(obj.S_R)

            moment = [roll_moment; pitch_moment; yaw_moment];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [R_WB, alpha, beta] = CalcWindToBodyRotation(obj, Va_i)
            rbi = obj.GetRotationMatrix();
            %air velocity Va_i
            Va_b = rbi * Va_i;



            %Calculate angle of attack a and sideslip angle b
            %             a = CalcAngleOfAttack(Va_b) + obj.AngleOfIncident;
            a = CalcAngleOfAttack(Va_b) ;
            %             a = 10;
            b = 0;

            %Rotation matrix from body to wind

            %             R_BW = [cosd(a) * cosd(b),    sind(b),     sind(a)*cosd(b);
            %                     -sind(b) * cosd(a),   cosd(b),     -sind(a)*sind(b);
            %                     -sind(a)         ,   0  ,        cosd(a)];

            R_BW = [cosd(a)    ,    0  ,     sind(a);
                0          ,    1  ,     0;
                -sind(a)   ,    0  ,     cosd(a)];

            %             R_BW = [cosd(b) * cosd(a),  -sind(b) * cosd(a),     -sind(a)
            %                     sind(b),            cosd(b),                0
            %                     cosd(b) * sind(a),  - sind(b) * sind(a),    cosd(a) ];
            R_WB = R_BW.';

            if nargout > 1
                alpha = a;
                beta = b;
            end
        end

        function coeff = aero_blending(obj, air_speed_norm)

            x = -10:.001:100;

            dist = 2;
            f1 = @(x) 0;
            f2 = @(x)  0.5 * (x-obj.blending_air_speed)/(obj.Transition_air_speed-obj.blending_air_speed);
            foo = blend(f1, f2, 5, dist);

            f3 = @(x) foo(x);
            f4 = 1;
            foo = blend(f3, f4, 20, dist);
            coeff = foo(air_speed_norm);

        end





    end

end

%% Other function

function alpha = CalcAngleOfAttack(Va_b)

alpha = atand(Va_b(3)/Va_b(1));

if isnan(alpha)
    alpha = 0;
end
end

function beta = CalcSideSlipAngle(Va_b)
beta = asind(Va_b(2) / norm(Va_b));
if isnan(beta)
    beta = 0;
end
end

function cl = get_cl(alpha)
data = load('C_l');
cl = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_l,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end
function cd = get_cd(alpha)
data = load('C_d');
cd = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_d,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end

function cm = get_cm(alpha)
data = load('C_m');
cm = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_m,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end

%% piecewise blending
function foo = blend(f1,f2,location,distance)

if nargin < 4
    distance = 0;
end
if nargin < 3 || isempty(location)
    location = 0;
end
validateattributes(location, {'numeric','DimVar'}, {},...
    'blend', 'blending center location', 3);
validateattributes(distance, {'numeric','DimVar'}, {'nonnegative'},...
    'blend', 'blending distance', 4);
if isnumeric(f1)
    f1 = @(~) f1;
end
if isnumeric(f2)
    f2 = @(~) f2;
end
blf = @(x) tanh((x-location)./distance)/2;
foo = @(x) (1/2 - blf(x)).*f1(x) + (1/2 + blf(x)).*f2(x);
end





function effectiveness_matrix = calc_eff_mat(q_bar, tilt)
    
    
    %           1         2        3         4
    
    
    Px = [ 1 * cosd(30) 1 * cosd(150) 1 * cosd(210) 1 * cosd(330)];
    Py = [ 1 * sind(30) 1 * sind(150) 1 * sind(210) 1 * sind(330)];

    Pz = zeros(1, 4);

    Ct = (1.08105e-4 / 5)          * 1.2194e+6;    
    Km = ((1.08105e-4 / 5) * 0.05) * 1.2194e+6;
   

    ro = 1.225;

     b = 1.0; %wing
% 
     S_A = 0.0720; %aileron surface area in m^2
     S_E = 0.03; %elevator surface area in m^2
     S_R = 0.008; %rudder surface area in m^2
% 
     Cla = 0.11730; %Aileron constant
     Cme = 0.55604; %Elevator constant
     Cnr = 0.08810; %Rudder constant


   
    effectiveness_matrix = [-Py(1) * Ct*cos(tilt(1)) +  Km * sin(tilt(1)),		      -Py(2) * Ct*cos(tilt(2)) -  Km * sin(tilt(2)),		   -Py(3) * Ct*cos(tilt(3)) +  Km * sin(tilt(3)),			    -Py(4) * Ct*cos(tilt(4)) -  Km * sin(tilt(4)),			   		  q_bar*S_A*b*Cla,	                0.0, 		 		    0.0;
                             Ct*(Px(1) * cos(tilt(1)) + Pz(1) * sin(tilt(1))),  	  Ct*(Px(2) * cos(tilt(2)) + Pz(2) * sin(tilt(2))),		   Ct*(Px(3) * cos(tilt(3)) + Pz(3) * sin(tilt(3))),			Ct*(Px(4) * cos(tilt(4)) + Pz(4) * sin(tilt(4))),			      0.0, 		 	                                q_bar*S_E*b*Cme,	 	0.0; 			
                            -Py(1) * Ct*sin(tilt(1)) -  Km * cos(tilt(1)),		      -Py(2) * Ct*sin(tilt(2)) +  Km * cos(tilt(2)),		   -Py(3) * Ct*sin(tilt(3)) -  Km * cos(tilt(3)),			    -Py(4) * Ct*sin(tilt(4)) +  Km * cos(tilt(4)),			   	      0.0, 		 	                                0.0, 		 		 	q_bar*S_R*b*Cnr; 	
                             Ct * sin(tilt(1)),	 									  Ct * sin(tilt(2)),			   					       Ct * sin(tilt(3)),											Ct * sin(tilt(4)),										       	  0.0, 		 	                                0.0, 		 	        0.0; 			
                             0.0,  			 										  0.0,  							   			 	       0.0,  													 	0.0,	 												  		  0.0, 		 	                                0.0, 		 	        0.0; 			
                            -Ct * cos(tilt(1)),	 						    		  -Ct * cos(tilt(2)),			   					       -Ct * cos(tilt(3)),											-Ct * cos(tilt(4)),									        	  0.0, 		 	                                0.0, 		 	        0.0]; 			
 
end
