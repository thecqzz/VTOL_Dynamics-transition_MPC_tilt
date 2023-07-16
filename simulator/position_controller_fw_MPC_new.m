classdef position_controller_fw_MPC_new < pid_controller
    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
        VelMax = [35;35;10];
        c = [1 1 1];
        k_x = 1;
        WingSurfaceArea = 0.44;
        Mass;
        q_ref = [5, 0.1, 0.1]';
        Q_v = diag([20;10;50])
        Q_rpy = diag([10 20 10]);
        Q_rpy_dot = diag([10 10 10]);
        Q_u = diag([0.0025 0.0025 0.0025 0.0025 1 1 1 1 10 10 10]);
        Q_t = 40;
        
        I_inv = diag([10.685,5.7465,4.6678]);
        R = [2  0   0;
             0  1   0;
             0  0   2];
        
        U0 = zeros(25,11);
        X0 = zeros(26,13); %, get solution TRAJECTORY
        flag = 1;
        mpciter = 1;
        xx = [];
        tiltIntegral = 0
        tilt_angle_max = pi/2
        tilt_angle_min = 0
        % steady fw speed  =  27.7425;
        blending_air_speed = 10;   
        Transition_air_speed = 23; 
    end

    properties
        rotor_position = [[0.866;0.5;0],[0.866;-0.5;0],[-0.866;-0.5;0],[-0.866;0.5;0]];
        RotorRotationDirections = [1, -1, 1, -1];
        rot = rotor;
    end


    methods



        function [accel_des,rpy_des,tilt_angle_des,V_des] = CalculateControlCommand(obj, mult, pos_des, vel_des, yaw_des, acc_des, time)

            import casadi.*
            
            obj.Mass = mult.Mass;
            
            Init_Vel = mult.State.Velocity;
            Init_rpy = deg2rad(mult.State.RPY);
            Init_rpy_dot = deg2rad(mult.State.EulerRate);
            Init_Tilt_1 = deg2rad(mult.Servos{1}.CurrentAngle);
            Init_Tilt_2 = deg2rad(mult.Servos{2}.CurrentAngle);
            Init_Tilt_3 = deg2rad(mult.Servos{3}.CurrentAngle);
            Init_Tilt_4 = deg2rad(mult.Servos{4}.CurrentAngle);
            Init_Tilt = [Init_Tilt_1;Init_Tilt_2;Init_Tilt_3;Init_Tilt_4];
            
            RotorSpeeds = mult.State.RotorSpeeds;

            %Init_last_rpy_dot = deg2rad(mult.State.LastEulerRate);
         
            %x_ref = [[10;0;0];[0;0;0];[0;0;0];0;[0;0;0]];
            dt = time - obj.LastTime;
            
            % Horizon
            N = 25;
            
            % Velocity
            V_x = SX.sym('V_x');
            V_y = SX.sym('V_y');
            V_z = SX.sym('V_z');
            V = [V_x; V_y; V_z];
            
            % Attitude
            Roll = SX.sym('Roll');
            Pitch = SX.sym('Pitch');
            Yaw = SX.sym('Yaw');
            rpy = [Roll; Pitch; Yaw];
            
            % Body rate setpoint parametrization
            Roll_dot = SX.sym('Roll_dot');
            Pitch_dot = SX.sym('Pitch_dot');
            Yaw_dot = SX.sym('Yaw_dot');
            rpy_dot = [Roll_dot; Pitch_dot; Yaw_dot];
            
            tilt_angle_1 = SX.sym('tilt_angle_1');
            tilt_angle_2 = SX.sym('tilt_angle_2');
            tilt_angle_3 = SX.sym('tilt_angle_3');
            tilt_angle_4 = SX.sym('tilt_angle_4');
            tilt = [tilt_angle_1;tilt_angle_2;tilt_angle_3;tilt_angle_4];

            
            % Last body rate parametrization
%             Roll_dot_last = SX.sym('Roll_dot_last');
%             Pitch_dot_last = SX.sym('Pitch_dot_last');
%             Yaw_dot_last = SX.sym('Yaw_dot_last');
%             last_rpy_dot = [Roll_dot_last; Pitch_dot_last; Yaw_dot_last];
                                    
            % 13x1
            state = [V; rpy; rpy_dot;tilt];
            n_states = length(state);
            % 3x1
            Roll_MPC = SX.sym('Roll_MPC');
            Pitch_MPC = SX.sym('Pitch_MPC');
            Yaw_MPC = SX.sym('Yaw_MPC');
            rpy_MPC = [Roll_MPC; Pitch_MPC; Yaw_MPC];
            
            % Tilt speed parametrization
            tilt_1_dot = SX.sym('tilt_1_dot');
            tilt_2_dot = SX.sym('tilt_2_dot');
            tilt_3_dot = SX.sym('tilt_3_dot');
            tilt_4_dot = SX.sym('tilt_4_dot');
            tilt_dot = [tilt_1_dot;tilt_2_dot;tilt_3_dot;tilt_4_dot];
            % Thrust
            Thrust_1 = SX.sym('Thrust_1');
            Thrust_2 = SX.sym('Thrust_2');
            Thrust_3 = SX.sym('Thrust_3');
            Thrust_4 = SX.sym('Thrust_4');
            Thrust = [Thrust_1;Thrust_2;Thrust_3;Thrust_4];
           
            % Control input 1x11
            controls= [Thrust;tilt_dot;rpy_MPC];
            n_controls = length(controls);
            
            F_aero = obj.CalcAeroForce(V,rpy);
            accel = obj.CalcAccel(F_aero,Thrust,rpy,tilt);
            accel = accel + physics.Gravity;

            torque = CalcTorque(obj,Thrust,tilt,RotorSpeeds);           
            %state update EQ
            rhs = [accel;
                rpy_dot;
                obj.I_inv*(torque);
                tilt_dot];
                

            f = Function('f',{state,controls},{rhs});  %nonlinear mapping function f(x,u)
            U = SX.sym('U',n_controls,N);
            P = SX.sym('P',n_states + N*n_states);
            X = SX.sym('X',n_states,(N+1));
            
            
            % A vector that represents the states over the optimization problem
            %        objective_function = J_ref + J_state_input + J_soft
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % eq contraint
            %init var
            st = X(:,1);
            g = []; % constraints vector
            g = [g;st-P(1:13)];  %init constraint
            
            objective_function = 0;
            
            h = dt;
            for k = 1:N
                st = X(:,k);
                con = U(:,k);
                st_next = X(:,k+1);
                R_I2B = GetRotationMatrix(st(4),st(5),st(6));
                Velocity_body = R_I2B*st(1:3);
                obj_soft_vel = obj.q_ref(1)*(exp(-3*Velocity_body(1) + 1) + obj.k_x*Velocity_body(1) - obj.c(1))...
                    + obj.q_ref(2)*(exp((-Velocity_body(2)- 2)) + exp(Velocity_body(2) - 2) - obj.c(2))...
                    + obj.q_ref(3)*(exp((-Velocity_body(3) - 2)) + exp(Velocity_body(3) - 2) - obj.c(3));
                 
                obj_stateinput = (st(1:3)-P(n_states*k+1:n_states*k+3))' * obj.Q_v * (st(1:3)-P(n_states*k+1:n_states*k+3)) +...
                                    (st(4:6))' * obj.Q_rpy * (st(4:6))...
                                   +(st(7:9))' * obj.Q_rpy_dot * (st(7:9))...
                                   + con' * obj.Q_u * con ; 
                
                obj_soft_tilt = exp(-0.332*Velocity_body(1)*(st(10)*180/pi)+1.8*(st(10)*180/pi)+(-0.477)*Velocity_body(1)-2.303) +...
                    exp(-0.332*Velocity_body(1)*(st(11)*180/pi)+1.8*(st(11)*180/pi)+(-0.477)*Velocity_body(1)-2.303) +...
                    exp(-0.332*Velocity_body(1)*(st(12)*180/pi)+1.8*(st(12)*180/pi)+(-0.477)*Velocity_body(1)-2.303) +...
                    exp(-0.332*Velocity_body(1)*(st(13)*180/pi)+1.8*(st(13)*180/pi)+(-0.477)*Velocity_body(1)-2.303);

                objective_function = objective_function +  obj_stateinput  + obj_soft_vel+obj_soft_tilt;
                
                k1 = f(st, con);
                k2 = f(st + h/2*k1,con);
                k3 = f(st + h/2*k2,con);
                k4 = f(st + h*k3, con);
                st_RK4_next = st + h/6* (k1 +2*k2 + 2*k3 +k4);
                g = [g;st_next - st_RK4_next(1:13)];
            end
                objective_function = objective_function + (st(1:3)-P(n_states*N+1:n_states*N+3))' * obj.Q_v * (st(1:3)-P(n_states*N+1:n_states*N+3));
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
            
            
            % make the decision variables one column vector
            OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];
            nlp_prob = struct('f',objective_function,'x',OPT_variables,'g',g,'p',P);
            opts = struct;
            opts.ipopt.max_iter = 60000;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.acceptable_tol = 1e-8;
            opts.ipopt.acceptable_obj_change_tol = 1e-6;
            solver = nlpsol('solver','ipopt',nlp_prob,opts);
            %------------------------------------------------------
            args = struct;
            % equality constraints
            
            args.lbg(1:n_states*(N+1)) = -1e-20;
            args.ubg(1:n_states*(N+1)) = 1e-20;
            
            % input and states constraints
            args.lbx(1:n_states:n_states*(N+1),1) = -30; args.ubx(1:n_states:n_states*(N+1),1) = 30; %V in m/s
            args.lbx(2:n_states:n_states*(N+1),1) = -30; args.ubx(2:n_states:n_states*(N+1),1) = 30;
            args.lbx(3:n_states:n_states*(N+1),1) = -10; args.ubx(3:n_states:n_states*(N+1),1) = 10;
            args.lbx(4:n_states:n_states*(N+1),1) = -pi/4; args.ubx(4:n_states:n_states*(N+1),1) = pi/4; %rpy in radian
            args.lbx(5:n_states:n_states*(N+1),1) = -pi/4;   args.ubx(5:n_states:n_states*(N+1),1) = pi/10;
            args.lbx(6:n_states:n_states*(N+1),1) = -inf; args.ubx(6:n_states:n_states*(N+1),1) = inf;
            args.lbx(7:n_states:n_states*(N+1),1) = -pi; args.ubx(7:n_states:n_states*(N+1),1) = pi; %rpy_dot in radian
            args.lbx(8:n_states:n_states*(N+1),1) = -pi; args.ubx(8:n_states:n_states*(N+1),1) = pi;
            args.lbx(9:n_states:n_states*(N+1),1) = -pi; args.ubx(9:n_states:n_states*(N+1),1) = pi;
            args.lbx(10:n_states:n_states*(N+1),1) = 0; args.ubx(10:n_states:n_states*(N+1),1) = pi/2; % tilt in degree
            args.lbx(11:n_states:n_states*(N+1),1) = 0; args.ubx(11:n_states:n_states*(N+1),1) = pi/2;
            args.lbx(12:n_states:n_states*(N+1),1) = 0; args.ubx(12:n_states:n_states*(N+1),1) = pi/2;
            args.lbx(13:n_states:n_states*(N+1),1) = 0; args.ubx(13:n_states:n_states*(N+1),1) = pi/2; 

            args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 0; args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 90; %thrust in Newton calculated from max pitch angle and weight
            args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 0; args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 90;
            args.lbx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = 0; args.ubx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = 90;
            args.lbx(n_states*(N+1)+4:n_controls:n_states*(N+1)+n_controls*N,1) = 0; args.ubx(n_states*(N+1)+4:n_controls:n_states*(N+1)+n_controls*N,1) = 90;

            args.lbx(n_states*(N+1)+5:n_controls:n_states*(N+1)+n_controls*N,1) = -inf; args.ubx(n_states*(N+1)+5:n_controls:n_states*(N+1)+n_controls*N,1) = inf; %tilt_speed in rad
            args.lbx(n_states*(N+1)+6:n_controls:n_states*(N+1)+n_controls*N,1) = -inf; args.ubx(n_states*(N+1)+6:n_controls:n_states*(N+1)+n_controls*N,1) = inf;
            args.lbx(n_states*(N+1)+7:n_controls:n_states*(N+1)+n_controls*N,1) = -inf; args.ubx(n_states*(N+1)+7:n_controls:n_states*(N+1)+n_controls*N,1) = inf;
            args.lbx(n_states*(N+1)+8:n_controls:n_states*(N+1)+n_controls*N,1) = -inf; args.ubx(n_states*(N+1)+8:n_controls:n_states*(N+1)+n_controls*N,1) = inf;

            args.lbx(n_states*(N+1)+9:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/4; args.ubx(n_states*(N+1)+9:n_controls:n_states*(N+1)+n_controls*N,1) = pi/4; %rpy_MPC
            args.lbx(n_states*(N+1)+10:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/4; args.ubx(n_states*(N+1)+10:n_controls:n_states*(N+1)+n_controls*N,1) = pi/10;
            args.lbx(n_states*(N+1)+11:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/4; args.ubx(n_states*(N+1)+11:n_controls:n_states*(N+1)+n_controls*N,1) = pi/4;

            
            %run mpc
            
            x_state = [Init_Vel ; Init_rpy ; Init_rpy_dot;Init_Tilt];
            u_trim = [9.81*obj.Mass.*ones(4,1);zeros(7,1)];
            
            args.p(1:n_states) = x_state;
%%                 ref tracking
            for k = 1:N %new - set the reference to track
                t_predict = time + (k-1)*dt; % predicted time instant
                Vx_ref = 2*t_predict; Vy_ref = 0; Vz_ref = 0;
     
                if Vx_ref >= 27.7425 % the trajectory end is reached
                    Vx_ref = 27.7425; Vy_ref = 0; Vz_ref = 0;
  
                end
                args.p(n_states*k+1:n_states*k+3) = [Vx_ref; Vy_ref; Vz_ref];
                args.p(n_states*k+4:n_states*k+13) = zeros(1,10);
                V_des =  [Vx_ref; Vy_ref; Vz_ref];
            end

            if  obj.flag == 1
                obj.X0 = repmat(x_state,1,N+1)';
                obj.U0 = repmat(u_trim,1,N)';
                obj.flag = 2;
            end
            reshape_a = reshape(obj.X0',n_states*(N+1),1);
            reshape_b = reshape(obj.U0',n_controls*N,1) ;
            
            args.x0 = [reshape_a; reshape_b];
            
            sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
                'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
            
            solver.stats().return_status
            
            u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)'; % get controls only from the solution
            [x_state, obj.U0] = shift(dt, x_state, u,f);
    
            obj.U0 = [u(2:size(u,1),:);u(size(u,1),:)];
            x0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % get solution TRAJECTORY

            disp(x_state')

            obj.X0 = [x0(2:end,:);x0(end,:)];
            control = u(1,:)';
            thrust_MPC = control(1:4);

            accel_des = obj.getAccel_des(Init_Tilt,thrust_MPC);
            
            rpy_des = rad2deg(control(9:11));

            tilt_speed = control(5:8);
            tilt_angle_des = getTilt_angle(Init_Tilt,tilt_speed,dt);
            
            disp(rpy_des)
            disp(thrust_MPC)
            
            disp(rad2deg(tilt_angle_des))

            obj.LastTime = time;

        end
    
    

  
        function accel_des = getAccel_des(obj,Init_Tilt,thrust_MPC)
            n_rotor = length(Init_Tilt);
            thrust_des = zeros(3,1);
            for i = 1:n_rotor
                R_r2b = [sin(Init_Tilt(i));
                              0 ;
                        -cos(Init_Tilt(i))];
                thrust_des = thrust_des + R_r2b*thrust_MPC(i);
            end
            accel_des = thrust_des./obj.Mass;
        end

        
        function force = CalcAeroForce(obj,V,rpy)
            %         V = V';
            %         a = atand(V(3)/V(1)) + obj.aoi;
            R_i2b = GetRotationMatrix(rpy(1),rpy(2),rpy(3));
            V_b = R_i2b*V;
            
            a = atan(V_b(3)/V_b(1)); %angle attack;
            b = 0;
            
            R_BW = [cos(a) * cos(b),    sin(b),     sin(a)*cos(b);
                   -sin(b) * cos(a),   cos(b),     -sin(a)*sin(b);
                      -sin(a)         ,   0  ,        cos(a)];
            R_WB = R_BW';
            
            V_air = R_BW*V_b;
            
            a_deg = a*180/pi;
  
            q_bar = (V_air' * V_air) * physics.AirDensity / 2;
            c_y = 0;
            c_z = 0.35 + 0.11 * a_deg;
            c_d = 0.03 + 0.2 * a_deg * a_deg;
            drag = q_bar * obj.WingSurfaceArea * c_d;
            lateral = q_bar * obj.WingSurfaceArea * c_y;
            lift = q_bar * obj.WingSurfaceArea *  c_z;

            air_speed_norm = norm(V_air);
            coeff = aero_blending(obj, air_speed_norm);
            
            drag = coeff * drag;
            lateral = coeff * lateral;
            lift = coeff * lift;

            force = R_WB * [-drag; lateral;-lift];
        end
        
        
        function accel = CalcAccel(obj,F_a,T,rpy,tilt)
            n_rotor = length(tilt);
            T_body = zeros(3,1);
            for i = 1:n_rotor
                R_r2b = [sin(tilt(i));
                              0 ;
                        -cos(tilt(i))];
                T_body = T_body + R_r2b*T(i);
            end
            
            R_i2b = GetRotationMatrix(rpy(1),rpy(2),rpy(3));
            R_b2i = R_i2b';
            F_tot = F_a + T_body;
            F_Inertial = R_b2i * F_tot;
            accel = F_Inertial / obj.Mass;
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



%% Helper functions

%R_i2b
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

function Torque = CalcTorque(obj,Thrust,tilt,rotor_speed)
% PD controller
% body_rate_sp = diag([5,5,0])* (rpyMPC - rpy);
% Torque = diag([5,5,0]) * (body_rate_sp - rpy_dot) + diag([2,2,0]) * (last_rpy_dot - rpy_dot);
T_r = zeros(3, 1);
T_g = zeros(3,1);
for i = 1:length(tilt)
    R_r2b = [sin(tilt(i)); 0 ;-cos(tilt(i))];
    r = obj.rotor_position(:,i);
    T = R_r2b*Thrust(i);
    T_r = T_r + cross(r,T);
    T_g =  T_g + obj.rot.TorqueConstant*(-1)^(obj.RotorRotationDirections(i))*(rotor_speed(i))^2*R_r2b;
end
Torque = T_r + T_g;
end

function [x_state, u0] = shift(T, x_state, u,f)
st = x_state;
con = u(1,:)';
k1 = f(st, con);   % new 
k2 = f(st + T/2*k1, con); % new
k3 = f(st + T/2*k2, con); % new
k4 = f(st + T*k3, con); % new
st_next_RK4=st +T/6*(k1 +2*k2 +2*k3 +k4); % new
x_state = full(st_next_RK4);
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end

function tilt_angle_bound = limit_tilt(obj,tilt_angle_not_bound)
if tilt_angle_not_bound > obj.tilt_angle_max
    tilt_angle_bound = obj.tilt_angle_max;
elseif tilt_angle_not_bound < obj.tilt_angle_min
    tilt_angle_bound = obj.tilt_angle_min;
else
    tilt_angle_bound = tilt_angle_not_bound;
end
end

function tilt_rk4 = getTilt_angle(tilt_angle,tilt_speed,h)
tilt_rk4 = zeros(4,1);
for i = 1:length(tilt_speed)
    k1 = tilt_speed(i);
    k2 = (tilt_speed(i) + h*k1/2);
    k3 = (tilt_speed(i) + h*k2/2);
    k4 = (tilt_speed(i) + h*k3);
    tilt_rk4(i) = tilt_angle(i) + h*(k1 + 2*k2+ 2+k3 +k4)/6;
end
tilt_rk4 = rad2deg(tilt_rk4);
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
