classdef position_controller_fw_MPC_new < pid_controller
    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
        VelMax = [35;35;10];
        Q = zeros(6);
        q_x = 0.1;
        c = [1 1 1];
        k_x = 1;
        WingSurfaceArea = 0.44;
        Mass;
        
        q_ref = [5, 0.1, 0.1]';
        alpha = 0.5; % for cost function calculation\

        Q_rpy = diag([0.1 5 0]);
        Q_rpy_dot = diag([0.1 5 0]);
        Q_u = diag([10 10 100 200 50]);
        Q_t = 30;
        
        I_inv = diag([10.685,5.7465,4.6678]);
        R = [2  0   0;
             0  1   0;
             0  0   2];

        acc_Max = [1; 1; 6]; % in m/s^2
        
        U0 = zeros(30,5);
        X0 = zeros(31,14); %, get solution TRAJECTORY
        flag = 1;
        mpciter = 1;
        xx = [];
        tiltIntegral = 0
 
    end


    methods


        function [accel_des,rpy_des,tilt_angle] = CalculateControlCommand(obj, mult, pos_des, vel_des, yaw_des, acc_des, time)


            

            import casadi.*
            
      
            obj.Mass = mult.Mass;
            
            Init_Vel = mult.State.Velocity;
            Init_rpy = deg2rad(mult.State.RPY);
            Init_rpy_dot = deg2rad(mult.State.EulerRate);
            Init_Tilt = deg2rad(mult.Servos{1}.CurrentAngle);
            Init_last_rpy_dot = deg2rad(mult.State.LastEulerRate);
            Init_last_thrust =[sin(Init_Tilt);0;-cos(Init_Tilt)]'* mult.State.LastThrust; % Assuming tilting is the same in all servos
          
            x_ref = [[0;0;-3];[0;0;0];[0;0;0];0;[0;0;0];72];
            vel_des = x_ref(1:3);
            dt = time - obj.LastTime;
            
            % Horizon
            N = 30;
            
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
            
            tilt = SX.sym('tilt');

            
            % Last body rate parametrization
            Roll_dot_last = SX.sym('Roll_dot_last');
            Pitch_dot_last = SX.sym('Pitch_dot_last');
            Yaw_dot_last = SX.sym('Yaw_dot_last');
            last_rpy_dot = [Roll_dot_last; Pitch_dot_last; Yaw_dot_last];
            
            % 1x1
            
            last_thrust = SX.sym('last_thrust');
            
            % 14x1
            state = [V; rpy; rpy_dot;tilt; last_rpy_dot; last_thrust];

            n_states = length(state);
            % 3x1
            Roll_MPC = SX.sym('Roll_MPC');
            Pitch_MPC = SX.sym('Pitch_MPC');
            Yaw_MPC = SX.sym('Yaw_MPC');
            rpy_MPC = [Roll_MPC; Pitch_MPC; Yaw_MPC];
            
            % Tilt speed parametrization
            tilt_dot = SX.sym('tilt_dot');
            % Thrust
            Thrust = SX.sym('Thrust');
            % Control input
            controls= [Thrust;tilt_dot ;rpy_MPC];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            n_controls = length(controls);
            
            F_aero = obj.CalcAeroForce(V,rpy);
            accel = obj.CalcAccel(F_aero,Thrust,rpy,tilt);
            accel = accel + physics.Gravity;

            torque = CalcTorque(rpy_MPC,x_ref(4:6),rpy_dot,last_rpy_dot);
            
            %state update EQ
            rhs = [accel;
                rpy_dot;
                obj.I_inv*(torque);
                tilt_dot;
                last_rpy_dot;
                last_thrust];
            
            f = Function('f',{state,controls},{rhs});  %nonlinear mapping function f(x,u)
            U = SX.sym('U',n_controls,N);
            P = SX.sym('P',n_states + n_states);
            X = SX.sym('X',n_states,(N+1));
            
            
            % A vector that represents the states over the optimization problem
            %        objective_function = J_ref + J_state_input + J_soft;
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % eq contraint
            %init var!!!!!!!!!!!!
            
            st = X(:,1);
            g = []; % constraints vector
            g = [g;st-P(1:14)];  %init constraint
            
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
                obj_stateinput = (st(4:6)-x_ref(4:6))' * obj.Q_rpy * (st(4:6)-x_ref(4:6))...
                    +(st(7:9)-x_ref(7:9))' * obj.Q_rpy_dot * (st(7:9)-x_ref(7:9))...
                    + con' * obj.Q_u * con ;
                obj_soft_tilt = exp(-0.332*Velocity_body(1)*st(10)+12.35*st(10)+(-0.477)*Velocity_body(1)-2.303);
                x = obj.q_ref' * (st(1:3) - vel_des); % before approximation
                
                obj_ref = 2*obj.alpha*log(1 + exp(x/obj.alpha)) - x - 2*obj.alpha*log(2);
                
                
                objective_function = objective_function +  obj_stateinput  + obj_ref + obj_soft_vel+obj_soft_tilt ;
                
                k1 = f(st, con);
                k2 = f(st + h/2*k1,con);
                k3 = f(st + h/2*k2,con);
                k4 = f(st + h*k3, con);
                st_RK4_next = st + h/6* (k1 +2*k2 + 2*k3 +k4);
                % g = [f(N-1) - x(N)]
                g = [g;st_next - st_RK4_next(1:14)];
            end
            
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
            args.lbx(1:n_states:n_states*(N+1),1) = -40; args.ubx(1:n_states:n_states*(N+1),1) = 40; %V in m/s
            args.lbx(2:n_states:n_states*(N+1),1) = -40; args.ubx(2:n_states:n_states*(N+1),1) = 40;
            args.lbx(3:n_states:n_states*(N+1),1) = -10; args.ubx(3:n_states:n_states*(N+1),1) = 10;
            args.lbx(4:n_states:n_states*(N+1),1) = -pi/4; args.ubx(4:n_states:n_states*(N+1),1) = pi/4; %rpy in radian
            args.lbx(5:n_states:n_states*(N+1),1) = -pi/4; args.ubx(5:n_states:n_states*(N+1),1) = pi/4;
            args.lbx(6:n_states:n_states*(N+1),1) = -inf; args.ubx(6:n_states:n_states*(N+1),1) = inf;
            args.lbx(7:n_states:n_states*(N+1),1) = -pi; args.ubx(7:n_states:n_states*(N+1),1) = pi; %rpy_dot in radian
            args.lbx(8:n_states:n_states*(N+1),1) = -pi; args.ubx(6:n_states:n_states*(N+1),1) = pi;
            args.lbx(9:n_states:n_states*(N+1),1) = -pi; args.ubx(9:n_states:n_states*(N+1),1) = pi;
            args.lbx(10:n_states:n_states*(N+1),1) = 0; args.ubx(10:n_states:n_states*(N+1),1) = pi/2; %rpy_dot last set points in degree
            args.lbx(11:n_states:n_states*(N+1),1) = -pi; args.ubx(11:n_states:n_states*(N+1),1) = pi;
            args.lbx(12:n_states:n_states*(N+1),1) = -pi; args.ubx(12:n_states:n_states*(N+1),1) = pi;
            args.lbx(13:n_states:n_states*(N+1),1) = -pi; args.ubx(13:n_states:n_states*(N+1),1) = pi; %thrust in N calculated from max pitch angle and weight
            args.lbx(14:n_states:n_states*(N+1),1) = 0; args.ubx(14:n_states:n_states*(N+1),1) = 80; %thrust in N calculated from max pitch angle and weight

            args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 0; args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 80; %thrust in Newton calculated from max pitch angle and weight
            
            args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/4; args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = pi/4; %rpy_MPC in degree
            args.lbx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/4; args.ubx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = pi/4;
            args.lbx(n_states*(N+1)+4:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/2; args.ubx(n_states*(N+1)+4:n_controls:n_states*(N+1)+n_controls*N,1) = pi/2;
            args.lbx(n_states*(N+1)+5:n_controls:n_states*(N+1)+n_controls*N,1) = -pi/2; args.ubx(n_states*(N+1)+5:n_controls:n_states*(N+1)+n_controls*N,1) = pi/2;

            
            %run mpc
            
            x_state = [Init_Vel ; Init_rpy ; Init_rpy_dot;Init_Tilt;Init_last_rpy_dot;Init_last_thrust];
            u_trim = [72;0;0;0;0];
            
            
            args.p = [x_state;x_ref];
            %             obj.X0 = [Init_Vel ; Init_rpy ; Init_rpy_dot;Init_Tilt;Init_last_rpy_dot;Init_last_thrust]; % initial condition.
            
            
            if  obj.flag == 1
                obj.X0 = repmat(x_ref,1,N+1)';
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
            obj.U0 = [u(2:size(u,1),:);u(size(u,1),:)];
            
            x0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % get solution TRAJECTORY
            
            obj.X0 = [x_state';x0(3:end,:);x0(end,:)];
            
            control = u(1,:)';
            
            
            R_BR =[sin(Init_Tilt);
                     0         ;
                -cos(Init_Tilt)];
            

            thrust_des =  R_BR * control(1);
            
            accel_des  = thrust_des ./ obj.Mass;
            
            rpy_des = rad2deg(control(3:5));
            tilt_speed = control(2);
            obj.LastTime = time;  
            obj.tiltIntegral = obj.tiltIntegral + tilt_speed*dt;
            tilt_angle = rad2deg(Init_Tilt + obj.tiltIntegral);
            

 
  
            
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
            c_d = 0.01 + 0.2 * a_deg * a_deg;
            drag = q_bar * obj.WingSurfaceArea * 0.07;
            lateral = q_bar * obj.WingSurfaceArea * c_y;
            lift = q_bar * obj.WingSurfaceArea * 0.35;

            force = R_i2b'*R_WB * [-drag; lateral;-lift];
            force = [0;0;0];
        end
        
        
        function accel = CalcAccel(obj,F_a,T,rpy,tilt)
            RBR = [sin(tilt);
                0;
                -cos(tilt)];
            
            R_i2b = GetRotationMatrix(rpy(1),rpy(2),rpy(3));
            R_b2i = R_i2b';

            T_inertial =R_b2i* RBR * T;
            
            F_tot = F_a + T_inertial;

            accel = F_tot / obj.Mass;
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

function Torque = CalcTorque(rpyMPC,rpy_ref,rpy_dot,rpy_last) %assuming same thrust and tilt for 4 rotor

body_rate_sp = diag([0.1,2,0.1])* (rpy_ref - rpyMPC);

Torque = diag([0.1,2,0.1]) * (body_rate_sp - rpy_dot); 

end

function [x0, u0] = shift(T, x0, u,f)
st = x0;
con = u(1,:)';
% f_value = f(st,con);
% st = st+ (T*f_value);
k1 = f(st, con);   % new 
k2 = f(st + T/2*k1, con); % new
k3 = f(st + T/2*k2, con); % new
k4 = f(st + T*k3, con); % new
st_next_RK4=st +T/6*(k1 +2*k2 +2*k3 +k4); % new

x0 = full(st_next_RK4);
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end
