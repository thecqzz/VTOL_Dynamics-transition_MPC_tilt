classdef position_controller_fw_MPC < pid_controller
    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
        Q = zeros(6);

%%%%%%%%
        q_ref = [5, 5, 10]';
        alpha = 0.1; % for cost function calculation\

        Q_rpy = diag([20 20 0]);
        Q_rpy_dot = diag([5 5 0]);
        Q_u = diag([0.0025 1 100 200 50]);
        q_t = 40;
        
        I_inv = diag([10.685,5.7465,4.6678]);
        R = [2  0   0;
             0  1   0;
             0  0   2];
        aoi = 10;
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    methods

    function [lin_accel,rpy_des] = CalculateControlCommand(obj, mult, pos_des,yaw_des, vel_des, acc_des, time)
        import casadi.*
        if isempty(vel_des)
                vel_des = zeros(3, 1);
        end
        if isempty(acc_des)
            acc_des = zeros(3, 1);
        end
        
        dt = time - obj.LastTime;
        T = dt;
        % Horizon
        N = 10;
        
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
        
        % Tilt angle parametrization 
        tilt_angle = SX.sym('tilt_angle');
        
        % Last body rate parametrization
        Roll_dot_last = SX.sym('Roll_dot_last');
        Pitch_dot_last = SX.sym('Pitch_dot_last');
        Yaw_dot_last = SX.sym('Yaw_dot_last');
        last_rpy_dot = [Roll_dot_last; Pitch_dot_last; Yaw_dot_last];
        
        % 1x1

        last_thrust = SX.sym('last_thrust');

        % 15x1
        state = [V; rpy; rpy_dot; tilt_angle; rpy_dot_last_setpoint; last_thrust];
        n_states = length(state);
        % 3x1
        Roll_MPC = SX.sym('Roll_MPC');
        Pitch_MPC = SX.sym('Pitch_MPC');
        Yaw_MPC = SX.sym('Yaw_MPC');
        rpy_MPC = [Roll_MPC; Pitch_MPC; Yaw_MPC];
        
        % Tilt speed parametrization
        tilt_speed = SX.sym('tilt_speed'); 

        % Control input
        controls= [Thrust; tilt_speed; rpy_MPC];
        
        V_sp = %%%
           
        % reference tracking
        J_ref = norm(obj.q_ref' * (V - V_sp)); % before approximation
        J_ref = 2*obj.alpha*log(1 + exp(J_ref/obj.alpha)) - J_ref - 2*obj.alpha*log(2);

        % state and input cost
        J_state_input = rpy'*obj.Q_rpy*rpy+ rpy_dot'*obj.Q_rpy_dot*rpy_dot + u'*obj.Q_u*u +obj.q_t*()^2;
    
        % soft constraints
        
        n_controls = length(controls);

        F_aero = obj.CalcAeroForce(V);
        
        accel = obj.CalInertialaccel(mult,F_aero,Thrust);
        accel = accel + physics.Gravity;
        rpy_sp = rpy_MPC + [0;0;Yaw];
        body_rate_sp = diagdiag([2,2,2]) * (rpy_sp - rpy);
        torque = diag([2,2,2]) * (body_rate_sp - rpy_dot) + diag([1,1,1]) * (last_rpy_dot - rpy_dot);
        
        
        rhs = [accel;
               body_rate_dot;
               obj.I_inv*(torque);
               tilt_speed;
               last_rpy_dot;
               last_thrust];
        
        f = Function('f',{states,controls},{rhs});  %nonlinear mapping function f(x,u)
        U = SX.sym('U',n_controls,N);
        P = SX.sym('P',n_states + n_states);
        X = SX.sym('X',n_states,(N+1));
        % A vector that represents the states over the optimization problem
        objective_function = J_ref + J_state_input + J_soft;
        g = []; % constraints vector
        x0 = [mult.State.Position;mult.State.Velocity]; %starting point
        xs = [pos_des;vel_des];

        %Choose Q value depends on the current error on rpy
        %state_diff 1-3 shows error on rpy,Q(1-3) weights error on rpy
        %Q(4-6) weights error on rpy_dot
  
        st = X(:,1);
        g = [g;st-P(1:6)];
        for k = 1:N
            st = X(:,k);
            con = U(:,k);
            objective_function = objective_function + (st - P((n_states+1):(n_states+n_states)))'...
                *obj.Q*(st-P((n_states+1):(n_states+n_states))) + con'*obj.R*con;
            st_next = X(:,k+1);
            f_val = f(st,con);
            st_next_euler = st + (T*f_val);
            g = [g;st_next - st_next_euler]; % compute constraints
        end
        
        % make the decision variables one column vector
        OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];
        nlp_prob = struct('f',objective_function,'x',OPT_variables,'g',g,'p',P);
        opts = struct;
        opts.ipopt.max_iter = 100;
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
        args.lbx(1:6:n_states*(N+1)-5,1) = -inf; args.ubx(1:6:n_states*(N+1)-5,1) = inf;
        args.lbx(2:6:n_states*(N+1)-4,1) = -inf; args.ubx(2:6:n_states*(N+1)-4,1) = inf;
        args.lbx(3:6:n_states*(N+1)-3,1) = -inf; args.ubx(3:6:n_states*(N+1)-3,1) = inf;

        %--------------------------------------------------------------------------
        % input constraints

        args.lbx(1:14:14*(N+1),1) = -40; args.ubx(1:14:14*(N+1),1) = 40; %V in m/s
        args.lbx(2:14:14*(N+1),1) = -40; args.ubx(2:14:14*(N+1),1) = 40; 
        args.lbx(3:14:14*(N+1),1) = -10; args.ubx(3:14:14*(N+1),1) = 10;
        args.lbx(4:14:14*(N+1),1) = -45; args.ubx(4:14:14*(N+1),1) = 45; %rpy in degree
        args.lbx(5:14:14*(N+1),1) = -45; args.ubx(5:14:14*(N+1),1) = 45;
        args.lbx(6:14:14*(N+1),1) = -inf; args.ubx(6:14:14*(N+1),1) = inf;
        args.lbx(7:14:14*(N+1),1) = -180; args.ubx(7:14:14*(N+1),1) = 180; %rpy_dot in degree
        args.lbx(8:14:14*(N+1),1) = -180; args.ubx(6:14:14*(N+1),1) = 180;
        args.lbx(9:14:14*(N+1),1) = -180; args.ubx(9:14:14*(N+1),1) = 180;
        args.lbx(10:14:14*(N+1),1) = -7; args.ubx(10:14:14*(N+1),1) = 90; %servo angle in degree
        args.lbx(11:14:14*(N+1),1) = -180; args.ubx(11:14:14*(N+1),1) = 180; %rpy_dot last set points in degree
        args.lbx(12:14:14*(N+1),1) = -180; args.ubx(12:14:14*(N+1),1) = 180;
        args.lbx(13:14:14*(N+1),1) = -180; args.ubx(13:14:14*(N+1),1) = 180;
        args.lbx(14:14:14*(N+1),1) = 0; args.ubx(14:14:14*(N+1),1) = 105; %thrust in N calculated from max pitch angle and weight

        args.lbx(14*(N+1)+1:5:14*(N+1)+5*N,1) = 0; args.ubx(14*(N+1)+1:5:14*(N+1)+5*N,1) = 105; %thrust in N calculated from max pitch angle and weight
        args.lbx(14*(N+1)+2:5:14*(N+1)+5*N,1) = -45; args.ubx(14*(N+1)+2:5:14*(N+1)+5*N,1) = 45; %servo angle speed in degree/s
        args.lbx(14*(N+1)+3:5:14*(N+1)+5*N,1) = -60; args.ubx(14*(N+1)+3:5:14*(N+1)+5*N,1) = 60; %rpy_MPC in degree
        args.lbx(14*(N+1)+4:5:14*(N+1)+5*N,1) = -60; args.ubx(14*(N+1)+4:5:14*(N+1)+5*N,1) = 60;
        args.lbx(14*(N+1)+5:5:14*(N+1)+5*N,1) = -90; args.ubx(14*(N+1)+5:5:14*(N+1)+5*N,1) = 90;
       




        u0 = repmat(mult.State.Acceleration',N,1);
        X0 = repmat(x0,1,N+1)'; 
        
        args.p = [x0;xs];
        args.x0 = [reshape(X0',6*(N+1),1);reshape(u0',3*N,1)];
%         sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,...
%             'ubg',args.ubg,'p',args.p);
        sol = solver('x0',args.x0,'lbg',args.lbg,...
                'ubg',args.ubg,'p',args.p);
        u = reshape(full(sol.x(6*(N+1)+1:end))',3,N)';
        lin_accel = u(1,:)';
        lin_accel = obj.LimitOutput(lin_accel);

    end
    function force = CalcAeroForce(obj,V)
        a = atand(V(3)/V(1)) + obj.aoi;
        b = 0;
        R_BW = [cosd(a) * cosd(b),    sind(b),     sind(a)*cosd(b);
                    -sind(b) * cosd(a),   cosd(b),     -sind(a)*sind(b);
                    -sind(a)         ,   0  ,        cosd(a)];
        R_WB = R_BW.';
        q_bar = (V' * V) * physics.AirDensity / 2;
        c_y = 0; 
        c_z = 0.35 + 0.11 * a;
        c_d = 0.01 + 0.2 * a * a;
        drag = q_bar * obj.WingSurfaceArea * c_d;
        lateral = q_bar * obj.WingSurfaceArea * c_y;
        lift = q_bar * obj.WingSurfaceArea * c_z;
        force = R_WB * [-drag; lateral;-lift];
    end
    function accel = CalInertialaccel(mult,F_a,T,rpy)
        F_tot = F_a + T;
        
        angles = [rpy(3) rpy(2) rpy(1)];
        
        RNI = zeros(3, 3);
        cang = cos(angles);
        sang = sin(angles);
        
        RNI(1,1) = cang(2).*cang(1);
        RNI(1,2) = cang(2).*sang(1);
        RNI(1,3) = -sang(2);
        RNI(2,1) = sang(3).*sang(2).*cang(1) - cang(3).*sang(1);
        RNI(2,2) = sang(3).*sang(2).*sang(1) + cang(3).*cang(1);
        RNI(2,3) = sang(3).*cang(2);
        RNI(3,1) = cang(3).*sang(2).*cang(1) + sang(3).*sang(1);
        RNI(3,2) = cang(3).*sang(2).*sang(1) - sang(3).*cang(1);
        RNI(3,3) = cang(3).*cang(2);
        rbi = RNI'; % Body to inertial matrix
        F_Inertial = rbi * F_tot;
        accel = F_Inertial / mult.Mass;
    end

    function des_vel = SetVelDes(obj,waypoint_des)


%             vel_des = [26.9130;0;0];
            % Calculate time step

            dt = time - obj.LastTime;
            %disp(vel_des)
            % Calculate the error
            pos_err = waypoint_des - mult.State.Position;


            vel_err = vel_des - mult.State.Velocity;

            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + pos_err * dt;

            % Calculate the PID result
            des_vel =  obj.P * pos_err + ...
                obj.D * vel_err + obj.I * obj.ErrorIntegral;
    end
%% Helper functions
function rpy = rpy_from_z_and_yaw(z_axis, yaw)

    % Initialize the output
    rpy = [0; 0; yaw];

    % Find the desired X axis
    x_c = cross([-sind(yaw); cosd(yaw); 0], z_axis);
    x_axis = x_c / norm(x_c);

    % Find the desired Y axis
    y_axis = cross(z_axis, x_axis);

    % Calculate the roll and pitch
    rpy(1) = 0;
    
    rpy(2) = asind(x_axis(3));
end


