classdef attitude_controller < pid_controller

    properties
        RateLimits = [70; 70; 30]; % in deg/s
        OutputMax = [8; 8; 8]; % in rad/s^2
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, time)
        % Calculates PID response using this formula:
        % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            import casadi.*
            if isempty(rpy_dot_des)
                rpy_dot_des = zeros(3, 1);
            end
            if isempty(eul_acc_des)
                eul_acc_des = zeros(3, 1);
            end
            
            % Calculate time step
            dt = time - obj.LastTime;
    
            % Calculate the error in radians
            rpy_err = wrapToPi(deg2rad(rpy_des - mult.State.RPY));
          
            % Calculate the rate in radians
            rpy_dot_err = deg2rad(rpy_dot_des - mult.State.EulerRate);
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + rpy_err * dt;
            %Prediction Horizon
            N = 5;
            %Boundary
            r_max = 70;
            p_max = 70;
            y_max = 30;
            rd_max = 8;
            pd_max = 8;
            yd_max = 8;
            %states
            roll = SX.sym('roll');
            pitch = SX.sym('pitch');
            yaw = SX.sym('yaw');
            states = [roll;pitch;yaw];
            n_states = length(states);
            %controls
            roll_acc = SX.sym('roll_acc');
            pitch_acc = SX.sym('pitch_acc');
            yaw_acc = SX.sym('yaw_acc');
            controls = [roll_acc;pitch_acc;yaw_acc];
            n_controls = length(controls);
            %Conversion matrix
            con_mat = [1 sin(roll)*tan(pitch) cos(roll)*tan(pitch);
                       0 cos(roll) -sin(roll);
                       0 sin(roll)/cos(pitch) cos(roll)/cos(pitch)];
            %right hand side
            rhs = con_mat * (controls*dt);
            %Problem setting
            f = Function('f',{states,controls},{rhs});
            U = SX.sym('U',n_controls,N);
            P = SX.sym('P',n_states+n_states);
            X = SX.sym('X',n_states,(N+1));
            obj = 0;
            g = [];
            Q = zeros(3,3);
            Q(1,1) = 1;
            Q(2,2) = 1;
            Q(3,3) = 1;
            R = zeros(3,3);
            R(1,1) = 0.5;
            R(2,2) = 0.5;
            R(3,3) = 0.5;
            %MPC
            st = X(:,1);
            g = [g;st-P(1:n_states)];
            for k = 1:N
                st = X(:,k);
                con = U(:,k);
                obj = obj + (st-P((n_states+1):(n_states+n_states)))'*Q*(st-P((n_states+1):(n_states+n_states))) + con'*R*con;
                st_next = X(:,k+1);
                f_value = f(st,con);
                st_next_euler = st + (f_value*dt);
                g = [g;st_next-st_next_euler];
            end
            OPT_variables = [reshape(X,3*(N+1),1);reshape(U,3*N,1)];
            nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);
            opts = struct;
            opts.ipopt.max_iter = 100;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.acceptable_tol = 1e-8;
            opts.ipopt.acceptable_obj_change_tol = 1e-6;
            solver = nlpsol('solver','ipopt',nlp_prob,opts);
            %Constraints
            args = struct;
            args.lbg(1:3*(N+1)) = -1e-20;
            args.ubg(1:3*(N+1)) = 1e-20;
            
            args.lbx(3*(N+1)+1:3:3*(N+1)+3*N,1) = -rd_max;
            args.ubx(3*(N+1)+1:3:3*(N+1)+3*N,1) = rd_max;
            args.lbx(3*(N+1)+2:3:3*(N+1)+3*N,1) = -pd_max;
            args.ubx(3*(N+1)+2:3:3*(N+1)+3*N,1) = pd_max;
            args.lbx(3*(N+1)+3:3:3*(N+1)+3*N,1) = -yd_max;
            args.ubx(3*(N+1)+3:3:3*(N+1)+3*N,1) = yd_max;
            %simulation loop
            x0 = mult.State.RPY;
            xs = rpy_des;
            
            u0 = mult.State.AngularAcceleration;
            X0 = repmat(x0,1,N+1)';
            
            args.p = [x0;xs];
            args.x0 = [reshape(X0',3*(N+1),1);reshape(u0',3*N,1)];
            sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
            u = reshape(full(sol.x(3*(N+1)+1:end))',3,N)';
            euler_accel = u(1,:);
            
           
            % Update the time of the last call
            obj.LastTime = time;
        end
        
    end
end