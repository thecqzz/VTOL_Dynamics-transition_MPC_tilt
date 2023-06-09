classdef control_allocation_vtol < handle
    properties
    end
    
    properties(SetAccess=protected, GetAccess=public)
        Method control_allocation_types % The control allocation method

%%%
        blending_air_speed = 22.194;
        Transition_air_speed = 27.7425;
        Max_speed_sq
        
        actuator_sp_5 = 0;
        actuator_sp_6 = 0;
        actuator_sp_7 = 0;
        actuator_sp_8 = 0;


%%%
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        NDI_L                 % L matrix (related to body-fixed thrust forces)
        NDI_M                 % M matrix (related to body-fixed thrust and reaction moments)
    end

    methods
        function obj = control_allocation_vtol(multirotor)
            obj.SetMethod(multirotor, control_allocation_types.Unified_PseudoInv);
        end
        
        function SetMethod(obj, multirotor, method)
            obj.Method = method;
            if method == control_allocation_types.NDI
                obj.InitializeNDIMethod(multirotor);
            end
        end
        
        function [rotor_speeds_squared, deflections,saturated] = CalcActuators(obj, mult, lin_accel, ang_accel)
        % Calculate the rotor speeds from the desired linear and angular accelerations
            
            persistent lin_accel_last
            persistent ang_accel_last
            persistent rotor_speeds_squared_last
            persistent deflections_last
            persistent saturated_last
            if isempty(lin_accel_last)
                lin_accel_last = zeros(3, 1);
            end
            if isempty(deflections_last)
                deflections_last = zeros(3, 1);
            end
            if isempty(ang_accel_last)
                ang_accel_last = zeros(3, 1);
            end
            if isempty(rotor_speeds_squared_last)
                rotor_speeds_squared_last = zeros(mult.NumOfRotors, 1);
            end
            if isempty(saturated_last)
                saturated_last = false;
            end
            

            % store max speed
            obj.Max_speed_sq = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));
        
            if obj.Method == control_allocation_types.NDI
                [rotor_speeds_squared, deflections] = obj.ActuatorCommands(mult, lin_accel, ang_accel);
            elseif obj.Method == control_allocation_types.Unified_PseudoInv
                [rotor_speeds_squared, deflections] = obj.ActuatorCommands_PseudoInv(mult, lin_accel, ang_accel);
            end

            saturation_flag = false;
            max_rotor_speeds = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared > max_rotor_speeds)
                %mx = max((rotor_speeds_squared - max_rotor_speeds) ./ max_rotor_speeds);
                %rotor_speeds_squared = rotor_speeds_squared - mx * max_rotor_speeds - 1e-5;
                ind = rotor_speeds_squared > max_rotor_speeds;
                rotor_speeds_squared(ind) = max_rotor_speeds(ind);
                saturation_flag = true;
            end
            min_rotor_speeds = cell2mat(cellfun(@(s)s.MinSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared < min_rotor_speeds)
                ind = rotor_speeds_squared < min_rotor_speeds;
                rotor_speeds_squared(ind) = min_rotor_speeds(ind);
                saturation_flag = true;
            end
            
            if nargin > 1
                saturated = saturation_flag;
            end
            lin_accel_last = lin_accel;
            ang_accel_last = ang_accel;
            saturated_last = saturated;
            rotor_speeds_squared_last = rotor_speeds_squared;
        end
    end
    
    %% Private Methods
    methods(Access=protected)
        function InitializeNDIMethod(obj, multirotor)
        % Initialize the NDI method
        
            % Calculate L matrix (related to body thrust forces)
            obj.NDI_L = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               obj.NDI_L(:, i) = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
            end

            % Calculate G matrix (related to body reaction moments)
            NDI_G = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               NDI_G(:, i) = multirotor.Rotors{i}.GetReactionMomentPerUnitInput();
            end
            
            % Calculate F matrix (related to body thrust moments)
            NDI_F = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
                r = multirotor.Rotors{i}.Position;
                F = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
                NDI_F(:, i) = cross(r, F);
            end
            
            obj.NDI_M = NDI_F + NDI_G;
        end

        function [rotor_speeds_squared, deflections] = ActuatorCommands_PseudoInv(obj, multirotor, lin_accel, ang_accel)
            
            % define speed to determine vtol mode
            air_speed_norm = norm(multirotor.State.AirVelocity);

                %y = [lin_accel; ang_accel];

                M_des = multirotor.I*ang_accel;
                F_des = multirotor.TotalMass.*lin_accel ;
 

                % total maxx = 7.4270


%                 F_des = [0,0,-3]';
                
                control_sp = [M_des; F_des];

%                 disp("show control sp in allocation")


                Va_i = multirotor.State.AirVelocity;
                q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;
                
                actuator_trim = zeros(8, 1); 

                actuator_trim(1) = multirotor.State.RotorSpeeds(1)/1.2194e+6;
                actuator_trim(2) = multirotor.State.RotorSpeeds(2)/1.2194e+6;
                actuator_trim(3) = multirotor.State.RotorSpeeds(3)/1.2194e+6;
                actuator_trim(4) = multirotor.State.RotorSpeeds(4)/1.2194e+6;            

                fixed_tilt = (pi / 2) * multirotor.State.ServoAngles(1) / 90;
                
                actuator_trim(5) = obj.actuator_sp_5;
                actuator_trim(6) = obj.actuator_sp_6;
                actuator_trim(7) = obj.actuator_sp_7;
                actuator_trim(8) = obj.actuator_sp_8;
                
                effectiveness_matrix = calc_eff_mat(q_bar, fixed_tilt);
                control_trim = effectiveness_matrix * actuator_trim;
                actuator_sp = actuator_trim + pinv(effectiveness_matrix) * (control_sp - control_trim);
                               
                rotor_speeds_squared = actuator_sp(1:4);
                
% %                 tilt(1) = rad2deg(fixed_tilt);
% %                 tilt(2) = rad2deg(fixed_tilt);

%                 deflections = [abs(actuator_sp(9)+actuator_sp(10))/2  actuator_sp(11) actuator_sp(12)];

               
                deflections = [actuator_sp(5)-actuator_sp(6), actuator_sp(7), actuator_sp(8)];

                obj.actuator_sp_5 = actuator_sp(5);
                obj.actuator_sp_6 = actuator_sp(6);
                obj.actuator_sp_7 = actuator_sp(7);
                obj.actuator_sp_8 = actuator_sp(8);

                
 
        end
    end
end

%% Other functions
function effectiveness_matrix = calc_eff_mat(q_bar, fixed_tilt)
    
    
    %           1         2        3         4
    

%     Px = [ 0.1515    0.1515  -0.1515   -0.1515];
%     Py = [ 0.245    -0.245   -0.245     0.245];

%     Px = [ 0.4 * cosd(30) 0.4 * cosd(150) 0.4 * cosd(210) 0.4 * cosd(330)];
%     Py = [ 0.4 * sind(30) 0.4 * sind(150) 0.4 * sind(210) 0.4 * sind(330)];
    
    Px = [ 1 * cosd(30) 1 * cosd(150) 1 * cosd(210) 1 * cosd(330)];
    Py = [ 1 * sind(30) 1 * sind(150) 1 * sind(210) 1 * sind(330)];

    Pz = zeros(1, 4);

    Ct = (1.08105e-4 / 5)          * 1.2194e+6;    
    Km = ((1.08105e-4 / 5) * 0.05) * 1.2194e+6;
   

    ro = 1.225;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%      S_A = 0.44;
%      S_E = 0.44;
%      S_R = 0.44;
%      b = 2.0 ;

%      c_bar = 0.2; %
%      Cla = 0.1;
%       Cme = 0.5;
%      Cnr = 0.5;
    

     b = 1.0; %wing
% 
     S_A = 0.0720; %aileron surface area in m^2
     S_E = 0.03; %elevator surface area in m^2
     S_R = 0.008; %rudder surface area in m^2
% 
     Cla = 0.11730; %Aileron constant
     Cme = 0.55604; %Elevator constant
     Cnr = 0.08810; %Rudder constant

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%     effectiveness_matrix = [-Py(1) * Ct*cos(trim(5)) - Ct * Km * sin(trim(5)),				-Py(2) * Ct*cos(trim(5)) - Ct * Km * sin(trim(5)),			 -Py(3) * Ct*cos(trim(5)) + Ct * Km * sin(trim(5)),			    -Py(4) * Ct*cos(trim(5)) + Ct * Km * sin(trim(5)),			 - Ct * Km * trim(1) *cos(trim(5)),    	    2 * q_bar*S_A*b*Cla,	    	    0.0, 		 		    0.0;
%                              Ct*(Px(1) * cos(trim(5)) + Pz(1) * sin(trim(5))),  			 Ct*(Px(2) * cos(trim(5)) + Pz(2) * sin(trim(5))),			  Ct*(Px(3) * cos(trim(5)) + Pz(3) * sin(trim(5))),			 	 Ct*(Px(4) * cos(trim(5)) + Pz(4) * sin(trim(5))),			 0.0, 	     		    0.0, 		 	     		 	    q_bar*S_E*b*Cme,	 	0.0; 			
%                             -Py(1) * Ct*sin(trim(5)) + Ct * Km * cos(trim(5)),				-Py(2) * Ct*sin(trim(5)) + Ct * Km * cos(trim(5)),			 -Py(3) * Ct*sin(trim(5)) - Ct * Km * cos(trim(5)),			    -Py(4) * Ct*sin(trim(5)) - Ct * Km * cos(trim(5)),			 - Ct * Km * trim(1) *sin(trim(5)) - Ct * Km * trim(1) *sin(trim(5)) - Ct * Km * trim(1) *sin(trim(5)) - Ct * Km * trim(1) *sin(trim(5)),   	        0.0, 		 	     		 	    0.0, 		 		 	q_bar*S_R*b*Cnr; 	
%                              Ct * sin(trim(5)),	 										     Ct * sin(trim(5)),			   							      Ct * sin(trim(5)),											 Ct * sin(trim(5)),										     Ct * trim(1) *cos(trim(5)) + Ct * trim(2) *cos(trim(5)) + Ct * trim(3) *cos(trim(5)) + Ct * trim(4) *cos(trim(5)), 									     		  	0.0, 		 	     		 	    0.0, 		 		 	0.0; 			
%                              0.0,  			 												 0.0,  						   			 				  0.0,  													 	     0.0,	 												     0.0,		 		  					     					     			0.0, 		 	     		 	    0.0, 		 		 	0.0; 			
%                             -Ct * cos(trim(5)),	 						    			    -Ct * cos(trim(5)),			   							     -Ct * cos(trim(5)),											-Ct * cos(trim(5)),										     Ct * trim(1) *sin(trim(5)) + Ct * trim(2) *sin(trim(5)) + Ct * trim(3) *sin(trim(5)) + Ct * trim(4) *sin(trim(5)), 									     		    0.0,            		 	     	0.0, 		 		 	0.0]; 			
%     

    effectiveness_matrix = [-Py(1) * Ct*cos(fixed_tilt) - Ct * Km * sin(fixed_tilt),		     -Py(2) * Ct*cos(fixed_tilt) - Ct * Km * sin(fixed_tilt),		  -Py(3) * Ct*cos(fixed_tilt) + Ct * Km * sin(fixed_tilt),			    -Py(4) * Ct*cos(fixed_tilt) + Ct * Km * sin(fixed_tilt),			   		  q_bar*S_A*b*Cla,	        -q_bar*S_A*b*Cla,        0.0, 		 		    0.0;
                             Ct*(Px(1) * cos(fixed_tilt) + Pz(1) * sin(fixed_tilt)),  			 Ct*(Px(2) * cos(fixed_tilt) + Pz(2) * sin(fixed_tilt)),		  Ct*(Px(3) * cos(fixed_tilt) + Pz(3) * sin(fixed_tilt)),			 	 Ct*(Px(4) * cos(fixed_tilt) + Pz(4) * sin(fixed_tilt)),			   		  0.0, 		 	            0.0,                    q_bar*S_E*b*Cme,	 	0.0; 			
                            -Py(1) * Ct*sin(fixed_tilt) + Ct * Km * cos(fixed_tilt),		     -Py(2) * Ct*sin(fixed_tilt) + Ct * Km * cos(fixed_tilt),		  -Py(3) * Ct*sin(fixed_tilt) - Ct * Km * cos(fixed_tilt),			    -Py(4) * Ct*sin(fixed_tilt) - Ct * Km * cos(fixed_tilt),			   	      0.0, 		 	            0.0,                    0.0, 		 		 	q_bar*S_R*b*Cnr; 	
                             Ct * sin(fixed_tilt),	 										     Ct * sin(fixed_tilt),			   							      Ct * sin(fixed_tilt),											         Ct * sin(fixed_tilt),										       		      0.0, 		 	            0.0,                    0.0, 		 	        0.0; 			
                             0.0,  			 												     0.0,  							   			 				      0.0,  													 	         0.0,	 												  			          0.0, 		 	            0.0,                    0.0, 		 	        0.0; 			
                            -Ct * cos(fixed_tilt),	 						    			     -Ct * cos(fixed_tilt),			   							      -Ct * cos(fixed_tilt),											     -Ct * cos(fixed_tilt),									        		      0.0, 		 	            0.0,                    0.0, 		 	        0.0]; 			
   
%     effectiveness_matrix = [-Py(1) * Ct*cos(trim(5)) - Ct * Km * sin(trim(5)),				-Py(2) * Ct*cos(trim(6)) - Ct * Km * sin(trim(6)),			 -Py(3) * Ct*cos(trim(7)) + Ct * Km * sin(trim(7)),			    -Py(4) * Ct*cos(trim(8)) + Ct * Km * sin(trim(8)),			 Py(1) * Ct*trim(1) *sin(trim(5)) - Ct * Km * trim(1) *cos(trim(5)),    	     Py(2) * Ct*trim(2) *sin(trim(6)) - Ct * Km * trim(2) *cos(trim(6)),		 		Py(3) * Ct*trim(3) *sin(trim(7)) + Ct * Km * trim(3) *cos(trim(7)),		            Py(4) * Ct*trim(4) *sin(trim(8)) + Ct * Km * trim(4) *cos(trim(8)),  		-q_bar*S*b*Cla,	    q_bar*S*b*Cla,	    0.0, 		 		    0.0;
%                              Ct*(Px(1) * cos(trim(5)) + Pz(1) * sin(trim(5))),  			 Ct*(Px(2) * cos(trim(6)) + Pz(2) * sin(trim(6))),			  Ct*(Px(3) * cos(trim(7)) + Pz(3) * sin(trim(7))),			 	 Ct*(Px(4) * cos(trim(8)) + Pz(4) * sin(trim(8))),			 Ct*trim(1) *(-Px(1) * sin(trim(5)) + Pz(1) * cos(trim(5))), 	     		     Ct*trim(2) *(-Px(2) *sin(trim(6)) + Pz(2) * cos(trim(6))),							Ct*trim(3) *(-Px(3) *sin(trim(7)) + Pz(3) * cos(trim(7))),							Ct*trim(4) *(-Px(4) *sin(trim(8)) + Pz(4) *cos(trim(8))),  					 0.0, 		 	    0.0, 		 	    q_bar*S*c_bar*Cme,	 	0.0; 			
%                             -Py(1) * Ct*sin(trim(5)) + Ct * Km * cos(trim(5)),				-Py(2) * Ct*sin(trim(6)) + Ct * Km * cos(trim(6)),			 -Py(3) * Ct*sin(trim(7)) - Ct * Km * cos(trim(7)),			    -Py(4) * Ct*sin(trim(8)) - Ct * Km * cos(trim(8)),			-Py(1) * Ct*trim(1) *cos(trim(5)) - Ct * Km * trim(1) *sin(trim(5)),   	        -Py(2) * Ct*trim(2) *cos(trim(6)) - Ct * Km * trim(2) *sin(trim(6)),		  	   -Py(3) * Ct*trim(3) *cos(trim(7)) + Ct * Km * trim(3) *sin(trim(7)), 	 		   -Py(4) * Ct*trim(4) *cos(trim(8)) + Ct * Km * trim(4) *sin(trim(8)),  		 0.0, 		 	    0.0, 		 	    0.0, 		 		 	q_bar*S*b*Cnr; 	
%                              Ct * sin(trim(5)),	 										     Ct * sin(trim(6)),			   							      Ct * sin(trim(7)),											 Ct * sin(trim(8)),										     Ct * trim(1) *cos(trim(5)), 									     		     Ct * trim(2) *cos(trim(6)),											 			Ct * trim(3) *cos(trim(7)), 											 			Ct * trim(4) *cos(trim(8)),   												 0.0, 		 	    0.0, 		 	    0.0, 		 		 	0.0; 			
%                              0.0,  			 												 0.0,  							   			 				  0.0,  													 	 0.0,	 													 0.0,		 		  					     					     			 0.0, 			      							 									0.0, 			    	  											   	 		   	0.0, 			           								 					 0.0, 		 	    0.0, 		 	    0.0, 		 		 	0.0; 			
%                             -Ct * cos(trim(5)),	 						    			    -Ct * cos(trim(6)),			   							     -Ct * cos(trim(7)),											-Ct * cos(trim(8)),										     Ct * trim(1) *sin(trim(5)), 									     		     Ct * trim(2) *sin(trim(6)),											 			Ct * trim(3) *sin(trim(7)), 											 			Ct * trim(4) *sin(trim(8)),   								 				 0.0, 		 	    0.0, 		 	    0.0, 		 		 	0.0]; 			
%     
end