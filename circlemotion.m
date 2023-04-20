clc
clear
%% INITIALIZATIONS
m = 7.427;
dt = 0.1;                       % time step
t = 0:dt:50 ;                   % time
point=zeros(2,length(t));
point(:,1) = [0;-200] ;   % initial position of point
v = zeros(2,length(t));
v(:,1) = [28.8811,0];         % tangential velocity of point
pC = [0; 0];                   % position of circular path's center
rC = 200;                         % circular path's radius
wrench = zeros(2,length(t));
wrench(:,1) = [0,30.97496];             % initial force

angle=zeros;
angle(1)=0;
a = zeros(2,length(t));
a(:,1) = wrench(:,1) / m;
for i = 2:length(t)

    angle_rate = norm(v(:,i-1)) / rC;
    angle(i) = wrapTo180(angle(i-1) + rad2deg(angle_rate) * dt); 
%     angle(i) = angle(i-1) + rad2deg(angle_rate) * dt; 
    frame = [cosd(angle(i))    0;
        0           sind(angle(i))];

    point(:,i) = point(:,i-1) + 0.5 * a(:,i-1) * dt * dt + v(:,i-1) * dt; %need a negative a in x dir to keep it on circle
%     point(:,i) = pC + [ -rC*cos(angle(i));rC*sin(angle(i))]; % point's position update
    wrench(:,i) = [-sind(angle(i))*norm(wrench(:,1)),cosd(angle(i))*norm(wrench(:,1))];
%     wrench(:,i) = [cosd(angle(i))*norm(wrench(:,1)),-sind(angle(i))*norm(wrench(:,1))];
    a(:,i) = wrench(:,i) / m;
    v(:,i) = v(:,i-1) + a(:,i-1) * dt;
%     disp(point)
end

%% Plotting
grid on,hold on
h = plot(point(2,1),point(1,1),'Ob');
title('Motion animation');xlabel('y (m)');ylabel('x (m)');
axis([-250 250 -250 250])
plotcircle(rC,pC(1),pC(2))    % point's path
for i = 1:length(point)
    set(h,'XData',point(2,i),'YData',point(1,i))
    pause(0.05)
end
%% helper function
function plotcircle(r,x,y)
plot(r*exp(1i*(0:pi/100:2*pi))+x+1i*y,'--');
end
