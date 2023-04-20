model = fixedwing;
s = state(model);
s(4) = 5; % 5 m/s

u = control(model);
u.RollAngle = pi/12;
u.AirSpeed = 5;

% Enviornment
e = environment(model);

sdot = derivative(model,s,u,e);
simOut = ode45(@(~,x)derivative(model,x,u,e), [0 50], s);
size(simOut.y)

% trajectory
downsample = 1:30:size(simOut.y,2);
translations = simOut.y(1:3,downsample)'; % xyz-position
rotations = eul2quat([simOut.y(5,downsample)',simOut.y(6,downsample)',simOut.y(7,downsample)']); % ZYX Euler
plotTransforms(translations,rotations,...
    'MeshFilePath','fixedwing.stl','InertialZDirection',"down")
hold on

plot3(simOut.y(1,:),-simOut.y(2,:),simOut.y(3,:),'--b') % full path
xlim([-10.0 10.0])
ylim([-20.0 5.0])
zlim([-0.5 4.00])
view([-45 90])
hold off