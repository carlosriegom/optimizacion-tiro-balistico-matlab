% impact.m
% NOTE: For running one throw, comment the other data
%% DATA - BALLISTIC SHOT %%
clear all
m = 269e-1;         % Mass of the bullet (kg)
Vtotal = 300;       % Initial velocity of the bullet (m/s)
A = 1e-4;           % Section area of the bullet (m^2)
Cd = 0.1;           % Aerodinamic coefficient
tmax = 3.5;         % Maximum time of integration of the trajectory (s)
dt = 0.1;           % Time interval's length (s) 
x0 = -1000;         % Initial position in the x-axis (m)

% Initial angles for bisectionRoot (rad)
alpha01 = 0;        % Lower bound of root (z(alpha))
alpha02 = pi/2;     % Upper bound of root (z(alpha))
beta01 = -pi/2;     % Lowerr bound of root (y(beta))
beta02 = +pi/2;     % Upper bound of root (y(beta))
dangleMax = 1e-6;   % Maximum error in value of root alpha0 and beta0 (rad)

%% DATA - ARCHERY %%
% clear all
% m = 20e-3;        % Mass of the arrow (kg)
% Vtotal = 80;      % Initial velocity of the arrow (m/s)
% A = 40e-4;        % Section area of the arrow (m^2
% Cd = 0.1;         % Aerodinamic coefficient
% tmax = 4;         % Maximum time of integration of the trajectory (s)
% dt = 0.1;         % Time interval's length (s)
% x0 = -100;        % Initial position in the x-axis (m)
% 
% % Initial angles for bisectionRoot (rad)
% alpha01 = 0;        % Lower bound of root (z(alpha))
% alpha02 = pi/2;     % Upper bound of root (z(alpha))
% beta01 = -pi/2;     % Lowerr bound of root (y(beta))
% beta02 = +pi/2;     % Upper bound of root (y(beta))
% dangleMax = 1e-6;   % Maximum error in value of root alpha0 and beta0 (rad)

%% FIND alpha0 and beta0 %%
[alpha0,beta0] = bisectionRoot(alpha01,alpha02,beta01,beta02,dangleMax,m,Cd,A,x0,Vtotal,tmax,dt);
[y,z,t01] = ImpactPoint(alpha0,beta0,m,Cd,A,x0,Vtotal,tmax,dt);
% Print results
fprintf('Impact point in y-axis: %e m\n',y);
fprintf('Impact point in z-axis: %e m\n',z);
fprintf('Total time: %2.3f s\n',t01);
fprintf('Angle alpha0 formed by the direction of the shot with the xy plane is: %2.6f°\n',alpha0*(180/pi));
fprintf('Angle beta0 formed by the projection of the shot direction with the x-axis: = %2.6f°\n',beta0*(180/pi));

%% FIND OPTIMIZED TRAJECTORY %%
% Set initial params
r0 = [x0;0;0];                                                                            % Initial position (m)
v0 = [Vtotal*cos(alpha0)*cos(beta0); Vtotal*cos(alpha0)*sin(beta0); Vtotal*sin(alpha0)];  % Initial velocity (m/s)

% Integrate trajectory
myforce = @(r,v,t) force(m,A,Cd,r,v,Vwind(r));
[r] = rungeKutta(myforce,m,r0,v0,dt,tmax);

%% PLOT THE TRAJECTORY %%
% Plot as a video
figure(1)
nt = size(r,3);
t = linspace(0,tmax,nt);
for it = 1:nt
    plot3(r(1,:,it),r(2,:,it),r(3,:,it),'bo'); hold on
    % Plot target
    scatter3(0,0,0,500,'filled','k'); hold on         
    scatter3(0,0,0,350,'filled','b'); hold on
    scatter3(0,0,0,250,'filled','r'); hold on
    scatter3(0,0,0,100,'filled','y'); hold on
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    grid on 
    xlim([x0,0])
    title(['time = ',num2str(t(it)),' s'])
    pause(dt)
end

% Find optimise time for ploting
it = find(t>t01,1,'first');

% Plot total trajectory
r = squeeze(r);             % Get all the columns together in a matrix
plot3(r(1,1:it),r(2,1:it),r(3,1:it),'k-');
hold on
plot3(0,0,0,'*');
grid on
xlim([x0,0])