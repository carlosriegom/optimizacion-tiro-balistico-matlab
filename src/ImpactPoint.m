% impactPoint.m
function [y,z,t01] = ImpactPoint(alpha,beta,m,Cd,A,x0,Vtotal,tmax,dt)
% Find the impact point in the y and z axis (where x = 0).
% Input:
%   m : mass of the object (kg)
%   A : area of the object (m^2)
%   Cd : drag coefficient
%   x0 : initial position in x-axis (m)
%   Vtotal : module of the initial velocity of the object (m/s)
%   alpha : angle formed by the direction of the shot with the xy plane
%   (rad)
%   beta : angle formed by the projection of the shot direction with the
%   x-axis (rad)
%   tmax : total time of the trajectory (s)
%   dt : interval of time (s)
% Output:
%   y : impact point in the y axis (m)
%   z : impact point in the z axis (m)

% Set initial params
r0 = [x0;0;0];                                                                      % Initial position (m)
v0 = [Vtotal*cos(alpha)*cos(beta); Vtotal*cos(alpha)*sin(beta); Vtotal*sin(alpha)]; % Initial velocity (m/s)

% Integrate trajectory of the object
myforce = @(r,v,t) force(m,A,Cd,r,v,Vwind(r));
[r] = rungeKutta(myforce,m,r0,v0,dt,tmax);
r = squeeze(r);                  % Get all the columns together in a matrix

% Set vector of times
nt = size(r,2);                  % Find number of columns of r
t = linspace(0,tmax,nt);         % Set vector of times 't', with the same length as r
it = find(r(1,:)<0,1,'last');    % Find the index of r in wich there is the last negative value of x before zero

% Set it for when it = nt (it+1 doesn't exist)
if it == nt
    it = it -1;
end

% Lever rule
x1 = r(1,it);                                % Last x before 0 (m)
x2 = r(1,it+1);                              % First x after 0 (m)
t1 = t(it);                                  % Time for x = x1 (s)
t2 = t(it+1);                                % Time for x = x2 (s)
t01 = (t1*x2-t2*x1)/(x2-x1);                 % Time for x = 0 (s)
v01 = (r(:,it+1)-r(:,it))/(t(it+1)-t(it));   % Velocity at x = 0 (m/s)
r01 = r(:,it)+v01*(t01-t(it));               % Position vector at time t01 (m)
y = r01(2);                                  % Impact point in the y axis (when t = t01) (m)
z = r01(3);                                  % Impact point in the z axis (when t = t01) (m)

end 


