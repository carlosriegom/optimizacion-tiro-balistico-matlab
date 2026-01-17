function [r,v,a] = rungeKutta( force, mass, r0, v0, dt, tmax )

% Solves Newton's equation, using fourth-order Runge-Kutta method,
% for np particles in nd dimensions. J.M.Soler, Jan.2011
% 
% Input:
%   force     : function to calculate the force as f=force(x,v,t), in N
%   mass(np)  : particle masses of the np particles in kg
%   r0(nd,np) : intial positions of np particles in nd dimensions, in m
%   v0(nd,np) : initial velocities in m/s
%   dt        : integration time interval in s
%   tmax      : final integration time in s
% Output:
%   r(nd,np,nt) : position vectors at each time (0:dt:tmax), in m
%   v(nd,np,nt) : velocities at each time, in m/s
%   a(nd,np,nt) : accelerations at each time, in m/s^2
%
% Algorithms:
% First we rewrite the second-order Newton equation as a first order
% differential equation, by doubling the number of variables:
%   y(1,t) = x(t),
%   y(2,t) = v(t).
% Then
%   dy(1,t)/dt = dx/dt = v = y(2,t)
%   dy(2,t)/dt = dv/dt = a = f(x,v,t)/m = f(y,t)/m.
% The fourth-order Runge-Kutta iteration is
%   z1 = y'(y1),           where y1=y(t)  and  y'=dy/dt,
%   z2 = y'(y1+z1*dt/2),
%   z3 = y'(y1+z2*dt/2),
%   z4 = y'(y1+z3*dt).
%   y(t+dt) = y(t) + z1*dt/6 + z2*dt/3 + z3*dt/3 + z4*dt/6
% Ref: W.H.Press et al, Numerical Recipes, Cambridge U.P.


% Check array sizes: nd=num. spacial dimensions, np=num. particles
[nd,np] = size(r0);
if (length(mass)~=np)
    'rungeKutta: ERROR: length(mass)~=np'
end
if (size(v0)~=size(r0))
     'rungeKutta: ERROR: size(v0)~=size(x0)'
end

% Set auxiliary variables and arrays
t = (0:dt:tmax);                            % integration times
nt = numel(t);                              % number of times
y1 = zeros(nd,np,2); y2=y1; y3=y1; y4=y1;   % positions and velocities
z1 = zeros(nd,np,2); z2=z1; z3=z1; z4=z1;   % velocities and accelerations

% Assign a mass to each coordinate
m = zeros(nd,np);   
for ip = 1:np
    m(:,ip) = mass(ip);
end

% Integrate trajectory
r = zeros(nd,np,nt);                        % assign size of array r
v=r; a=r;                                   % assign size to v and a
r(:,:,1) = r0;                              % positions at t=0
v(:,:,1) = v0;                              % velocities at t=0
y1(:,:,1) = r0;                             % current positions
y1(:,:,2) = v0;                             % current velocities
for it = 1:nt-1                             % time step loop
    t = (it-1)*dt;                          % initial time t of step
    z1(:,:,1) = y1(:,:,2);                  % dy(:,:,1)/dt at t
    z1(:,:,2) = force(y1(:,:,1), ...
                      y1(:,:,2),t) ./ m;    % dy(:,:,2)/dt at t
    t = (it-0.5)*dt;                        % mid time of step: t+dt/2
    y2 = y1 + z1*dt/2;                      % y at t+dt/2
    z2(:,:,1) = y2(:,:,2);                  % dy(:,:,1)/dt at t+dt/2
    z2(:,:,2) = force(y2(:,:,1), ...
                      y2(:,:,2),t) ./ m;    % dy(:,:,2)/dt at t+dt/2
    y3 = y1 + z2*dt/2;                      % new estimate of y at t+dt/2
    z3(:,:,1) = y3(:,:,2);                  % new dy(:,:,1)/dt at t+dt/2
    z3(:,:,2) = force(y3(:,:,1), ...
                      y3(:,:,2),t) ./ m;    % new dy(:,:,2)/dt at t+dt/2
    t = it*dt;                              % final time of step: t+dt
    y4 = y1 + z3*dt;                        % y at t+dt
    z4(:,:,1) = y4(:,:,2);                  % dy(:,:,1)/dt at t+dt
    z4(:,:,2) = force(y4(:,:,1), ...
                      y4(:,:,2),t) ./ m;    % dy(:,:,2)/dt at t+dt
    y1 = y1 + z1*dt/6 + z2*dt/3 ...
            + z3*dt/3 + z4*dt/6;            % new y(t+dt)
    r(:,:,it+1) = y1(:,:,1);                % positions at t+dt
    v(:,:,it+1) = y1(:,:,2);                % velocities at t+dt
    a(:,:,it)   = z1(:,:,2);                % accelerations at t
end
t = nt*dt;
a(:,:,nt) = force(r(:,:,nt),v(:,:,nt),t)./m; % acceleration at final point

end % function rungeKutta
