% bisectionRoot.m
function [alpha0,beta0] = bisectionRoot(alpha01,alpha02,beta01,beta02,dangleMax,m,Cd,A,x0,Vtotal,tmax,dt)
% Find a root alpha0 and beta0 of function func, bracketed between alpha1
% and alpha2 & beta1 and beta2, respectively.
% In order to find the optimized angles (alpha0 and beta0) at which the
% object impacts in the required point (0,0,0), set an initial beta
% angle and find the alpha0 that satisfies z(alpha)=0. Then, set the
% last alpha0 as constant and find the beta0 that satisfies y(beta)=0.
% Repeat this process until the difference between alpha0 and its previous
% value is less than 1e-6. Do the same with beta0.
% Input:
%   m : mass of the object (kg)
%   A : area of the object (m^2)
%   Cd : drag coefficient
%   x0 : initial position in x-axis (m)
%   Vtotal : module of the initial velocity of the object (m/s)
%   tmax : total time of the trajectory (s)
%   dt : interval of time (s)
%   alpha1, alpha2 : lower and upper bounds of root (rad)
%   beta1, beta2 : lower and upper bounds of root (rad)
%   dangleMax  : max. error in value of root alpha0 and beta0 (rad)
% Output:
%   alpha0 : root of z(alpha), i.e. z(alpha0)=0
%   beta0 : root of y(beta), i.e. y(beta0)=0

% Set lastalpha and lastbeta for first iteration
lastalpha = inf;
lastbeta = inf;
error = 1e-6;                       % Set error

while true
if ~exist('beta0')   % Set beta angle for first iteration
    beta = pi/180;   % default value
else
    beta = beta0;    % Set beta0 as constant to find alpha0
end

% Set alpha1 and alpha2 as constant for every iteration
alpha1 = alpha01; 
alpha2 = alpha02;

% Check bounds
[~,z1,~] = ImpactPoint(alpha1,beta,m,Cd,A,x0,Vtotal,tmax,dt);
[~,z2,~] = ImpactPoint(alpha2,beta,m,Cd,A,x0,Vtotal,tmax,dt);

% Bisection iteration
while (abs(alpha2-alpha1)>dangleMax)
    alpha0 = (alpha1+alpha2)/2;
    [~,z0,~] = ImpactPoint(alpha0, beta,m,Cd,A,x0,Vtotal,tmax,dt);
    if (z0*z1>0) 
        alpha1 = alpha0;
        z1 = z0;
    else
        alpha2 = alpha0;
        z2 = z0;
    end
end % while
alpha0 = (alpha1*z2-alpha2*z1) / (z2-z1);   % Find the root
alpha = alpha0;                             % Set alpha0 as constant to find beta0

% Set beta1 and beta2 as constant for every iteration
beta1 = beta01;         
beta2 = beta02;

% Check bounds
[y1,~,~] = ImpactPoint(alpha, beta1,m,Cd,A,x0,Vtotal,tmax,dt);
[y2,~,~] = ImpactPoint(alpha, beta2,m,Cd,A,x0,Vtotal,tmax,dt);

% Bisection iteration
while (abs(beta2-beta1)>dangleMax)
    beta0 = (beta1+beta2)/2;
    [y0,~,~] = ImpactPoint(alpha, beta0,m,Cd,A,x0,Vtotal,tmax,dt);
    if (y0*y1>0) 
        beta1 = beta0;
        y1 = y0;
    else
        beta2 = beta0;
        y2 = y0;
    end
end % while
beta0 = (beta1*y2-beta2*y1) / (y2-y1);     % Find the root

% Stop iteration when the difference between alpha0 and its previous
% value is less than the error (same with beta0)
if abs(lastalpha-alpha0)<error && abs(lastbeta-beta0)<error
    break 
end

% Set new lastalpha and lastbeta
lastalpha = alpha0;
lastbeta = beta0;
end % while true
end % function
