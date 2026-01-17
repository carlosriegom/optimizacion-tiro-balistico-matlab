% myforce.m
function f = force(m,A,Cd,r,v,Vwind)
% Find the total force due to wind and gravity applied in an object along
% the trajectory
% Input:
%   m : mass of the object (kg)
%   A : area of the object (m^2)
%   Cd : drag coefficient
%   den : air density (Kg/m^3)
%   r(3,1) : position of the object (m)
%   v(3,1) : velocity of the object (m/s)
%   Vwind(3,1) : velocity of the wind (m/s)
% Output:
%   f(3,1) : total force on the object (N)

% Set Planet's surface gravity
% Earth
R = 6371e3;            % Earth's radius (m)
M = 5.972e24;          % Earth's mass (Kg)  
G = 6.672e-11;         % Universal gravitation constant (m^2/kg*s^-2)
den = 1.23;            % Air density at T = 25ºC (Kg/m^3)

% Mars
% R = 3389e3;          % Earth's radius (m)
% M = 6.39e23;         % Earth's mass (Kg)  
% G = 6.672e-11;       % Universal gravitation constant (m^2/kg*s^-2)
% den = 1.30;          % Air density at T = 0ºC (Kg/m^3)

% Set force
f = -A*0.5*den*Cd*(norm(v-Vwind))^2*(v-Vwind)/norm(v-Vwind);

% Add gravity force on the z-axis
g = -G*M/R^2;                        % Planet's surface gravity (m/s^2)
f_g = m*g;                          % Gravity force (N)
f(3,:) = f(3,:) + f_g;              % Final z-axis force

end 