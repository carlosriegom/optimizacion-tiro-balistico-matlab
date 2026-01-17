% Vwind.m
function V = Vwind(r)
% Find the velocity of the wind in function the height 
% Input: 
% r(1,3) : position of the particle (m)
% Output:
% V(1,3) : velocity of the wind (m/s)

% Set values of wind
vmin = 0;                           % min value (m/s)
vmax = 50*(1e3/3600);               % max value (m/s)
vy = linspace(vmin,vmax,50);
Vx = zeros(1,length(vy));           % x-axis velocity (m/s)       
Vz = Vx;                            % z-axis velocity (m/s)
Vy = vy;                            % y-axis velocity (m/s)
V = [Vx;Vy;Vz];                     % Matrix velocity (m/s)

% Velocity in function of height
h = r(3);                           % Find height of the object
h = round(h);                       % h must be a whole number
% NOTE: becuase the height has been rounded, at height h the velocity is
% the value whose index is nearer to h
if h < 0                            
    V = [0;0;0];                    % There is no wind under ground
elseif h == 0                       
    V = [0;0;0];                    % At h=0 (ground), there is no wind
elseif h > 0 && h < length(vy)
    V = V(:,h);                     % V increases with height
elseif h >= length(vy)
    V = [0;vmax;0];                 % At more than height 50 m, wind is constant (vmax)
end
end

