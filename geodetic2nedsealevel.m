%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function converts standard geodetic coordinates to the
% North-East-Down reference frame at sea level. 
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland
% Email address: pjglavine23@gmail.com
% Date: Feb. 2017
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x y z] = geodetic2nedsealevel(lambda,phi,h,lambda0,phi0,alt)

% Get degree angle values for Tangent Plane
phi01 = phi0;
if phi0 < 0 % phi is south
    phi01 = 360-phi01;
end
lambda01 = lambda0;
if lambda0 < 0 % lambda is west
    lambda01 = 360-lambda01;
end

% Obtain ECEF xyz coordinates of point P and Tangent Plane origin
[xe ye ze] = geodetic2rect(phi,lambda,h);
[xo yo zo] = geodetic2rect(phi0,lambda0,alt);

% Calculate vector between tangent plane origin and point P in ECEF
xhat = [xe ye ze] - [xo yo zo];

%Convert tangent plane angle to radians for transformation
po = phi01*pi/180;
lo = lambda01*pi/180;

% Rotation matrix to transform from ECEF to Tangent Plane
Rte = [-sin(po)*cos(lo) -sin(po)*sin(lo) cos(po);
       -sin(lo) cos(lo) 0;
       -cos(po)*cos(lo) -cos(po)*sin(lo) -sin(po)];

% Point in Tangent Plane
Pt = Rte*xhat';
x = Pt(1);
y = Pt(2);
z = Pt(3);
end

