%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function converts geodetic to rectangular coordinates.
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland
% Email address: pjglavine23@gmail.com
% Date: Aug. 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x y z] = geodetic2rect(phi, lambda, h)

% WGS84 Parameters
a = 6378137;
recipf = 298.257223563;
e = sqrt((1/recipf)*(2-(1/recipf)));

phi1 = phi*pi/180;
if phi < 0 % 0 south 1 north
    phi1 = 2*pi-phi1;
end

lambda1 = lambda*pi/180;
if lambda < 0 %0 west 1 east
    lambda1 = 2*pi-lambda1;
end

RN = a/((1-e^2*(sin(phi1))^2)^(1/2));

x = (RN+h)*cos(phi1)*cos(lambda1);
y = (RN + h)*cos(phi1)*sin(lambda1);
z = (RN*(1-e^2) + h)*sin(phi1);

end