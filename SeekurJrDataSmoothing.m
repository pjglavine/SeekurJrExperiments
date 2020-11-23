%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is used to smooth highly noisy IMU measurements from the Seekur
% Jr inertial navigation system (INS) experiments. This was mainly done to
% reduce the impulse response of the INS to a sudden drop as the robot
% traveled over the edge of a curb.
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland
% Email address: pjglavine23@gmail.com
% Date: Aug. 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
load('SeekurDataOct4Reduced.mat')

% Store the IMU measurements in vectors for each directional component.
Accelerometer_x = Accelerometer(:,1);
Accelerometer_y = Accelerometer(:,2);
Accelerometer_z = Accelerometer(:,3);

Gyroscope_x = Gyroscope(:,1);
Gyroscope_y = Gyroscope(:,2);
Gyroscope_z = Gyroscope(:,3);

Magnetometer_x = Magnetometer(:,1);
Magnetometer_y = Magnetometer(:,2);
Magnetometer_z = Magnetometer(:,3);

% Apply pchip smoothing to the IMU data.

Smooth_Accelerometer_x = filloutliers(Accelerometer_x,'pchip','gesd');
Smooth_Accelerometer_y = filloutliers(Accelerometer_y,'pchip','gesd');
Smooth_Accelerometer_z = filloutliers(Accelerometer_z,'pchip','gesd');

% Isolate the data corresponding to the robot driving over the bump in the
% curb and perform additional smoothing to reduce the shock the INS
% experiences.
Bump_Accelerometer_x = Smooth_Accelerometer_x(1409:1459,1);
Bump_Accelerometer_y = Smooth_Accelerometer_y(1409:1459,1);
Bump_Accelerometer_z = Smooth_Accelerometer_z(1409:1459,1);

Smooth_Bump_Accelerometer_x = smoothdata(Bump_Accelerometer_x,'SmoothingFactor',0.97);
Smooth_Bump_Accelerometer_y = smoothdata(Bump_Accelerometer_y,'SmoothingFactor',0.97);
Smooth_Bump_Accelerometer_z = smoothdata(Bump_Accelerometer_z,'SmoothingFactor',0.97);

Smooth_Accelerometer_x(1409:1459,1)= Smooth_Bump_Accelerometer_x;
Smooth_Accelerometer_x(1409:1459,1)= Smooth_Bump_Accelerometer_x;
Smooth_Accelerometer_x(1409:1459,1)= Smooth_Bump_Accelerometer_x;

Smooth_Gyroscope_x = filloutliers(Gyroscope_x,'pchip','gesd');
Smooth_Gyroscope_y = filloutliers(Gyroscope_y,'pchip','gesd');
Smooth_Gyroscope_z = filloutliers(Gyroscope_z,'pchip','gesd');

Smooth_Magnetometer_x = filloutliers(Magnetometer_x,'pchip','gesd');
Smooth_Magnetometer_y = filloutliers(Magnetometer_y,'pchip','gesd');
Smooth_Magnetometer_z = filloutliers(Magnetometer_z,'pchip','gesd');

Bump_Gyroscope_x = Smooth_Gyroscope_x(1409:1459,1);
Bump_Gyroscope_y = Smooth_Gyroscope_y(1409:1459,1);
Bump_Gyroscope_z = Smooth_Gyroscope_z(1409:1459,1);

Smooth_Bump_Gyroscope_x = smoothdata(Bump_Gyroscope_x,'SmoothingFactor',0.97);
Smooth_Bump_Gyroscope_y = smoothdata(Bump_Gyroscope_y,'SmoothingFactor',0.97);
Smooth_Bump_Gyroscope_z = smoothdata(Bump_Gyroscope_z,'SmoothingFactor',0.97);

Smooth_Gyroscope_x(1409:1459,1)= Smooth_Bump_Gyroscope_x;
Smooth_Gyroscope_x(1409:1459,1)= Smooth_Bump_Gyroscope_x;
Smooth_Gyroscope_x(1409:1459,1)= Smooth_Bump_Gyroscope_x;

Bump_Magnetometer_x = Smooth_Magnetometer_x(1409:1459,1);
Bump_Magnetometer_y = Smooth_Magnetometer_y(1409:1459,1);
Bump_Magnetometer_z = Smooth_Magnetometer_z(1409:1459,1);

Smooth_Bump_Magnetometer_x = smoothdata(Bump_Magnetometer_x,'SmoothingFactor',0.97);
Smooth_Bump_Magnetometer_y = smoothdata(Bump_Magnetometer_y,'SmoothingFactor',0.97);
Smooth_Bump_Magnetometer_z = smoothdata(Bump_Magnetometer_z,'SmoothingFactor',0.97);

Smooth_Magnetometer_x(1409:1459,1)= Smooth_Bump_Magnetometer_x;
Smooth_Magnetometer_x(1409:1459,1)= Smooth_Bump_Magnetometer_x;
Smooth_Magnetometer_x(1409:1459,1)= Smooth_Bump_Magnetometer_x;

% figure(13)
% plot(Bump_Accelerometer_x)
% hold on
% plot(Smooth_Bump_Accelerometer_x)
% 
% figure(14)
% plot(Bump_Accelerometer_y)
% hold on
% plot(Smooth_Bump_Accelerometer_y)
% 
% figure(15)
% plot(Bump_Accelerometer_z)
% hold on
% plot(Smooth_Bump_Accelerometer_z)

figure(1)
plot(Accelerometer_x)
hold on
plot(Smooth_Accelerometer_x)

figure(2)
plot(Accelerometer_y)
hold on
plot(Smooth_Accelerometer_y)

figure(3)
plot(Accelerometer_z)
hold on
plot(Smooth_Accelerometer_z)

figure(4)
plot(Gyroscope_x)
hold on
plot(Smooth_Gyroscope_x)

figure(5)
plot(Gyroscope_y)
hold on
plot(Smooth_Gyroscope_y)

figure(6)
plot(Gyroscope_z)
hold on
plot(Smooth_Gyroscope_z)

figure(7)
plot(Magnetometer_x)
hold on
plot(Smooth_Magnetometer_x)

figure(8)
plot(Magnetometer_y)
hold on
plot(Smooth_Magnetometer_y)

figure(9)
plot(Magnetometer_z)
hold on
plot(Smooth_Magnetometer_z)

Accelerometer = [Smooth_Accelerometer_x Smooth_Accelerometer_y Smooth_Accelerometer_z];
Gyroscope = [Smooth_Gyroscope_x Smooth_Gyroscope_y Smooth_Gyroscope_z];
Magnetometer= [Smooth_Magnetometer_x Smooth_Magnetometer_y Smooth_Magnetometer_z];

% figure(10)
% plot(Smooth_Gyroscope_x)
% hold on
% plot(Smooth_Gyroscope_x2)
% 
% figure(11)
% plot(Smooth_Gyroscope_y)
% hold on
% plot(Smooth_Gyroscope_y2)
% 
% figure(12)
% plot(Smooth_Gyroscope_z)
% hold on
% plot(Smooth_Gyroscope_z2)