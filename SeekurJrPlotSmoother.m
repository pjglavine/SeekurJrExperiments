%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code simply smoothes the data obtained from different Seekur Jr
% inertial navigation system (INS) tests and plots the results. The various
% JrData0.XNoise.mat files in this folder contain filter data using varied 
% noise corruption magnitudes added to the measured differential Global 
% Positioning System (DGPS) measurements. 
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland
% Email address: pjglavine23@gmail.com
% Date: Aug. 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
load('JrData0.15Noise.mat')
Smooth_X_C = zeros(size(X_estC,1),size(X_estC,2));
Smooth_X_EKF = zeros(size(X_est_EKF,1),size(X_est_EKF,2));
Smooth_MU = zeros(size(MU,1),size(MU,2));
Smooth_GPS_P_estC = zeros(size(GPS_P_estC,1),size(GPS_P_estC,2));
smooth_factor = 30;
smooth_Y1 = zeros(size(Y1,1),size(Y1,2));
smooth_Y_est1 = smooth_Y1;

for i = 1:size(X_estC,1)
    if i == 4 || i == 5 || i == 6
        smooth_factor = 50;
    end
    Smooth_X_C(i,:) = smoothdata(X_estC(i,:),'gaussian',smooth_factor);
    Smooth_X_EKF(i,:) = smoothdata(X_est_EKF(i,:),'gaussian',smooth_factor);
    smooth_factor = 30;
end
for i = 1:size(MU,1)
    Smooth_MU(i,:) = smoothdata(MU(i,:),'gaussian',1);
end

for i = 1:size(GPS_P_estC,1)
    Smooth_GPS_P_estC(i,:) = smoothdata(GPS_P_estC(i,:),'gaussian',30);
end

for i = 1:size(Y1,1)
    smooth_Y1(i,:)= smoothdata(Y1(i,:),'gaussian',50);
    smooth_Y_est1(i,:) = smoothdata(Y_est1(i,:),'gaussian',50);
end

X_estC = Smooth_X_C;
X_est_EKF = Smooth_X_EKF;
MU = Smooth_MU;
GPS_P_estC = Smooth_GPS_P_estC;
X = X_estC;
t = linspace(0,k*dt,k);
crop_num = 1400;

axes_font_size = 14;
title_font_size = 16;
% Result visualization
figure(8)
subplot(3,1,1)
plot(t,(X_estC(4,:))','-r','LineWidth',2,'MarkerSize',2)
ylabel('Velocity [m/s]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
title('Estimated X Velocity vs Time','FontSize',title_font_size,'FontWeight','Bold');
legend({'Estimated X Velocity'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
grid on
subplot(3,1,2)
plot(t,(X_estC(5,:))','-g','LineWidth',2,'MarkerSize',2)
title('Estimated X Velocity vs Time','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Velocity [m/s]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated Y Velocity'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
subplot(3,1,3)
plot(t,(X_estC(6,:))','-b','LineWidth',2,'MarkerSize',2)
title('Estimated Z Velocity vs Time','FontSize',title_font_size,'FontWeight','Bold');
legend({'Estimated Z Velocity'},'FontSize',axes_font_size);
ylabel('Velocity [m/s]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
set(gca,'FontSize',20)


figure(9)
[yaw pitch roll]=quat2angle(X_estC(7:10,:)');
plot(t,yaw,'-r','LineWidth',2,'MarkerSize',2)
hold on
plot(t,pitch,'-g','LineWidth',2,'MarkerSize',2)
hold on
plot(t,roll,'-b','LineWidth',2,'MarkerSize',2)
hold on
%     plot((X_estC(7:10,:))','--+')
grid on
title('Estimated Robot Orientation','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Orientation Angle [rad]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Yaw','Pitch','Roll'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
figure(10)
plot(t,(X_estC(11,:))','-r','LineWidth',2,'MarkerSize',2)
hold on
plot(t,(X_estC(12,:))','-g','LineWidth',2,'MarkerSize',2)
hold on
plot(t,(X_estC(13,:))','-b','LineWidth',2,'MarkerSize',2)
hold on
grid on
title('Estimated Gyro Bias','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Gyro Bias [radians/s]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated X Bias','Estimated Y Bias','Estimated Z Bias'},'FontSize',axes_font_size);
set(gca,'FontSize',20)

figure(11)
plot(t,(X_estC(14,:))','-r','LineWidth',2,'MarkerSize',2)
hold on
plot(t,(X_estC(15,:))','-g','LineWidth',2,'MarkerSize',2)
hold on
plot(t,(X_estC(16,:))','-b','LineWidth',2,'MarkerSize',2)
hold on
grid on
title('Estimated Accelerometer Bias','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Accelerometer Bias [m/s^2]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated X Bias','Estimated Y Bias','Estimated Z Bias'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
%%
figure(12)
Position_Error = abs(([gpsx_true(1:end); gpsy_true(1:end); gpsz_true(1:end)]'-GPS_estC').^2);
Smooth_Position_Error = zeros(size(Position_Error,1),size(Position_Error,2));
for i = 1:size(Position_Error,1)
    Smooth_Position_Error(i,:) = smoothdata(Position_Error(i,:),'gaussian',5);
end

hold off
subplot(3,1,1)
Position_Error = Smooth_Position_Error;
Pos_E1 = repelem(Position_Error(1:end-1,1),4);
Pos_E2 = repelem(Position_Error(1:end-1,2),4);
Pos_E3 = repelem(Position_Error(1:end-1,3),4);
Pos_E1 = Pos_E1(1:end-1,:);
Pos_E2 = Pos_E2(1:end-1,:);
Pos_E3 = Pos_E3(1:end-1,:);
Position_Error = [Pos_E1 Pos_E2 Pos_E3];

GPS_P_estC = GPS_P_estC';
Pos_E1 = repelem(GPS_P_estC(1:end-1,1),4);
Pos_E2 = repelem(GPS_P_estC(1:end-1,2),4);
Pos_E3 = repelem(GPS_P_estC(1:end-1,3),4);
Pos_E1 = Pos_E1(1:end-1,:);
Pos_E2 = Pos_E2(1:end-1,:);
Pos_E3 = Pos_E3(1:end-1,:);
GPS_P_estC = [Pos_E1 Pos_E2 Pos_E3]';

semilogy(t,Position_Error(:,1),'-r','LineWidth',2,'MarkerSize',2)
hold on

semilogy(t,2*GPS_P_estC(1,:)','-k','LineWidth',3,'MarkerSize',3)
hold on
title('X Position Error vs Time','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position Error [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'X Error','95% X Error Confidence Bound'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
subplot(3,1,2)
semilogy(t,Position_Error(:,2),'-b','LineWidth',2,'MarkerSize',2)
hold on
semilogy(t,2*GPS_P_estC(2,:)','-k','LineWidth',3,'MarkerSize',3)
hold on
title('Y Position Error vs Time','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position Error [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Y Error','95% Y Error Confidence Bound'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
subplot(3,1,3)
semilogy(t,Position_Error(:,3),'-g','LineWidth',2,'MarkerSize',2)
hold on
semilogy(t,2*GPS_P_estC(3,:)','-k','LineWidth',3,'MarkerSize',3)
hold on
title('Z Position Error vs Time','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position Error [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Z Error','95% Z Error Confidence Bound'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
% 

figure(16)
plot(t,MU(1,:),'-r','LineWidth',2,'MarkerSize',2)
hold on
plot(t,MU(2,:),'-b','LineWidth',2,'MarkerSize',2)
title('IMM Filter Model Probabilities vs Time','FontSize',axes_font_size,'FontWeight','Bold');
ylabel('Model Probability','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Model 1 Probability','Model 2 Probability'},'FontSize',axes_font_size);
set(gca,'FontSize',20)

gpsx_stretched = [gpsx_stretched gpsx_stretched(1,end) gpsx_stretched(1,end) gpsx_stretched(1,end)];
gpsy_stretched = [gpsy_stretched gpsy_stretched(1,end) gpsy_stretched(1,end) gpsy_stretched(1,end)];
gpsz_stretched = [gpsz_stretched gpsz_stretched(1,end) gpsz_stretched(1,end) gpsz_stretched(1,end)];

figure(17)
subplot(3,1,1)
plot(t,(X_estC(1,:))','-or','LineWidth',2,'MarkerSize',3)
hold on
plot(t,gpsx_stretched,'-k','LineWidth',2,'MarkerSize',1 )
title('Estimated vs Actual X Positions','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated X Position','Actual X Position'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
grid on
subplot(3,1,2)
plot(t,(X_estC(2,:))','-og','LineWidth',2,'MarkerSize',3)
hold on
plot(t,gpsy_stretched,'-k','LineWidth',2,'MarkerSize',1 )
title('Estimated vs Actual X Positions','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated Y Position','Actual Y Position'},'FontSize',axes_font_size);
set(gca,'FontSize',20)
subplot(3,1,3)
plot(t,(X_estC(3,:))','-oc','LineWidth',2,'MarkerSize',3)
hold on
plot(t,gpsz_stretched,'-k','LineWidth',2,'MarkerSize',1 )
title('Estimated vs Actual Z Positions','FontSize',title_font_size,'FontWeight','Bold');
ylabel('Position [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('Time [s]','FontSize',axes_font_size,'FontWeight','Bold');
legend({'Estimated Z Position','Actual Z Position'},'FontSize',axes_font_size);
set(gca,'FontSize',20)

X = X_est_EKF;
t = linspace(0,k*dt,k);

figure(20)
plot(X_est_EKF(1,:),X_est_EKF(2,:),'-g','LineWidth',2,'MarkerSize',2)
title('Estimated vs Actual Trajectory','FontSize',axes_font_size,'FontWeight','Bold');
ylabel('Y Position [m]','FontSize',axes_font_size,'FontWeight','Bold');
xlabel('X Position [m]','FontSize',axes_font_size,'FontWeight','Bold');
hold on
plot(X_estC(1,:),X_estC(2,:),'-r','LineWidth',2,'MarkerSize',2)
hold on
plot(gpsx_true,gpsy_true,'-b','LineWidth',2,'MarkerSize',2)
hold on
%plot(gpsx,gpsy,'-k','LineWidth',0.5,'MarkerSize',1)
legend({'EKF','IMM','DGPS'},'FontSize',axes_font_size);
set(gca,'FontSize',20)


