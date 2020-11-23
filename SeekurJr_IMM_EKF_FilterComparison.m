%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inertial navigation system (INS) for Seekur Jr robot that integrates
% system equations using an inertial measurement unit (IMU) sensor and
% applies corrections using Global Positioning System (GPS) and
% Magnetometer feedback. This script runs both an Extended Kalman filter
% (EKF) and Interactive Multiple Model (IMM) filter to compare the
% localization performance of both. The filters use identical datasets that
% have been corrupted with added Gaussian noise. The performance is
% measured against the original dataset containing no added noise. The
% added noise is varied in magnitude to evaluate each filter's
% effectiveness in the presence of small or large noise corrupting effects.
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland
% Email address: pjglavine23@gmail.com
% Date: Aug. 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all;

% Load Seekur Jr Sensor Data
load('SeekurDataOct4ReducedSmooth.mat')
THRESHOLD = 0.0796;
tuning = 0; % 0: Noise tuning turned off, 1: IMM Tuning ON 2: EKF Tuning ON

% load('SeekurDataOct26Reduced.mat')
% THRESHOLD = 0.059;
%
% load('SeekurDataOct4Reduced2.mat')
% THRESHOLD = 0.079;

% load('SeekurDataOct4NoLaser.mat')
% THRESHOLD = 0.0524;

simulator = 0; % 0: Not using simulator, 1: Simulator being used
cropping = 0; % 0: No data cropping, 1: Data cropping turned ON
crop_num = 1600; % Data index to end cropping data at
Graphing = 1; % 0: No Graphing, 1: Graphing ON

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Select first GPS data point as the reference point of the Tangent Plane
lambdaO = Latitude(1,1);
phiO = Longitude(1,1);
altO = Altitude(1,1);

% GPS Antenna offset from robot coordinate frame location.
x_off = 0.295;
y_off = -0.11;
z_off = 0.365;

% x_off = 0;
% y_off = 0;
% z_off = 0;

% Convert GPS coordinates into x,y,z coordinates of a local tangent plane
gpsx = zeros(1,length(Latitude));
gpsy = zeros(1,length(Latitude));
gpsz = zeros(1,length(Latitude));

for i = 1:length(Latitude)
    [x,y,z]=geodetic2nedsealevel(Latitude(i,1),Longitude(i,1),Altitude(i,1),lambdaO, phiO, altO);
    gpsx(i) = x-x_off;
    gpsy(i) = y-y_off;
    gpsz(i) = z-z_off;
end

if simulator == 1
    gpsx = Latitude';
    gpsy = Longitude';
    gpsz = Altitude';
end

% Save the GPS ground truth values
gpsx_true = gpsx;
gpsy_true = gpsy;
gpsz_true = gpsz;

% Stretch GPS matrix data to match the dimensions of the IMU data. This is
% purely for graphing purposes since the GPS measures location values much
% slower than the IMU updates.
gpsx_stretched = repelem(gpsx(1,1:end-2),4);
gpsy_stretched = repelem(gpsy(1,1:end-2),4);
gpsz_stretched = repelem(gpsz(1,1:end-2),4);

% Add Gaussian noise to artificially corrupt the measured GPS data.
lower_bound = -1;
upper_bound = 1;
noise = 0.05;

% Noise is added to the GPS data in this loop.
for i = 1:length(Latitude)
    gpsx(i) = gpsx(i) + noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
    gpsy(i) = gpsy(i) + noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
    gpsz(i) = gpsz(i) + noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
end

% Error minimum set to something really high for Q and R matrix noise
% tuning in the IMM and EKF filters. This number is iteratively compared
% with the RMS positional errors yielded from the IMM and EKF filters and
% is replaced if the estimated error is lower than the Current_Min_Error
% value. Noise parameter tuning was conducted by varying the values of the
% filter's Q and R matrices that would minimize the RMS position error
% estimated.
Current_Min_Error = 1000;

% Variables used to index the poor gps reception areas that are simulated.
% For example, if you would like to add large corrupting noise to portions
% of the robot's trajectory you can specify the GPS data indices that this
% noise will occur during, and set the magnitude of that noise in the
% following loop.
Bad_GPS = 0; %0: No bad GPS data given to INS, 1: Bad GPS readings simulated

if Bad_GPS == 1
    GPS_LOWER_RANGE = 1000;
    GPS_UPPER_RANGE = 1100;
    
    for i = GPS_LOWER_RANGE:GPS_UPPER_RANGE
        gpsx(i) = gpsx(i) + 7*noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
        gpsy(i) = gpsy(i) + 7*noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
        gpsz(i) = gpsz(i) + 7*noise*((upper_bound-lower_bound).*rand(1) + lower_bound);
    end
end

% Sensor = 1 Trust GPS more
% Sensor = 2 Trust Compass more

% Variable used to increment plot numbers in the graphing section at the
% end of this script.
plot_num = 0;

% Round the IMU and GPS Time to 6 significant figures after the decimal
Time = round(Time,6);

if simulator == 1
    DGPS_Time = round(GPS_Time,6);
    ODOM_Time = round(GPS_Time,6);
else
    DGPS_Time = round(DGPS_Time,6);
    ODOM_Time = round(ODOM_Time,6);
end

% Generate time differential step size for each IMU measurement.
DT = zeros(1,length(Time)-1);

for ii = 1:length(Time)-1
    DT(ii) = Time(ii+1) - Time(ii);
end

% Generate time steps for GPS measurements
% DGPS_DT = zeros(1,length(DGPS_Time)-1);
% for ii = 1:length(DGPS_Time)-1
%     DGPS_DT(ii) = DGPS_Time(ii+1) - DGPS_Time(ii);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This matches GPS time stamps to IMU timestamps within theshold value

% Create a matrix of indices that label the IMU times that correspond with
% each GPS time

if simulator == 1
    DGPS_Time_Index = zeros(1,length(GPS_Time));
    ODOM_Time_Index = zeros(1,length(GPS_Time));
else
    DGPS_Time_Index = zeros(1,length(DGPS_Time));
    ODOM_Time_Index = zeros(1,length(ODOM_Time));
end

% Flag for convergence
flag = 1;

% Initial time difference threshold between DGPS time stamp and IMU time
% stamp
threshold = THRESHOLD;

while flag == 1
    for i = 1:length(DGPS_Time)
        for j = 1:length(Time)
            
            % If difference between DGPS time stamp and IMU timestamp is
            % within the threshold value, store the index of the IMU time
            % stamp as a DGPS time index.
            if abs(DGPS_Time(i) - Time(j)) < threshold
                DGPS_Time_Index(i) = j;
                break
            end
        end
    end
    
    % Check if any DGPS Time indices are equal to zero, this occurs when the
    % threshold range is too low. If there are values for all DGPS indices
    % break from the loop.
    if isempty(find(~DGPS_Time_Index,1))
        flag = 0;
    end
    
    % Increase threshold to match all time stamps
    threshold = threshold + 0.0001;
end
threshold;
theshold = THRESHOLD;
flag = 1;
while flag == 1
    for i = 1:length(ODOM_Time)
        for j = 1:length(Time)
            
            % If difference between DGPS time stamp and IMU timestamp is
            % within the threshold value, store the index of the IMU time
            % stamp as a DGPS time index.
            if abs(ODOM_Time(i) - Time(j)) < threshold
                ODOM_Time_Index(i) = j;
                break
            end
        end
    end
    
    % Check if any DGPS Time indices are equal to zero, this occurs when the
    % threshold range is too low. If there are values for all DGPS indices
    % break from the loop.
    if isempty(find(~ODOM_Time_Index,1))
        flag = 0;
    end
    
    % Increase threshold to match all time stamps
    threshold = threshold + 0.0001;
end

% This makes sure that there are no duplicate DGPS time indices. It offsets
% the index by 1 so that the DGPS measurement matches with the next IMU time
% stamp. I don't think this should cause any major issues.
for i = 1:length(DGPS_Time_Index)-1
    if DGPS_Time_Index(i+1) == DGPS_Time_Index(i)
        DGPS_Time_Index(i+1) = DGPS_Time_Index(i) + 1;
    end
end

% This makes sure that there are no duplicate ODOM time indices. It offsets
% the index by 1 so that the ODOM measurement matches with the next IMU time
% stamp.
for i = 1:length(ODOM_Time_Index)-1
    if ODOM_Time_Index(i+1) == ODOM_Time_Index(i)
        ODOM_Time_Index(i+1) = ODOM_Time_Index(i) + 1;
    end
end

% This was used to check if all values of the indices were unique.
% [C,idx]=unique(DGPS_Time_Index,'stable');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

% If tuning the INS, noise_test would typically be a row vector containing
% various noise values to be tested for each process or measurement noise
% parameter that was being tuned. For example, in the below loop, if
% noise_test = zeros(1,10), then the noise_test vector would be filled with
% 10 noise values starting at 0.0005 and ending at 0.005, with each
% successive value increasing by 0.0005. These are arbitrary numbers, but
% they illustrate the process.
noise_test = zeros(1,1);
for i = 1:length(noise_test)
    noise_test(1,i) = 0.0005*i;
end
ij = 1;

% This is the starting point for the INS filter implementations. If the
% system is being tuned, then the N matrices may include terms that are
% replaced by (noise_test(1,ij))^2. For example, the GPS position noise in
% the system R matrix may have its 3 nonzero components replaced with the
% noise_test^2 value for tuning purposes.


for ij = 1:length(noise_test)

    % Noise Figures 
    % IMM mode 1 Noise Terms
    N1=diag([0.005^2 0.004^2 0.004^2 0.015^2 0.015^2 0.015^2 0.005^2 0.005^2 0.005^2 ...
        0.003^2 0.004^2 0.003^2 0.008^2 0.008^2 0.008^2  0.0350^2  0.0350^2  0.0350^2]);
    
    % EKF Noise terms use IMM mode 1 noise terms also.
    N_EKF = N1;
    
    % IMM Mode 2 Noise Terms
    N2=diag([0.009^2 0.008^2 0.008^2 0.010^2 0.010^2 0.010^2 0.005^2 0.005^2 0.005^2 ...
        0.003^2 0.004^2 0.003^2 0.0035^2 0.0035^2 0.0035^2 0.035^2 0.035^2 0.035^2]);
    
    I34=[0 1 0 0;0 0 1 0;0 0 0 1]; % 3x5 Identity matrix
    gez=-9.81; % Gravity vector
    ge=[0 0 gez]'; % Gravity vector in world frame
    
    % Normalization of first magnetic field vector. Create a reference vector
    % for heading correction using the first Magnetometer measurement.
    
    MagNorm = norm([Magnetometer(1,1) Magnetometer(1,2) Magnetometer(1,3)]');
    mex = Magnetometer(1,1)/MagNorm;
    mey = Magnetometer(1,2)/MagNorm;
    mez = Magnetometer(1,3)/MagNorm;
    
    me=[mex mey mez]'; % Magnetic field vector in world frame
    
    % IMM Mode 1 Observer Initialization
    
    x_est1=[0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]'; % State estimate initialization
    P1=0.01*eye(16); % State covariance initialization
    Q1=N1(1:12,1:12); % Process noise
    
    R1=N1(13:18,13:18); % Measurement noise (Contains both GPS and Magnetometer noise)
    R2=N1(16:18,16:18); % Only contains Magnetometer noise figures.
    % Note that two matrices, R1 and R2, are used because there are times
    % when the GPS is not available due to measurement updates lagging
    % behind the rapidly updating magnetometer sensor.
        
    I1 = eye(16); % Identity matrix for calculations
    
    % IMM Mode 2 Observer Initialization
    x_est2=[0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]'; % State estimate initialization
    P2=0.01*eye(16); % State covariance initialization
    Q2=N2(1:12,1:12); % Process noise
    R3=N2(13:18,13:18); % Measurement noise (Contains both GPS and Magnetometer noise)
    R4=N2(16:18,16:18); % Only contains Magnetometer noise figures.
    I2 = eye(16); % Identity matrix for calculations
    
    %Data aquisition variables

    Y1=zeros(6,length(Time)-1); % Measurement vector for compass and GPS readings
  
    % State covariance estimate and measurement vector estimate for mode 1
    % of IMM.
    P_est1= zeros(16,length(Time)-1);
    Y_est1 = zeros(6,length(Time)-1);
    
    % State covariance estimate and measurement vector estimate for mode 2
    % of IMM.
    P_est2=zeros(16,length(Time)-1);
    Y_est2 = [];
    
    % Combined state estimate and combined state covariance estimate for
    % IMM.
    X_estC = zeros(16,length(Time)-1);
    P_estC = zeros(16,length(Time)-1);
    
    % IMM initialization
    
    % Mode switching probability matrix 
    p11 = 0.90;
    p12 = 1-p11;
    p22 = 0.90;
    p21 = 1-p22;
    p_ij = [p11 p12; p21 p22];
    
    % Switch for tuning IMM on or off depending on which filter is running,
    % either IMM or EKF. 1: IMM is ON. Mode 1 and 2 probabilities set to
    % 50% each since the best model for the state estimator is initially
    % uncertain.
    IMM = 1;
    if IMM ==1
        mu_i = [0.5 0.5];
    else
        mu_i = [1 0]; % Mode 1 has 100% probability, therefore we assume that the first mode is correct.
    end
    
    % Create a probabilities vector to track model likelihoods in the IMM.
    MU = zeros(2,length(Time)-1);
    MU(:,1) = mu_i';
    % Set number of modes being considered
    modes = 2;
    
    % Time synced matrices for storing x y z position estimates for comparison
    % with GPS measurements and a Yaw estimate for comparing with the seekur
    % odometry yaw estimates.
    GPS_estC =zeros(3,length(DGPS_Time_Index));
    GPS_P_estC = zeros(16,length(DGPS_Time_Index));
    Yaw_est1 = [];
    Yaw_est2 = [];

    %% Loop
    tic
    
    % Begin the state estimation process.

    % Initialize DGPS counter, this increments through the DGPS indice matrix
    DGPS_Count = 1;
    ODOM_Count = 1;
    % Use DGPS correction set to 1
    DGPS_ON = 1;
    initial_k = 1; % Count variable.
    DGPS_Data_Available = 1; % 1: DGPS data is available for this loop iteration.
    
    % Actual Acceleration readings for troubleshooting
    acc = zeros(3,length(Time)-1);
    omg = zeros(3,length(Time)-1);
    
    
    if ((tuning == 0) || (tuning == 2))
        
        for k = initial_k:length(Time)-1
            % If no time has passed, set dt to zero
            if k == 1
                dt = 0;
            else
                % Otherwise look up the new dt value, this is necessary
                % when there are inconsistencies between adjacent data
                % point timestamps.
                dt = DT(k-1);
            end
            
            
            % Create a probability normalizing vector, used to maintain a
            % total model probablity of 1 given N filter modes.
            phibar_j = zeros(1,modes);
            for j = 1:modes
                for i = 1:modes
                    phibar_j(j) = phibar_j(j) + p_ij(i,j)*mu_i(i);
                end
            end
            
            % Update the conditional probabilities for each mode, given the
            % previously determined likelihoods of each mode matching the
            % currently observed system dynamics.
            mu_ij = zeros(modes,modes);
            for i = 1:modes
                for j = 1:modes
                    mu_ij(i,j) = (1/phibar_j(j))*p_ij(i,j)*mu_i(i);
                end
            end
            
            % Calculate mixed state estimates given the current conditional
            % model probabilities, and the individual filter mode state
            % estimates.
            X_hat01 = zeros(16,1);
            X_hat02 = X_hat01;
            for j = 1:modes
                for i = 1:modes
                    if j==1 && i == 1
                        X_hat01(:,1) = X_hat01(:,1) + x_est1(:,1)*mu_ij(i,j);
                    elseif j==1 && i==2
                        X_hat01(:,1) = X_hat01(:,1) + x_est2(:,1)*mu_ij(i,j);
                    elseif j==2 && i==1
                        X_hat02(:,1) = X_hat02(:,1) + x_est1(:,1)*mu_ij(i,j);
                    else
                        X_hat02(:,1) = X_hat02(:,1) + x_est2(:,1)*mu_ij(i,j);
                    end
                end
            end
            
            % Calculate the mixed covariances for the mixed states, given
            % the conditional probabilities, individual filter model
            % estimates and the mixed state estimates.
            
            P_01 = zeros(16,16);
            P_02 = P_01;
            
            for j = 1:modes
                for i = 1:modes
                    if j==1 && i == 1
                        P_01(:,:) = P_01(:,:)+ mu_ij(i,j)*(P1(:,:) + (x_est1(:,1)-X_hat01(:,1))*(x_est1(:,1)-X_hat01(:,1))');
                    elseif j==1 && i==2
                        P_01 = P_01(:,:)+ mu_ij(i,j)*(P2 + (x_est2(:,1)-X_hat01(:,1))*(x_est2(:,1)-X_hat01(:,1))');
                    elseif j==2 && i==1
                        P_02(:,:) = P_02(:,:)+ mu_ij(i,j)*(P1(:,:) + (x_est1(:,1)-X_hat02(:,1))*(x_est1(:,1)-X_hat02(:,1))');
                    else
                        P_02 = P_02+ mu_ij(i,j)*(P2 + (x_est2(:,1)-X_hat02(:,1))*(x_est2(:,1)-X_hat02(:,1))');
                    end
                end
            end
            
            
            % Update IMU readings for process propagation
            fmx = Accelerometer(k,1);
            fmy = Accelerometer(k,2);
            fmz = Accelerometer(k,3);
            wmx = Gyroscope(k,1);
            wmy = Gyroscope(k,2);
            wmz = Gyroscope(k,3);
            
            % Update variables from estimates.
            
            % Velocities
            vx=X_hat01(4);vy=X_hat01(5);vz=X_hat01(6);
            
            % Quaternion orientations
            q0=X_hat01(7);q1=X_hat01(8);q2=X_hat01(9);q3=X_hat01(10);
            
            % Gyroscope bias
            bwx=X_hat01(11);bwy=X_hat01(12);bwz=X_hat01(13);
            
            % Accelerometer bias
            bax=X_hat01(14);bay=X_hat01(15);baz=X_hat01(16);
            
            % States stored in vectors for calculations.
            v=[vx vy vz]';
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            
            % Update rotation matrix. Rq is the quaternion parameterized
            % rotation matrix that rotates a vector from the body frame to
            % the world frame.
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            
            % Transposed rotation matrix to simplify written code that uses
            % (Rq')
            Rqp = Rq';
            
            % Linearized Matrices for Kalman Filter update, this process
            % was performed in a separate script to facilitate runtime of
            % the filter.
            F = [ 0, 0, 0, q0^2 + q1^2 - q2^2 - q3^2,         2*q1*q2 - 2*q0*q3,         2*q0*q2 + 2*q1*q3, 2*q0*vx - 2*q3*vy + 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q1*vz - 2*q0*vy - 2*q3*vx,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q0*q3 + 2*q1*q2, q0^2 - q1^2 + q2^2 - q3^2,         2*q2*q3 - 2*q0*q1, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q2*vx - 2*q1*vy - 2*q0*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q0*vx - 2*q3*vy + 2*q2*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q1*q3 - 2*q0*q2,         2*q0*q1 + 2*q2*q3, q0^2 - q1^2 - q2^2 + q3^2, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q3*vy - 2*q0*vx - 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                   -2*gez*q2,                    2*gez*q3,                   -2*gez*q0,                    2*gez*q1,     0,     0,     0, -1,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q1,                    2*gez*q0,                    2*gez*q3,                    2*gez*q2,     0,     0,     0,  0, -1,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q0,                   -2*gez*q1,                   -2*gez*q2,                    2*gez*q3,     0,     0,     0,  0,  0, -1;
                0, 0, 0,                         0,                         0,                         0,                           0,               bwx/2 - wmx/2,               bwy/2 - wmy/2,               bwz/2 - wmz/2,  q1/2,  q2/2,  q3/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmx/2 - bwx/2,                           0,               wmz/2 - bwz/2,               bwy/2 - wmy/2, -q0/2,  q3/2, -q2/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmy/2 - bwy/2,               bwz/2 - wmz/2,                           0,               wmx/2 - bwx/2, -q3/2, -q0/2,  q1/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmz/2 - bwz/2,               wmy/2 - bwy/2,               bwx/2 - wmx/2,                           0,  q2/2, -q1/2, -q0/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0];
            
            
            
            % Linearized process noise matrix.
            
            G = [0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                -1,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0, -1,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0, -1, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q1/2,  q2/2,  q3/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q0/2,  q3/2, -q2/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q3/2, -q0/2,  q1/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q2/2, -q1/2, -q0/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 1, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 1, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 1;
                0,  0,  0, 1, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 1, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 1,     0,     0,     0, 0, 0, 0];
            
            % Linearized output matrix.
            H = [0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
                       
            % Input estimates, remove biases and account for gravity
            
            % Gyroscope input estimates.
            wmx = wmx - bwx;
            wmy = wmy - bwy;
            wmz = wmz - bwz;
            
            % Accelerometer input estimates.
            fmx = fmx - bax + Rqp(1,:)*ge;
            fmy = fmy - bay + Rqp(2,:)*ge;
            fmz = fmz - baz + Rqp(3,:)*ge;
            
            % Inputs stored in vector for calculations.
            acc(:,k) = [fmx fmy fmz]';
            omg(:,k) = [wmx wmy wmz]';
            
            % Estimate updates
            dp = Rq*v;
            dv = acc(:,k);
            dq = 0.5*skewqr([0 (omg(:,k))'])*q;
            dba = [0 0 0]';
            dbw = [0 0 0]';
            f=[dp; dv; dq; dbw; dba];
            
            % Uncorrected estimate update
            x_est1=X_hat01+f*dt;
            
            % Normalize quaternion values
            x_est1(7:10)=quatnormalize(x_est1(7:10)')';
            
            % Reassign updated state values to state variables
            
            % Position estimates
            px=x_est1(1);py=x_est1(2);pz=x_est1(3);
            
            % Velocity estimates
            vx=x_est1(4);vy=x_est1(5);vz=x_est1(6);
            
            % Quaternion estimates
            q0=x_est1(7);q1=x_est1(8);q2=x_est1(9);q3=x_est1(10);
            
            % Gyroscope bias estimates
            bwx=x_est1(11);bwy=x_est1(12);bwz=x_est1(13);
            
            % Accelerometer bias estimates
            bax=x_est1(14);bay=x_est1(15);baz=x_est1(16);
            
            % Updated rotation states.
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            
            % Observer Correction
            
            % If we are using DGPS correction and the current k (IMU index) is equal
            % to the next DGPS time index, correct the estimate using DGPS and the
            % Magnetometer.
            
            % If DGPS count exceeds the number of DGPS time indices, then
            % we've run out of DGPS measurements and therefore the DGPS
            % availability is set to 0 (DGPS unavailable).
            if ((DGPS_ON == 1) && (DGPS_Count > length(DGPS_Time_Index)))
                DGPS_Data_Available = 0;
            end
            
            % If DPGS data is available, check if the current IMU time
            % increment corresponds to the current DGPS time index. If yes,
            % set the GPS_Flag variable to 1 (GPS is being used this time
            % increment), and update the filter with both GPS and
            % magnetometer data.
            
            if DGPS_Data_Available == 1
                if((DGPS_ON == 1) && (k == DGPS_Time_Index(DGPS_Count)))
                                        
                    GPS_Flag = 1;
                    
                    H = [ 1, 0, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 1, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 1, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
                    
                    % Update DGPS Measurement
                    pm = [gpsx(DGPS_Count) gpsy(DGPS_Count) gpsz(DGPS_Count)]';
                    
                    % Normalize the next Magnetometer reading.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    
                    % Update the Magnetometer reading.
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    y=[pm' mg']';
                    
                    % Compute Normalization factor for the magnetometer estimate.
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    % Generate measurement estimate
                    y_est = [px py pz Rq(:,1)'*me/est_norm Rq(:,2)'*me/est_norm Rq(:,3)'*me/est_norm]';
                    
                    % Covariance matrix update
                    P1 = (I1+F*dt)*P1*(I1+F*dt)' + G*Q1*G'*dt^2;
                    
                    % Calculate Kalman gain
                    % R1 includes DGPS Noise figures
                    S1 = H*P1*H' + R1;
                    K1 = P1*H'*(S1^-1);
                    
                    % Correct estimate
                    x_est1 = x_est1 + K1*(y-y_est);
                    
                    Z1 = K1*(y-y_est);
                    % Correct covariance Matrix
                    P1 = P1 - K1*H*P1;
                    %         Y=[Y y];
                    
                    % Increment DGPS count to look at next DGPS reading.
                    % DGPS_Count = DGPS_Count + 1;
                    
                    Y1(:,k) = y;
                    Y_est1(:,k) = y_est;
                    
                else
                    GPS_Flag = 0;
                    %Otherwise, if the next DGPS Measurement does not correspond to the
                    % current IMU reading, only use the Mag in the update.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    
                    y=[mg']';
                    % Generate measurement estimate
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    y_est = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                    
                    % Covariance matrix update
                    P1 = (I1+F*dt)*P1*(I1+F*dt)' + G*Q1*G'*dt^2;
                    
                    % Calculate Kalman gain
                    % R2 Only includes Magnetometer noise
                    S1 = H*P1*H' + R2;
                    K1 = P1*H'*(S1^-1);
                    % Correct estimate
                    x_est1 = x_est1 + K1*(y-y_est);
                    
                    Z1 = K1*(y-y_est);
                    
                    % Correct covariance Matrix
                    P1 = P1 - K1*H*P1;
                    y = [ 0 0 0 y' ]';
                    Y1(:,k) = y;
                    y_est = [ 0 0 0 y_est' ]';
                    Y_est1(:,k) = y_est;
                end
            end
            if DGPS_Data_Available == 0
                GPS_Flag = 0;
                % Otherwise, if the next DGPS Measurement does not correspond to the
                % current IMU reading, only use the Mag in the update.
                MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                
                y=[mg']';
                
                % Generate measurement estimate
                est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                
                y_est = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                
                % Covariance matrix update
                P1 = (I1+F*dt)*P1*(I1+F*dt)' + G*Q1*G'*dt^2;
                
                % Calculate Kalman gain
                % R2 Only includes Magnetometer noise
                S1 = H*P1*H' + R2;
                K1 = P1*H'*(S1^-1);
                % Correct estimate
                x_est1 = x_est1 + K1*(y-y_est);
                
                Z1 = K1*(y-y_est);
                % Correct covariance Matrix
                P1 = P1 - K1*H*P1;
                y = [ 0 0 0 y' ]';
                Y1(:,k) = y;
                y_est = [ 0 0 0 y_est' ]';
                Y_est1(:,k) = y_est;
            end
            
            x_est1(7:10)=quatnormalize(x_est1(7:10)')';
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Begin estimation process for second EKF Model in IMM.
            
            % Update IMU readings for process propagation
            fmx = Accelerometer(k,1);
            fmy = Accelerometer(k,2);
            fmz = Accelerometer(k,3);
            wmx = Gyroscope(k,1);
            wmy = Gyroscope(k,2);
            wmz = Gyroscope(k,3);
            
            % Update variables from estimates.
            vx=X_hat02(4);vy=X_hat02(5);vz=X_hat02(6);
            q0=X_hat02(7);q1=X_hat02(8);q2=X_hat02(9);q3=X_hat02(10);
            bwx=X_hat02(11);bwy=X_hat02(12);bwz=X_hat02(13);
            bax=X_hat02(14);bay=X_hat02(15);baz=X_hat02(16);
            v=[vx vy vz]';
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            
            % Update rotation matrix.
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            Rqp = Rq';
            
            % Linearized Matrices for Kalman Filter update
            F = [ 0, 0, 0, q0^2 + q1^2 - q2^2 - q3^2,         2*q1*q2 - 2*q0*q3,         2*q0*q2 + 2*q1*q3, 2*q0*vx - 2*q3*vy + 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q1*vz - 2*q0*vy - 2*q3*vx,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q0*q3 + 2*q1*q2, q0^2 - q1^2 + q2^2 - q3^2,         2*q2*q3 - 2*q0*q1, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q2*vx - 2*q1*vy - 2*q0*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q0*vx - 2*q3*vy + 2*q2*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q1*q3 - 2*q0*q2,         2*q0*q1 + 2*q2*q3, q0^2 - q1^2 - q2^2 + q3^2, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q3*vy - 2*q0*vx - 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                   -2*gez*q2,                    2*gez*q3,                   -2*gez*q0,                    2*gez*q1,     0,     0,     0, -1,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q1,                    2*gez*q0,                    2*gez*q3,                    2*gez*q2,     0,     0,     0,  0, -1,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q0,                   -2*gez*q1,                   -2*gez*q2,                    2*gez*q3,     0,     0,     0,  0,  0, -1;
                0, 0, 0,                         0,                         0,                         0,                           0,               bwx/2 - wmx/2,               bwy/2 - wmy/2,               bwz/2 - wmz/2,  q1/2,  q2/2,  q3/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmx/2 - bwx/2,                           0,               wmz/2 - bwz/2,               bwy/2 - wmy/2, -q0/2,  q3/2, -q2/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmy/2 - bwy/2,               bwz/2 - wmz/2,                           0,               wmx/2 - bwx/2, -q3/2, -q0/2,  q1/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmz/2 - bwz/2,               wmy/2 - bwy/2,               bwx/2 - wmx/2,                           0,  q2/2, -q1/2, -q0/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0];
            
            
            G = [0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                -1,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0, -1,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0, -1, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q1/2,  q2/2,  q3/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q0/2,  q3/2, -q2/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q3/2, -q0/2,  q1/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q2/2, -q1/2, -q0/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 1, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 1, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 1;
                0,  0,  0, 1, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 1, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 1,     0,     0,     0, 0, 0, 0];
            
            H = [0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
                        
            % Input estimates, remove biases and account for gravity
            wmx = wmx - bwx;
            wmy = wmy - bwy;
            wmz = wmz - bwz;
            
            fmx = fmx - bax + Rqp(1,:)*ge;
            fmy = fmy - bay + Rqp(2,:)*ge;
            fmz = fmz - baz + Rqp(3,:)*ge;
            
            acc(:,k) = [fmx fmy fmz]';
            omg(:,k) = [wmx wmy wmz]';
            
            % Estimate updates
            dp = Rq*v;
            dv = acc(:,k);
            dq = 0.5*skewqr([0 (omg(:,k))'])*q;
            dba = [0 0 0]';
            dbw = [0 0 0]';
            f=[dp; dv; dq; dbw; dba];
            
            % Uncorrected estimate update
            x_est2=X_hat02+f*dt;
            
            % Normalize quaternion values
            x_est2(7:10)=quatnormalize(x_est2(7:10)')';
            
            % Reassign updated state values to state variables
            px=x_est2(1);py=x_est2(2);pz=x_est2(3);
            vx=x_est2(4);vy=x_est2(5);vz=x_est2(6);
            q0=x_est2(7);q1=x_est2(8);q2=x_est2(9);q3=x_est2(10);
            bwx=x_est2(11);bwy=x_est2(12);bwz=x_est2(13);
            bax=x_est2(14);bay=x_est2(15);baz=x_est2(16);
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            
            % Observer Correction
            
            % If we are using DGPS correction and the current k (IMU index) is equal
            % to the next DGPS time index, correct the estimate using DGPS and the
            % Magnetometer.
            
            if ((DGPS_ON == 1) && (DGPS_Count > length(DGPS_Time_Index)))
                DGPS_Data_Available = 0;
            end
            
            if DGPS_Data_Available == 1
                if((DGPS_ON == 1) && (k == DGPS_Time_Index(DGPS_Count)))
                    GPS_Flag = 1;
                    H = [ 1, 0, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 1, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 1, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
                    
                    %Update DGPS Measurement
                    pm = [gpsx(DGPS_Count) gpsy(DGPS_Count) gpsz(DGPS_Count)]';
                    
                    % Normalize the next Magnetometer reading.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    
                    % Update the Magnetometer reading.
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    y=[pm' mg']';
                    
                    % Compute Normalization factor for the magnetometer estimate.
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    % Generate measurement estimate
                    y_est = [px py pz Rq(:,1)'*me/est_norm Rq(:,2)'*me/est_norm Rq(:,3)'*me/est_norm]';
                    
                    % Covariance matrix update
                    P2 = (I2+F*dt)*P2*(I2+F*dt)' + G*Q2*G'*dt^2;
                    
                    % Calculate Kalman gain
                    % R1 includes DGPS Noise figures
                    S2 = H*P2*H' + R3;
                    K2 = P2*H'*(S2^-1);
                    
                    % Correct estimate
                    x_est2 = x_est2 + K2*(y-y_est);
                    
                    Z2 = K2*(y-y_est);
                    % Correct covariance Matrix
                    P2 = P2 - K2*H*P2;
                    % Y=[Y y];
                    
                    % Increment DGPS count to look at next DGPS reading.
                    DGPS_Count = DGPS_Count + 1;
                    % Y2 = [Y2 y];
                    % Y_est2 = [Y_est2 y_est];
                    
                else
                    
                    GPS_Flag = 0;
                    %Otherwise, if the next DGPS Measurement does not correspond to the
                    % current IMU reading, only use the Mag in the update.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    
                    y=(mg')';
                    % Generate measurement estimate
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    y_est = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                    
                    % Covariance matrix update
                    P2 = (I2+F*dt)*P2*(I2+F*dt)' + G*Q2*G'*dt^2;
                    
                    %         Calculate Kalman gain
                    % R2 Only includes Magnetometer noise
                    S2 = H*P2*H' + R2;
                    K2 = P2*H'*(S2^-1);
                    % Correct estimate
                    x_est2 = x_est2 + K2*(y-y_est);
                    
                    Z2 = K2*(y-y_est);
                    % Correct covariance Matrix
                    P2 = P2 - K2*H*P2;
                    y = [ 0 0 0 y' ]';
                    % Y2 = [Y2 y];
                    y_est = [ 0 0 0 y_est' ]';
                    % Y_est2 = [Y_est2 y_est];
                end
            end
            if DGPS_Data_Available == 0
                
                GPS_Flag = 0;
                %Otherwise, if the next DGPS Measurement does not correspond to the
                % current IMU reading, only use the Mag in the update.
                MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                
                y=[mg']';
                % Generate measurement estimate
                est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                
                y_est = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                
                % Covariance matrix update
                P2 = (I2+F*dt)*P2*(I2+F*dt)' + G*Q2*G'*dt^2;
                
                %         Calculate Kalman gain
                % R2 Only includes Magnetometer noise
                S2 = H*P2*H' + R2;
                K2 = P2*H'*(S2^-1);
                % Correct estimate
                x_est2 = x_est2 + K2*(y-y_est);
                
                Z2 = K2*(y-y_est);
                % Correct covariance Matrix
                P2 = P2 - K2*H*P2;
                y = [ 0 0 0 y' ]';
                %         Y2 = [Y2 y];
                y_est = [ 0 0 0 y_est' ]';
                %         Y_est2 = [Y_est2 y_est];
            end
            
            x_est2(7:10)=quatnormalize(x_est2(7:10)')';
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % IMM Mixing and likelihood following individual filter
            % estimates starts here.
            
            
            % Compute likelihood of each model (Different matrix 
            % dimensionality requires two cases to be considered, GPS on
            % and GPS off.
            
            if GPS_Flag == 1
                if det(2*pi*S1)>0
                    lambda1 = (1/(sqrt(det(2*pi*S1))))*exp((-0.5*((Z1(1:6,1))')*((S1^-1)*(Z1(1:6,1)))));
                end
                if det(2*pi*S2)> 0
                    lambda2 = (1/(sqrt(det(2*pi*S2))))*exp((-0.5*((Z2(1:6,1))')*((S2^-1)*((Z2(1:6,1))))));
                end
            end
            
            if GPS_Flag == 0
                if det(2*pi*S1)>0
                    lambda1 = (1/(sqrt(det(2*pi*S1))))*exp((-0.5*((Z1(1:3,1))')*((S1^-1)*(Z1(1:3,1)))));
                end
                if det(2*pi*S2)> 0
                    lambda2 = (1/(sqrt(det(2*pi*S2))))*exp((-0.5*((Z2(1:3,1))')*((S2^-1)*((Z2(1:3,1))))));
                end
            end
            
            % To avoid discontinuities, this ensures that one model is being
            % used at all times.
            
            if lambda1 < 0.001 && lambda2< 0.001
                lambda1 = 0.5;
                lambda2 = 0.5;
            end
            if lambda1 > 0 && lambda2 < 0.0001
                lambda1 = 1;
                lambda2 = 0;
            end
            if lambda2 > 0 && lambda1 < 0.0001
                lambda1 = 0;
                lambda2 = 1;
            end
            
            % Compute normalization constant
            c = lambda1*phibar_j(1) + lambda2*phibar_j(2);
            
            % Update the probability of each model
            mu_i(1) = (1/c)*lambda1*phibar_j(1);
            mu_i(2) = (1/c)*lambda2*phibar_j(2);
            
            % Store updated model probabilities in matrix for analysis.
            MU(:,k) = [mu_i(1) mu_i(2)]';
            
            % Calculate the final combined state estimate and combined 
            % state covariance estimates of the two IMM models using the 
            % current model probabilities and individual filter estimates.
            
            x_estC = x_est1*mu_i(1);
            x_estC = x_estC + x_est2*mu_i(2);
            
            P_C(:,:) = mu_i(1)*(P1(:,:)+(x_est1(:,1)-x_estC(:,1))*(x_est1(:,1)-x_estC(:,1))');
            P_C = P_C + mu_i(2)*(P2 +(x_est2(:,1)-x_estC(:,1))*(x_est2(:,1)-x_estC(:,1))');
            
            P_est1(:,k)=sqrt(diag(P1));
            P_est2(:,k)=sqrt(diag(P2));
            X_estC(:,k) = x_estC;
            P_estC(:,k)=sqrt(diag(P_C));
                      
        end
        
        % This section obtains estimator data for corresponding GPS time 
        % stamps to be used for plotting purposes.

        gps_length_flag = 0;
        for i = 1:length(DGPS_Time_Index)
            if DGPS_Time_Index(i) > length(X_estC)
                gps_length_flag = 1;
                break
            end
            GPS_estC(:,i) = X_estC(1:3,DGPS_Time_Index(i));
            GPS_P_estC(:,i) = P_estC(:,DGPS_Time_Index(i));
        end
        
        % Compute positional RMS error for the calculated estimates
        % compared with the original noise free GPS data.
        if cropping == 0
            RMSEx = (sqrt(mean((GPS_estC(1,1:end-1) - gpsx_true(1,1:end-1)).^2)));
            RMSEy = (sqrt(mean((GPS_estC(2,1:end-1) - gpsy_true(1,1:end-1)).^2)));
            RMSEz = (sqrt(mean((GPS_estC(3,1:end-1) - gpsz_true(1,1:end-1)).^2)));
        else
            'IMM Cropped'
            RMSEx = (sqrt(mean((GPS_estC(1,crop_num) - gpsx_true(1,crop_num)).^2)));
            RMSEy = (sqrt(mean((GPS_estC(2,crop_num) - gpsy_true(1,crop_num)).^2)));
            RMSEz = (sqrt(mean((GPS_estC(3,crop_num) - gpsz_true(1,crop_num)).^2)));
        end
        
        RMSE_IMM = sqrt(RMSEx^2 + RMSEy^2 + RMSEz^2)
        RMSE_IMMx = RMSEx;
        RMSE_IMMy = RMSEy;
        RMSE_IMMz = RMSEz;
        
        %%% Used during tuning to minimize error with noise parameters
        %         if RMSE_IMM < Current_Min_Error
        %             Current_Min_Error = RMSE_IMM;
        %             noise_index = ij;
        %         end
        %
        
        if Graphing == 1;
            
            %% Plotting
            X = X_estC;
            toc
            t = linspace(0,k*dt,k);
            
            figure(1+plot_num)
            plot(X_estC(1,:),X_estC(2,:),'-r','LineWidth',2,'MarkerSize',2)
            title('Predicted vs actual Positions')
            ylabel('Position [m]');
            xlabel('Time [s]');
            hold on
            if gps_length_flag == 0
                plot(gpsx_true,gpsy_true,'-b','LineWidth',2,'MarkerSize',2)
            else
                plot(gpsx_true(1,1:end-1),gpsy_true(1,1:end-1),'-b','LineWidth',2,'MarkerSize',2)
            end
            figure(2+plot_num)
            plot(X_estC(4,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(X_estC(5,:),'-b','LineWidth',2,'MarkerSize',2)
            hold on
            plot(X_estC(6,:),'-g','LineWidth',2,'MarkerSize',2)
            title('Velocities')
            ylabel('Velocity [m/s]');
            xlabel('Time [s]');
            
            
            figure(3+plot_num)
            plot(Y_est1(4,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y1(4,:),'-b','LineWidth',2,'MarkerSize',2)
            title('Predicted vs actual Magnetometer Measurements')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated X Flux','Actual X Flux')
            
            figure(4+plot_num)
            plot(Y_est1(5,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y1(5,:),'-b','LineWidth',2,'MarkerSize',2)
            hold on
            title('Predicted vs actual Magnetometer Measurements')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated Y Flux','Actual Y Flux')
            
            figure(5+plot_num)
            plot(Y_est1(6,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y1(6,:),'-b','LineWidth',2,'MarkerSize',2)
            hold on
            title('Predicted vs actual Magnetometer Measurements')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated Z Flux','Actual Z Flux')

            title('Predicted vs actual Positions')
            ylabel('Position [m]');
            xlabel('Time [s]');
            
            % Result visualization
            figure(8+plot_num)
            plot((X_estC(4:6,:))','--+','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Velocities')
            ylabel('Velocity [m/s]');
            xlabel('Time [s]');
            legend('Estimated X Velocity','Estimated Y Velocity','Estimated Z Velocity')
            
            figure(9+plot_num)
            [yaw pitch roll]=quat2angle(X_estC(7:10,:)');
            plot(([yaw pitch roll]),'--+','LineWidth',2,'MarkerSize',2)
            %     plot((X_estC(7:10,:))','--+')
            grid on
            title('Predicted vs actual Quaternions')
            ylabel('Quaternion Value');
            xlabel('Time [s]');
            legend('Yaw','Pitch','Roll')
            
            figure(10+plot_num)
            plot((X_estC(11:13,:))','--+','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Gyro Bias')
            ylabel('Gyro Bias [radians/s]');
            xlabel('Time [s]');
            legend('Estimated X Bias','Estimated Y Bias','Estimated Z Bias')
            
            
            figure(11+plot_num)
            plot((X_estC(14:16,:))','--+','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Accelerometer Bias')
            ylabel('Accelerometer Bias [m/s^2]');
            xlabel('Time [s]');
            legend('Estimated X Bias','Estimated Y Bias','Estimated Z Bias')
            %%
            figure(12+plot_num)
            
            if gps_length_flag == 0
                semilogy(abs(([gpsx_true(1:end); gpsy_true(1:end); gpsz_true(1:end)]'-GPS_estC').^2))
                hold on
                semilogy(2*GPS_P_estC(1:3,:)','--+')
                grid on
                title('Position error')
                ylabel('Position Error [m]');
                xlabel('Time [s]');
                legend('X Error','Y Error','Z Error','95% X Error Confidence Bound',...
                    '95% Y Error Confidence Bound','95% Z Error Confidence Bound')
            else
                semilogy(abs(([gpsx_true(1:end-1); gpsy_true(1:end-1); gpsz_true(1:end-1)]'-GPS_estC(:,1:end-1)').^2))
                hold on
                semilogy(2*GPS_P_estC(1:3,:)','--+')
                grid on
                title('Position error')
                ylabel('Position Error [m]');
                xlabel('Time [s]');
                legend('X Error','Y Error','Z Error','95% X Error Confidence Bound',...
                    '95% Y Error Confidence Bound','95% Z Error Confidence Bound')
            end

            %%
            %
            % legend('Yaw','Odom Yaw')
            if gps_length_flag == 0;
                %                 if cropping == 0
                %                     RMSEx = (sqrt(mean((GPS_estC(1,:) - gpsx_true(1,1:end)).^2)));
                %                     RMSEy = (sqrt(mean((GPS_estC(2,:) - gpsy_true(1,1:end)).^2)));
                %                     RMSEz = (sqrt(mean((GPS_estC(3,:) - gpsz_true(1,1:end)).^2)));
                %                 else
                %                     'IMM Cropped'
                %                     RMSEx = (sqrt(mean((GPS_estC(1,1:crop_num) - gpsx_true(1,1:crop_num)).^2)));
                %                     RMSEy = (sqrt(mean((GPS_estC(2,1:crop_num) - gpsy_true(1,1:crop_num)).^2)));
                %                     RMSEz = (sqrt(mean((GPS_estC(3,1:crop_num) - gpsz_true(1,1:crop_num)).^2)));
                %                 end
                %
                figure(15+plot_num)
                plot3(GPS_estC(1,:),GPS_estC(2,:),GPS_estC(3,:),'-r','LineWidth',2,'MarkerSize',2)
                hold on
                plot3(gpsx_true, gpsy_true, gpsz_true,'-b','LineWidth',2,'MarkerSize',2)
                ylabel('Y Position [m]');
                xlabel('X Position [m]');
                zlabel('Z Position [m]');
                title('Robot Position')
                legend('Estimated Robot Position', 'Robot DGPS Position')
                view(2)
            else
                %                 if cropping == 0
                %                     RMSEx = (sqrt(mean((GPS_estC(1,1:end-1) - gpsx_true(1,1:end-1)).^2)));
                %                     RMSEy = (sqrt(mean((GPS_estC(2,1:end-1) - gpsy_true(1,1:end-1)).^2)));
                %                     RMSEz = (sqrt(mean((GPS_estC(3,1:end-1) - gpsz_true(1,1:end-1)).^2)));
                %                 else
                %                     'IMM Cropped'
                %                     RMSEx = (sqrt(mean((GPS_estC(1,1:crop_num) - gpsx_true(1,1:crop_num)).^2)));
                %                     RMSEy = (sqrt(mean((GPS_estC(2,1:crop_num) - gpsy_true(1,1:crop_num)).^2)));
                %                     RMSEz =(sqrt(mean((GPS_estC(3,1:crop_num) - gpsz_true(1,1:crop_num)).^2)));
                %                 end
                
                
                figure(15+plot_num)
                plot3(GPS_estC(1,1:end-1),GPS_estC(2,1:end-1),GPS_estC(3,1:end-1),'-r','LineWidth',2,'MarkerSize',2)
                hold on
                plot3(gpsx_true(1,1:end-1), gpsy_true(1,1:end-1), gpsz_true(1,1:end-1),'-b','LineWidth',2,'MarkerSize',2)
                ylabel('Y Position [m]');
                xlabel('X Position [m]');
                title('Robot Position')
                legend('Estimated Robot Position', 'Robot DGPS Position')
                view(2)
            end
            
            
            figure(16+plot_num)
            plot(MU(1,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(MU(2,:),'-b','LineWidth',2,'MarkerSize',2)
            ylabel('Model Probability');
            xlabel('Time [s]');
            legend('Model 1 Probability','Model 2 Probability')
            
            figure(17+plot_num)
            plot(X_estC(1,:),'-r','LineWidth',2,'MarkerSize',2)
            
            hold on
            plot(gpsx_stretched,'-b','LineWidth',2,'MarkerSize',2 )
            title('Actual vs Estimated X Positions')
            ylabel('X Position [m]');
            xlabel('Time [s]');
            
            figure(18+plot_num)
            plot(X_estC(2,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(gpsy_stretched,'-b','LineWidth',2,'MarkerSize',2 )
            title('Actual vs Estimated Y Positions')
            ylabel('Y Position [m]');
            xlabel('Time [s]');
            
            figure(19+plot_num)
            plot(X_estC(3,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(gpsz_stretched,'-b','LineWidth',2,'MarkerSize',2 )
            title('Actual vs Estimated Z Positions')
            ylabel('Z Position [m]');
            xlabel('Time [s]');
            %     figure(17+plot_num)
            %     plot(Quality)
            %     title('DGPS Solution Quality')
            %     xlabel('Data Points')
            %     ylabel('Solution Quality')
        end
    end
    
    %%
    if ((tuning == 0) || (tuning == 1))
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % Single EKF INS Implementation for the Seekur Jr Robot
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % This was used to check if all values of the indices were unique.
        % [C,idx]=unique(DGPS_Time_Index,'stable');
        
        I34=[0 1 0 0;0 0 1 0;0 0 0 1]; % 3x5 Identity matrix
        gez=-9.81; % Gravity vector
        ge=[0 0 gez]'; % Gravity vector in world frame
        %mex=10; % Magnetic field vector
        
        % Normalization of first magnetic field vector. Create a reference vector
        % for heading correction using the first Magnetometer measurement.
        MagNorm = norm([Magnetometer(1,1) Magnetometer(1,2) Magnetometer(1,3)]');
        mex = Magnetometer(1,1)/MagNorm;
        mey = Magnetometer(1,2)/MagNorm;
        mez = Magnetometer(1,3)/MagNorm;
        
        me=[mex mey mez]'; % Magnetic field vector in world frame
        
        %Observer parameters
        %x_est=[0 0 0 0 0 0 1 0 0 0 Gyroscope(1,1) Gyroscope(1,2) Gyroscope(1,3) Accelerometer(1,1) Accelerometer(1,2) Accelerometer(1,3)]';   
        x_est_EKF=[0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]';    
        px=x_est_EKF(1);py=x_est_EKF(2);pz=x_est_EKF(3);
        vx=x_est_EKF(4);vy=x_est_EKF(5);vz=x_est_EKF(6);
        q0=x_est_EKF(7);q1=x_est_EKF(8);q2=x_est_EKF(9);q3=x_est_EKF(10);
        bwx=x_est_EKF(11);bwy=x_est_EKF(12);bwz=x_est_EKF(13);
        bax=x_est_EKF(14);bay=x_est_EKF(15);baz=x_est_EKF(16);
        P_EKF=0.01*eye(16);                   %state covariance - require initialization
        Q_EKF=N_EKF(1:12,1:12);                   %process noise
        R1_EKF=N_EKF(13:18,13:18);                 %measurement noise
        R2_EKF=N_EKF(16:18,16:18);
        I = eye(16);
        
        %Data aquisition variables
        X_EKF=[];
        U_EKF=[];
        Y_EKF=[];
        X_est_EKF=[];
        P_est_EKF=[];
        Y_est_EKF = [];
        
        % Time synced matrices for storing x y z position estimates for comparison
        % with GPS measurements and a Yaw estimate for comparing with the seekur
        % odometry yaw estimates.
        GPS_est_EKF = [];
        GPS_P_est_EKF = [];
        Yaw_est_EKF = [];

        %% Loop
        tic
        % Initialize DGPS counter, this increments through the DGPS indice matrix
        DGPS_Count = 1;
        ODOM_Count = 1;
        % Use DGPS correction set to 1
        DGPS_ON = 1;
        initial_k = 1;
        DGPS_Data_Available = 1;
        %         GPS_est_EKF = zeros(3,1970);
        %         GPS_P_est_EKF = zeros(16,1970);
        %         Y_est_EKF = zeros(6,5907);
        %         Y_EKF = Y_est_EKF;
        % Actual Acceleration readings for troubleshooting
        acc = zeros(3,length(Time)-1);
        omg = zeros(3,length(Time)-1);
        for k = initial_k:length(Time)-1
            % If no time has passed, set dt to zero
            if k == 1
                dt = 0;
            else
                % Otherwise look up the new dt value
                dt = DT(k-1);
            end
            
            % Update IMU readings for process propagation
            fmx = Accelerometer(k,1);
            fmy = Accelerometer(k,2);
            fmz = Accelerometer(k,3);
            wmx = Gyroscope(k,1);
            wmy = Gyroscope(k,2);
            wmz = Gyroscope(k,3);
            
            % Update variables from estimates.
            px=x_est_EKF(1);py=x_est_EKF(2);pz=x_est_EKF(3);
            vx=x_est_EKF(4);vy=x_est_EKF(5);vz=x_est_EKF(6);
            q0=x_est_EKF(7);q1=x_est_EKF(8);q2=x_est_EKF(9);q3=x_est_EKF(10);
            bwx=x_est_EKF(11);bwy=x_est_EKF(12);bwz=x_est_EKF(13);
            bax=x_est_EKF(14);bay=x_est_EKF(15);baz=x_est_EKF(16);
            v=[vx vy vz]';
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            
            % Update rotation matrix.
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            Rqp = Rq';
            
            % Linearized Matrices for Kalman Filter update
            F_EKF = [ 0, 0, 0, q0^2 + q1^2 - q2^2 - q3^2,         2*q1*q2 - 2*q0*q3,         2*q0*q2 + 2*q1*q3, 2*q0*vx - 2*q3*vy + 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q1*vz - 2*q0*vy - 2*q3*vx,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q0*q3 + 2*q1*q2, q0^2 - q1^2 + q2^2 - q3^2,         2*q2*q3 - 2*q0*q1, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q2*vx - 2*q1*vy - 2*q0*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz, 2*q0*vx - 2*q3*vy + 2*q2*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,         2*q1*q3 - 2*q0*q2,         2*q0*q1 + 2*q2*q3, q0^2 - q1^2 - q2^2 + q3^2, 2*q1*vy - 2*q2*vx + 2*q0*vz, 2*q3*vx + 2*q0*vy - 2*q1*vz, 2*q3*vy - 2*q0*vx - 2*q2*vz, 2*q1*vx + 2*q2*vy + 2*q3*vz,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                   -2*gez*q2,                    2*gez*q3,                   -2*gez*q0,                    2*gez*q1,     0,     0,     0, -1,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q1,                    2*gez*q0,                    2*gez*q3,                    2*gez*q2,     0,     0,     0,  0, -1,  0;
                0, 0, 0,                         0,                         0,                         0,                    2*gez*q0,                   -2*gez*q1,                   -2*gez*q2,                    2*gez*q3,     0,     0,     0,  0,  0, -1;
                0, 0, 0,                         0,                         0,                         0,                           0,               bwx/2 - wmx/2,               bwy/2 - wmy/2,               bwz/2 - wmz/2,  q1/2,  q2/2,  q3/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmx/2 - bwx/2,                           0,               wmz/2 - bwz/2,               bwy/2 - wmy/2, -q0/2,  q3/2, -q2/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmy/2 - bwy/2,               bwz/2 - wmz/2,                           0,               wmx/2 - bwx/2, -q3/2, -q0/2,  q1/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,               wmz/2 - bwz/2,               wmy/2 - bwy/2,               bwx/2 - wmx/2,                           0,  q2/2, -q1/2, -q0/2,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0;
                0, 0, 0,                         0,                         0,                         0,                           0,                           0,                           0,                           0,     0,     0,     0,  0,  0,  0];
            
            
            G_EKF = [0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                -1,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0, -1,  0, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0, -1, 0, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q1/2,  q2/2,  q3/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q0/2,  q3/2, -q2/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0, -q3/2, -q0/2,  q1/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,  q2/2, -q1/2, -q0/2, 0, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 1, 0, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 1, 0;
                0,  0,  0, 0, 0, 0,     0,     0,     0, 0, 0, 1;
                0,  0,  0, 1, 0, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 1, 0,     0,     0,     0, 0, 0, 0;
                0,  0,  0, 0, 0, 1,     0,     0,     0, 0, 0, 0];
            
            H_EKF = [0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
            
            
            % Input estimates, remove biases and account for gravity
            wmx = wmx - bwx;
            wmy = wmy - bwy;
            wmz = wmz - bwz;
            
            fmx = fmx - bax + Rqp(1,:)*ge;
            fmy = fmy - bay + Rqp(2,:)*ge;
            fmz = fmz - baz + Rqp(3,:)*ge;
            
            acc(:,k) = [fmx fmy fmz]';
            omg(:,k) = [wmx wmy wmz]';
            
            % Estimate updates
            dp = Rq*v;
            dv = acc(:,k);
            dq = 0.5*skewqr([0 (omg(:,k))'])*q;
            dba = [0 0 0]';
            dbw = [0 0 0]';
            f=[dp; dv; dq; dbw; dba];
            
            % Uncorrected estimate update
            x_est_EKF=x_est_EKF+f*dt;
            
            % Normalize quaternion values
            x_est_EKF(7:10)=quatnormalize(x_est_EKF(7:10)')';
            
            % Reassign updated state values to state variables
            px=x_est_EKF(1);py=x_est_EKF(2);pz=x_est_EKF(3);
            vx=x_est_EKF(4);vy=x_est_EKF(5);vz=x_est_EKF(6);
            q0=x_est_EKF(7);q1=x_est_EKF(8);q2=x_est_EKF(9);q3=x_est_EKF(10);
            bwx=x_est_EKF(11);bwy=x_est_EKF(12);bwz=x_est_EKF(13);
            bax=x_est_EKF(14);bay=x_est_EKF(15);baz=x_est_EKF(16);
            q=[q0 q1 q2 q3]';
            q = quatnormalize(q')';
            Rq=I34*skewql(q)*skewqr(q)'*I34';
            
            
            %Observer Correction
            
            % If we are using DGPS correction and the current k (IMU index) is equal
            % to the next DGPS time index, correct the estimate using DGPS and the
            % Magnetometer.
            if ((DGPS_ON == 1) && (DGPS_Count > length(DGPS_Time_Index)))
                DGPS_Data_Available = 0;
            end
            
            if DGPS_Data_Available == 1
                if((DGPS_ON == 1) && (k == DGPS_Time_Index(DGPS_Count)))
                    
                    H_EKF = [ 1, 0, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 1, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 1, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mey*q1 - 2*mex*q2 - 2*mez*q0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mey*q0 - 2*mex*q3 + 2*mez*q1, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 2*mez*q2 - 2*mey*q3 - 2*mex*q0, 0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0, 2*mex*q2 - 2*mey*q1 + 2*mez*q0, 2*mex*q3 - 2*mey*q0 - 2*mez*q1, 2*mex*q0 + 2*mey*q3 - 2*mez*q2, 2*mex*q1 + 2*mey*q2 + 2*mez*q3, 0, 0, 0, 0, 0, 0];
                    
                    %Update DGPS Measurement
                    pm = [gpsx(DGPS_Count) gpsy(DGPS_Count) gpsz(DGPS_Count)]';
                    
                    % Normalize the next Magnetometer reading.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    
                    % Update the Magnetometer reading.
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    y_EKF=[pm' mg']';
                    
                    % Compute Normalization factor for the magnetometer estimate.
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    % Generate measurement estimate
                    y_est_EKF = [px py pz Rq(:,1)'*me/est_norm Rq(:,2)'*me/est_norm Rq(:,3)'*me/est_norm]';
                    
                    % Calculate Kalman gain
                    % R1 includes DGPS Noise figures
                    % Covariance matrix update
                    P_EKF = (I+F_EKF*dt)*P_EKF*(I+F_EKF*dt)' + G_EKF*Q_EKF*G_EKF'*dt^2;
                    
                    S_EKF = H_EKF*P_EKF*H_EKF' + R1_EKF;
                    K_EKF = P_EKF*H_EKF'*(S_EKF^-1);
                    % Correct estimate
                    x_est_EKF = x_est_EKF + K_EKF*(y_EKF-y_est_EKF);
                    % Correct covariance Matrix
                    P_EKF = P_EKF - K_EKF*H_EKF*P_EKF;
                    GPS_est_EKF(:,DGPS_Count) = x_est_EKF(1:3);
                    %         Y=[Y y];
                    
                    % Increment DGPS count to look at next DGPS reading.
                    DGPS_Count = DGPS_Count + 1;
                    GPS_P_est_EKF = [GPS_P_est_EKF sqrt(diag(P_EKF))];
                    
                else
                    
                    %Otherwise, if the next DGPS Measurement does not correspond to the
                    % current IMU reading, only use the Mag in the update.
                    MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                    mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                    
                    y_EKF=[mg']';
                    % Generate measurement estimate
                    est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                    
                    y_est_EKF = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                    
                    % Covariance matrix update
                    P_EKF = (I+F_EKF*dt)*P_EKF*(I+F_EKF*dt)' + G_EKF*Q_EKF*G_EKF'*dt^2;
                    
                    %         Calculate Kalman gain
                    % R2 Only includes Magnetometer noise
                    S_EKF = H_EKF*P_EKF*H_EKF' + R2_EKF;
                    K_EKF = P_EKF*H_EKF'*(S_EKF^-1);
                    % Correct estimate
                    x_est_EKF = x_est_EKF + K_EKF*(y_EKF-y_est_EKF);
                    % Correct covariance Matrix
                    P_EKF = P_EKF - K_EKF*H_EKF*P_EKF;
                    y_EKF = [ 0 0 0 y_EKF' ]';
                    Y_EKF = [Y_EKF y_EKF];
                    y_est_EKF = [ 0 0 0 y_est_EKF' ]';
                    Y_est_EKF = [Y_est_EKF y_est_EKF];
                end
            end
            if DGPS_Data_Available == 0
                %Otherwise, if the next DGPS Measurement does not correspond to the
                % current IMU reading, only use the Mag in the update.
                MagNorm = sqrt((Magnetometer(k,1))^2 + (Magnetometer(k,2))^2 +(Magnetometer(k,3))^2);
                mg = [Magnetometer(k,1)/MagNorm Magnetometer(k,2)/MagNorm Magnetometer(k,3)/MagNorm]';
                
                y_EKF=[mg']';
                % Generate measurement estimate
                est_norm = norm([Rqp(1,:)*me Rqp(2,:)*me Rqp(3,:)*me]');
                
                y_est_EKF = [Rqp(1,:)*me/est_norm Rqp(2,:)*me/est_norm Rqp(3,:)*me/est_norm]';
                
                % Covariance matrix update
                P_EKF = (I+F_EKF*dt)*P_EKF*(I+F_EKF*dt)' + G_EKF*Q_EKF*G_EKF'*dt^2;
                
                %         Calculate Kalman gain
                % R2 Only includes Magnetometer noise
                S_EKF = H_EKF*P_EKF*H_EKF' + R2_EKF;
                K_EKF = P_EKF*H_EKF'*(S_EKF^-1);
                % Correct estimate
                x_est_EKF = x_est_EKF + K_EKF*(y_EKF-y_est_EKF);
                % Correct covariance Matrix
                P_EKF = P_EKF - K_EKF*H_EKF*P_EKF;
                y_EKF = [ 0 0 0 y_EKF' ]';
                Y_EKF = [Y_EKF y_EKF];
                y_est_EKF = [ 0 0 0 y_est_EKF' ]';
                Y_est_EKF = [Y_est_EKF y_est_EKF];
            end
            
            x_est_EKF(7:10)=quatnormalize(x_est_EKF(7:10)')';
            
            if k == ODOM_Time_Index(ODOM_Count)
                
                [yaw_temp_EKF pitch_temp_EKF roll_temp_EKF]=quat2angle(x_est_EKF(7:10,:)');
                Yaw_est_EKF(ODOM_Count) = yaw_temp_EKF;
                ODOM_Count = ODOM_Count + 1;
            end

            X_est_EKF=[X_est_EKF x_est_EKF];
            P_est_EKF=[P_est_EKF sqrt(diag(P_EKF))];
            
        end

        %
        if Graphing == 1
            X = X_est_EKF;
            toc
            t = linspace(0,k*dt,k);
            
            figure(20)
            plot(X_est_EKF(1,:),X_est_EKF(2,:),'-g','LineWidth',2,'MarkerSize',2)
            title('Predicted vs actual Positions EKF')
            ylabel('Position [m]');
            xlabel('Time [s]');
            hold on
            plot(X_estC(1,:),X_estC(2,:),'-r','LineWidth',2,'MarkerSize',2)
            hold on
            plot(gpsx_true,gpsy_true,'-b','LineWidth',2,'MarkerSize',2)
            hold on
            plot(gpsx,gpsy,'-k','LineWidth',0.5,'MarkerSize',0.5)
            legend('EKF','IMM','DGPS')
            
            figure(21)
            plot(X_est_EKF(4,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            plot(X_est_EKF(5,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            plot(X_est_EKF(6,:),'-','LineWidth',2,'MarkerSize',2)
            title('Velocities EKF')
            ylabel('Velocity [m/s]');
            xlabel('Time [s]');
            
            
            figure(22)
            plot(Y_est_EKF(4,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y_EKF(4,:),'-','LineWidth',2,'MarkerSize',2)
            title('Predicted vs actual Magnetometer Measurements EKF')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated X Flux','Actual X Flux')
            
            figure(23)
            plot(Y_est_EKF(5,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y_EKF(5,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            title('Predicted vs actual Magnetometer Measurements EKF')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated Y Flux','Actual Y Flux')
            
            figure(24)
            plot(Y_est_EKF(6,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            plot(Y_EKF(6,:),'-','LineWidth',2,'MarkerSize',2)
            hold on
            title('Predicted vs actual Magnetometer Measurements EKF')
            ylabel('Magnetic Flux microTeslas');
            xlabel('Time [s]');
            legend('Estimated Z Flux','Actual Z Flux')
            
            % Result visualization
            figure(25)
            plot((X_est_EKF(4:6,:))','-','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Velocities EKF')
            ylabel('Velocity [m/s]');
            xlabel('Time [s]');
            legend('Estimated X Velocity','Estimated Y Velocity','Estimated Z Velocity')
            
            figure(26)
            
            plot((X_est_EKF(7:10,:))','-','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Quaternions EKF')
            ylabel('Quaternion Value');
            xlabel('Time [s]');
            legend('Estimated q0','Estimated q1','Estimated q2','Estimated q3')
            
            figure(27)
            plot((X_est_EKF(11:13,:))','-','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Gyro Bias EKF')
            ylabel('Gyro Bias [radians/s]');
            xlabel('Time [s]');
            legend('Estimated X Bias','Estimated Y Bias','Estimated Z Bias')
            
            figure(28)
            plot((X_est_EKF(14:16,:))','-','LineWidth',2,'MarkerSize',2)
            grid on
            title('Predicted vs actual Accelerometer Bias EKF')
            ylabel('Accelerometer Bias [m/s^2]');
            xlabel('Time [s]');
            legend('Estimated X Bias','Estimated Y Bias','Estimated Z Bias')
            %%
            figure(29)
            semilogy(abs(([gpsx_true(1:end-1); gpsy_true(1:end-1); gpsz_true(1:end-1)]'-GPS_est_EKF').^2),'-','LineWidth',2,'MarkerSize',2)
            hold on
            semilogy(2*GPS_P_est_EKF(1:3,:)','-','LineWidth',2,'MarkerSize',2)
            grid on
            title('Position error EKF')
            ylabel('Position Error [m]');
            xlabel('Time [s]');
            legend('X Error','Y Error','Z Error','95% X Error Confidence Bound',...
                '95% Y Error Confidence Bound','95% Z Error Confidence Bound')

        end
        if tuning == 0 || tuning == 1
            if cropping == 0
                RMSEx_EKF = (sqrt(mean((GPS_est_EKF(1,:) - gpsx_true(1,1:end-1)).^2)));
                RMSEy_EKF = (sqrt(mean((GPS_est_EKF(2,:) - gpsy_true(1,1:end-1)).^2)));
                RMSEz_EKF = (sqrt(mean((GPS_est_EKF(3,:) - gpsz_true(1,1:end-1)).^2)));
            else
                'EKF Cropped'
                RMSEx_EKF = (sqrt(mean((GPS_est_EKF(1,1:crop_num) - gpsx_true(1,1:crop_num)).^2)));
                RMSEy_EKF = (sqrt(mean((GPS_est_EKF(2,1:crop_num) - gpsy_true(1,1:crop_num)).^2)));
                RMSEz_EKF = (sqrt(mean((GPS_est_EKF(3,1:crop_num) - gpsz_true(1,1:crop_num)).^2)));
            end
            RMSE_EKF = sqrt(RMSEx_EKF^2 + RMSEy_EKF^2 + RMSEz_EKF^2)
            if Bad_GPS == 1
                BAD_GPS_RMSE_EKF = sqrt((mean(sqrt(mean((GPS_est_EKF(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsx_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2+(mean(sqrt(mean((GPS_est_EKF(2,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsy_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2+(mean(sqrt(mean((GPS_est_EKF(3,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsz_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2)
            end
        end
        %
        %         if tuning == 1 || tuning == 2
        %             if RMSE_EKF < Current_Min_Error
        %                 Current_Min_Error = RMSE_EKF;
        %                 noise_index = ij;
        %             end
        %         end
        
        if ((tuning == 0 || tuning == 2) && (Bad_GPS == 1))
            BAD_GPS_RMSE_IMM = sqrt((mean(sqrt(mean((GPS_estC(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsx_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2+(mean(sqrt(mean((GPS_estC(2,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsy_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2+(mean(sqrt(mean((GPS_estC(3,GPS_LOWER_RANGE:GPS_UPPER_RANGE) - gpsz_true(1,GPS_LOWER_RANGE:GPS_UPPER_RANGE)).^2))))^2)
        end
        
        if tuning == 0
            X_ERROR_DIFFERENCE = RMSE_IMMx-RMSEx_EKF
            Y_ERROR_DIFFERENCE = RMSE_IMMy-RMSEy_EKF
            Z_ERROR_DIFFERENCE = RMSE_IMMz-RMSEz_EKF
            RMSE_DIFFERENCE_TOTAL = RMSE_IMM-RMSE_EKF
        end
    end
    
    if tuning == 1 || tuning == 2
        if RMSE_IMM < Current_Min_Error
            Current_Min_Error = RMSE_IMM;
            noise_index = ij;
        end
    end
        
end
% noise_index
% noise_test(noise_index)

