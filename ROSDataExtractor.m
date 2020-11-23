%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file reads Topics from a ROS bag file and stores the data in Matlab
% variable matrices for later use in Inertial Navigation System (INS) or
% robotics applications. This process may take several minutes, depending on
% the size of the .bag file
%
% Author: Patrick Glavine, MEng., Memorial University of Newfoundland Email
% address: pjglavine23@gmail.com Date: Aug. 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

% Create a ROS bag object in Matlab using the input data gathered from ROS.
filename = 'SeekurDataOct4.bag'; 
bag = rosbag(filename);

%%
% This section selects the desired bag Topics from the bag object. Note,
% certain topics have been commented out, this is because these particular
% topics were not used for the INS in the Seekur Jr Experiment at the time.

% bagInfo = rosbag('info',filename)
%bSel1 = select(bag,'Topic','/pioneer1/gps'); % Seekur Jr built-in Trimble GPS
bSel2 = select(bag,'Topic','/pioneer1/odom'); % Seekur Jr onboard odometry
%bSel3 = select(bag,'Topic','/pioneer1/scan'); % Seekur Jr unfiltered laser
%bSel4 = select(bag,'Topic','/pioneer1/scan_filtered'); % Filtered laser scans 
bSel5 = select(bag,'Topic','/pioneer1/wheelEncoder'); % Seekur Jr wheel encoder data packets, obtained from Seekur Jr system information packets (SIP)
%bSel6 = select(bag,'Topic','/zr300_node/pointcloud'); % Intel ZR300 sensor pointcloud data
%bSel7 = select(bag,'Topic','/zr300_node/imu'); % Intel ZR300 sensor IMU data
bSel7 = select(bag,'Topic','/imu/data_raw'); % If using Reach IMU Data
bSel8 = select(bag,'Topic','/imu/MagneticField'); % Reach Magnetometer data
bSel9 = select(bag,'Topic','/ReachGPS'); % Reach GPS data


% Initialize structures as empty in case the bag does not contain these
% data types.
%%
GPSstructs = 0;
ODOMstructs = 0;
SCANstructs = 0;
SCANFstructs = 0;
ENCstructs = 0;
PCLOUDstructs = 0;
IMUstructs = 0;
MAGstructs = 0;
DGPSstructs = 0;

% Read data messages if they are available.
%GPSstructs = readMessages(bSel1);
ODOMstructs = readMessages(bSel2);
%SCANstructs = readMessages(bSel3);
%SCANFstructs = readMessages(bSel4);
ENCstructs = readMessages(bSel5);
%PCLOUDstructs = readMessages(bSel6);
IMUstructs = readMessages(bSel7);
MAGstructs = readMessages(bSel8);
DGPSstructs = readMessages(bSel9);

%%
% Store filtered and unfiltered laser scan data in structure data types.
LaserMsgs = SCANstructs;
FilteredLaserMsgs = SCANFstructs;
%%

% Check if GPS data is available. Store data in usable variables.
i = length(GPSstructs);

if i > 1
    gps = zeros(3,i);
    GPS_Time_Sec = zeros(1,i);
    GPS_Time_NSeconds = zeros(1,i);
    for j = 1:i
        gps(1,j) = GPSstructs{j,1}.Latitude;
        gps(2,j) = GPSstructs{j,1}.Longitude;
        gps(3,j) = GPSstructs{j,1}.Altitude;
        GPS_Time_Sec(1,j) = GPSstructs{j,1}.Header.Stamp.Sec;
        GPS_Time_NSeconds(1,j) = GPSstructs{j,1}.Header.Stamp.Nsec;
    end
    
    % Convert Second and Nanosecond timestamps into unified data structure.
    GPS_Time = zeros(1,length(GPS_Time_Sec));
    for j = 1:i
        A = GPS_Time_Sec(1,j);
        B = GPS_Time_NSeconds(1,j);
        str = sprintf('%d.%d',A,B);
        GPS_Time(1,j) = str2double(str);
    end
    gps = gps';
    
    % Sort the data to ensure that time stamps are always increasing
    % between adjacent data points.
    [Ordered_GPS_Time, Indices] = sort(GPS_Time,'ascend');
    Temp_GPS = zeros(i,3);
    GPS_Time = Ordered_GPS_Time;
    
    % Reorder the gps data to match the rearranged time stamps
    for j = 1:i
        Temp_GPS(j,:) = gps(Indices(j),:);
    end
    gps = Temp_GPS;
end

% Wheel Encoder Data
i = length(ENCstructs);

if i > 1
    enc = zeros(5,i);
    ENC_Time_Sec = zeros(1,i);
    ENC_Time_NSeconds = zeros(1,i);
    
    for j = 1:i
        
        % Obtain SIP packet data and scale appropriately.
        enc(1,j) = ENCstructs{j,1}.X*0.001; % X and Y in Meters
        enc(2,j) = ENCstructs{j,1}.Y*0.001;
        enc(3,j) = ENCstructs{j,1}.Th*0.001534; % Theta in Radians
        enc(4,j) = ENCstructs{j,1}.VelLeft*0.001; % Left and Right Wheel Velocities in m/s
        enc(5,j) = ENCstructs{j,1}.VelRight*0.001;
        ENC_Time_Sec(1,j) = ENCstructs{j,1}.Header.Stamp.Sec;
        ENC_Time_NSeconds(1,j) = ENCstructs{j,1}.Header.Stamp.Nsec;
    end
    
    % Append ROS time stamps. Nanosecond time is appended to Second time
    % stamp after decimal point.
    ENC_Time = zeros(1,length(ENC_Time_Sec));
    for j = 1:i
        A = ENC_Time_Sec(1,j);
        B = ENC_Time_NSeconds(1,j);
        str = sprintf('%d.%d',A,B);
        ENC_Time(1,j) = str2double(str);
    end
    enc = enc';
    
    % Sort the SIP Packet data such that the time stamps increase between
    % consecutive data points. Output is sorted time matrix with indices
    % corresponding to the original indice references from the unsorted
    % time matrices
    [Ordered_ENC_Time, Indices] = sort(ENC_Time,'ascend');
    Temp_ENC = zeros(i,5);
    ENC_Time = Ordered_ENC_Time;
    % Reorder the SIP Packet data such that the data values are
    % corresponding to the sorted time values.
    for j = 1:i
        Temp_ENC(j,:) = enc(Indices(j),:);
    end
    enc = Temp_ENC;
end

% Seekur onboard Odometry Data
i = length(ODOMstructs);

if i > 1
    odom = zeros(3,i);
    odom_quat = zeros(4,i);
    odom_euler = zeros(3,i);
    odom_linear_velocity = zeros(3,i);
    odom_angular_velocity = zeros(3,i);
    ODOM_Time_Sec = zeros(1,i);
    ODOM_Time_NSeconds = zeros(1,i);
    for j = 1:i
        
        % Odometry X Y Z Pose estimates in the odom frame
        odom(1,j) = ODOMstructs{j,1}.Pose.Pose.Position.X;
        odom(2,j) = ODOMstructs{j,1}.Pose.Pose.Position.Y;
        odom(3,j) = ODOMstructs{j,1}.Pose.Pose.Position.Z;
        
        % Robot orientation expressed as a quaternion.
        odom_quat(1,j) = ODOMstructs{j,1}.Pose.Pose.Orientation.X;
        odom_quat(2,j) = ODOMstructs{j,1}.Pose.Pose.Orientation.Y;
        odom_quat(3,j) = ODOMstructs{j,1}.Pose.Pose.Orientation.Z;
        odom_quat(4,j) = ODOMstructs{j,1}.Pose.Pose.Orientation.W;
        
        % Convert Quaternion orientations into euler angles.
        quaternion = [odom_quat(1,j) odom_quat(2,j) odom_quat(3,j) odom_quat(4,j)];
        [odom_euler(1,j), odom_euler(2,j), odom_euler(3,j)] = quat2angle(quaternion);
        
        % Obtain linear and angular velocities recorded for each odometry
        % reading.
        odom_linear_velocity(1,j) = ODOMstructs{j,1}.Twist.Twist.Linear.X;
        odom_linear_velocity(2,j) = ODOMstructs{j,1}.Twist.Twist.Linear.Y;
        odom_linear_velocity(3,j) = ODOMstructs{j,1}.Twist.Twist.Linear.Z;
        odom_angular_velocity(1,j) = ODOMstructs{j,1}.Twist.Twist.Angular.X;
        odom_angular_velocity(2,j) = ODOMstructs{j,1}.Twist.Twist.Angular.Y;
        odom_angular_velocity(3,j) = ODOMstructs{j,1}.Twist.Twist.Angular.Z;
        ODOM_Time_Sec(1,j) = ODOMstructs{j,1}.Header.Stamp.Sec;
        ODOM_Time_NSeconds(1,j) = ODOMstructs{j,1}.Header.Stamp.Nsec;
    end
    
    % Append ROS time stamps. Nanosecond time is appended to Second time
    % stamp after decimal point.
    ODOM_Time = zeros(1,length(ODOM_Time_Sec));
    for j = 1:i
        A = ODOM_Time_Sec(1,j);
        B = ODOM_Time_NSeconds(1,j);
        str = sprintf('%d.%d',A,B);
        ODOM_Time(1,j) = str2double(str);
    end
    
    % Sort the Odometry data such that the time stamps increase between
    % consecutive data points. Output is sorted time matrix with indices
    % corresponding to the original indice references from the unsorted
    % time matrices
    [Ordered_ODOM_Time, Indices] = sort(ODOM_Time,'ascend');
    Temp_ODOM1 = zeros(i,3);
    Temp_ODOM2 = zeros(i,4);
    Temp_ODOM3 = zeros(i,3);
    ODOM_Time = Ordered_ODOM_Time;
    odom = odom';
    odom_quat = odom_quat';
    odom_euler = odom_euler';
    
    % Reorder the Odometry data such that the data values are
    % corresponding to the sorted time values.
    for j = 1:i
        Temp_ODOM1(j,:) = odom(Indices(j),:);
        Temp_ODOM2(j,:) = odom_quat(Indices(j),:);
        Temp_ODOM3(j,:) = odom_euler(Indices(j),:);
    end
    odom = Temp_ODOM1;
    odom_quat = Temp_ODOM2;
    odom_euler = Temp_ODOM3;
    
end

% Reach IMU Data
i = length(IMUstructs);

if i > 1
    Accelerometer = zeros(3,i);
    Gyroscope = zeros(3,i);
    IMU_Time_Sec = zeros(1,i);
    IMU_Time_NSeconds = zeros(1,i);
    
    
    for j = 1:i
        
        % Reach Accelerometer X Y Z Data
        Accelerometer(1,j) = IMUstructs{j,1}.LinearAcceleration.X;
        Accelerometer(2,j) = IMUstructs{j,1}.LinearAcceleration.Y;
        Accelerometer(3,j) = IMUstructs{j,1}.LinearAcceleration.Z;
        
        % Reach Gyroscope X Y Z Data
        Gyroscope(1,j) = IMUstructs{j,1}.AngularVelocity.X;
        Gyroscope(2,j) = IMUstructs{j,1}.AngularVelocity.Y;
        Gyroscope(3,j) = IMUstructs{j,1}.AngularVelocity.Z;
        IMU_Time_Sec(1,j) = IMUstructs{j,1}.Header.Stamp.Sec;
        IMU_Time_NSeconds(1,j) = IMUstructs{j,1}.Header.Stamp.Nsec;
    end
    
    Gyroscope = Gyroscope';
    Accelerometer = Accelerometer';
    
    % Append ROS time stamps. Nanosecond time is appended to Second time
    % stamp after decimal point.
    i = length(IMU_Time_Sec);
    IMU_Time = zeros(1,length(IMU_Time_Sec));
    for j = 1:i
        A = IMU_Time_Sec(1,j);
        B = IMU_Time_NSeconds(1,j);
        str = sprintf('%d.%d',A,B);
        IMU_Time(1,j) = str2double(str);
    end
    
    % Sort the IMU data such that the time stamps increase between
    % consecutive data points. Output is sorted time matrix with indices
    % corresponding to the original indice references from the unsorted
    % time matrices
    [Ordered_IMU_Time, Indices] = sort(IMU_Time,'ascend');
    Temp_Gyro = zeros(length(Gyroscope),3);
    Temp_Acc = zeros(length(Gyroscope),3);
    IMU_Time = Ordered_IMU_Time;
    
    % Reorder the Odometry data such that the data values are
    % corresponding to the sorted time values.
    for j = 1:i
        Temp_Gyro(j,:) = Gyroscope(Indices(j),:);
        Temp_Acc(j,:) = Accelerometer(Indices(j),:);
    end
    Gyroscope = Temp_Gyro;
    Accelerometer = Temp_Acc;
    
end

%%
% Reach Magnetometer Data
i = length(MAGstructs);

if i > 1
   % i = length(Accelerometer);
    Magnetometer = zeros(3,i);
    MAG_Time_Sec = zeros(1,i);
    MAG_Time_NSeconds = zeros(1,i);
    for j = 1:i
        
        % Reach Magnetometer X Y Z Data
        Magnetometer(1,j) = MAGstructs{j,1}.MagneticField_.X;
        Magnetometer(2,j) = MAGstructs{j,1}.MagneticField_.Y;
        Magnetometer(3,j) = MAGstructs{j,1}.MagneticField_.Z;
        MAG_Time_Sec(1,j) = MAGstructs{j,1}.Header.Stamp.Sec;
        MAG_Time_NSeconds(1,j) = MAGstructs{j,1}.Header.Stamp.Nsec;
    end
    
    Magnetometer = Magnetometer';
    
    %%
    
    % Sometimes Reach outputs different number of Mag data than IMU
    % data, set the matrix size such that the dimensions match.
    % Magnetometer = Magnetometer(1:length(Gyroscope)-1,:);
    
    % Calibrate Magnetometer Data Using Max/Min Centering and Axis Scaling
    
    MAG = Magnetometer';
    
    
    % Calculate the Bias based on Minimum and Maximum Magnetometer values
    % in the X Y and Z directions
    min_magx = min(MAG(1,:));
    min_magy = min(MAG(2,:));
    min_magz = min(MAG(3,:));
    
    max_magx = max(MAG(1,:));
    max_magy = max(MAG(2,:));
    max_magz = max(MAG(3,:));
    
    x_bias = (max_magx+min_magx)/2;
    y_bias = (max_magy+min_magy)/2;
    z_bias = (max_magz+min_magz)/2;
    
    
    % Correct the Magnetometer readings using these biases.
    Mag_correctedx = MAG(1,:)-x_bias;
    Mag_correctedy = MAG(2,:)-y_bias;
    Mag_correctedz = MAG(3,:)-z_bias;
    
%     mean_Mag_correctedx = mean(Mag_correctedx);
%     mean_Mag_correctedy = mean(Mag_correctedy);
%     mean_Mag_correctedz = mean(Mag_correctedz);
    
    % Calculate scale factors for X Y and Z directions
    mag_scalex = (max_magx-min_magx)/2
    mag_scaley = (max_magy-min_magy)/2
    mag_scalez = (max_magz-min_magz)/2
    
    % Compute an average scale for the X Y and Z directions.
    avg_rad = mag_scalex + mag_scaley + mag_scalez;
    avg_rad = avg_rad / 3
    
    % Compute X Y and Z scales using the average
    mag_scalex = avg_rad/mag_scalex
    mag_scaley = avg_rad/mag_scaley
    mag_scalez = avg_rad/mag_scalez
    
    % Multiply the previously corrected Magnetometer data by the
    % appropriate scale factor for each direction.
    Mag_corrected2x = mag_scalex*(Mag_correctedx);
    Mag_corrected2y = mag_scaley*(Mag_correctedy);
    Mag_corrected2z = mag_scalez*(Mag_correctedz);

    Magnetometer = [Mag_corrected2x' Mag_corrected2y' Mag_corrected2z'];
    
    % Append ROS time stamps. Nanosecond time is appended to Second time
    % stamp after decimal point.
    i = length(MAG_Time_Sec);
    MAG_Time = zeros(1,length(MAG_Time_Sec));
    for j = 1:i
        A = MAG_Time_Sec(1,j);
        B = MAG_Time_NSeconds(1,j);
        str = sprintf('%d.%d',A,B);
        MAG_Time(1,j) = str2double(str);
    end
%     
%     % Sort the Magnetometer data such that the time stamps increase between
%     % consecutive data points. Output is sorted time matrix with indices
%     % corresponding to the original indice references from the unsorted
%     % time matrices  

    [Ordered_MAG_Time, Indices] = sort(MAG_Time,'ascend');
    
    Temp_Mag = zeros(i,3);
    for j = 1:i
        Temp_Mag(j,:) = Magnetometer(Indices(j),:);
    end
    Magnetometer = Temp_Mag;
    
    % Ensure that the Magnetometer, Gyroscope and Accelerometer data all
    % have the same time stamps and number of data elements. This just
    % duplicates the final Magnetometer reading and adds 0.005 seconds to
    % the MAG time matrix, with such a small dt between readings, this
    % should not cause any problems.
%     Magnetometer = [Magnetometer; Magnetometer(i,:)];
%     MAG_Time = [MAG_Time MAG_Time(i)+0.005];
end

%%
% Reach Differential GPS Data
i = length(DGPSstructs);

if i > 1
    DGPS = zeros (13, i);
    Latitude = zeros(i,1);
    Longitude = zeros(i,1);
    Altitude = zeros(i,1);
    Quality = zeros(i,1);
    NSats = zeros(i,1);
    sdn = zeros(i,1);
    sde = zeros(i,1);
    sdu = zeros(i,1);
    sdne = zeros(i,1);
    sdeu = zeros(i,1);
    sdun = zeros(i,1);
    AoD = zeros(i,1);
    ARratio = zeros(i,1);
    format long;
    DGPS_Time = zeros(i,1);
    
    % Convert data from text string to numerical array.
    for j = 1:i
        DGPS_Temp = str2num(DGPSstructs{j,1}.Data);
        count = 1;
        DGPS_Time(j)=DGPS_Temp(1);
        for k = 1:length(DGPS_Temp)
            
            % Issues with char date and time data near the beginning
            % of the string messages was causing inconsistencies. This
            % check looks for a Latitude value and starts storing data at
            % that point until the end of the string. In this case, the
            % Latitude was always 47.something. This number should be
            % changed to match the first two digits of the local Latitude
            % in the dataset. This is a rough, temporary solution for this
            % issue.
            if floor(DGPS_Temp(k)) == 47
                break
            end
            count = count + 1;
        end
        Latitude(j) = DGPS_Temp(count);
        Longitude(j) = DGPS_Temp(count+1);
        Altitude(j) = DGPS_Temp(count+2);
        Quality(j) = DGPS_Temp(count+3);
        NSats(j) = DGPS_Temp(count+4);
        sdn(j) = DGPS_Temp(count+5);
        sde(j) = DGPS_Temp(count+6);
        sdu(j) = DGPS_Temp(count+7);
        sdne(j) = DGPS_Temp(count+8);
        sdeu(j) = DGPS_Temp(count+9);
        sdun(j) = DGPS_Temp(count+10);
        AoD(j) = DGPS_Temp(count+11);
        ARratio(j) = DGPS_Temp(count+12);
    end
    DGPS(1,:) = Latitude(:)';
    DGPS(2,:) = Longitude(:);
    DGPS(3,:) = Altitude(:);
    DGPS(4,:) = Quality(:);
    DGPS(5,:) = NSats(:);
    DGPS(6,:) = sdn(:);
    DGPS(7,:) = sde(:);
    DGPS(8,:) = sdu(:);
    DGPS(9,:) = sdne(:);
    DGPS(10,:) = sdeu(:);
    DGPS(11,:) = sdun(:);
    DGPS(12,:) = AoD(:);
    DGPS(13,:) = ARratio(:);
end

% Offset Log time of all sensors with respect to the IMU, ie: start time at
% 0 rather than an arbitrary ROS Time
%%
Time = IMU_Time;
Time = Time - IMU_Time(1,1);
ENC_Time = ENC_Time - IMU_Time(1,1);
GPS_Time = GPS_Time - IMU_Time(1,1);
%%
MAG_Time = MAG_Time - IMU_Time(1,1);
ODOM_Time = ODOM_Time - IMU_Time(1,1);
DGPS_Time = DGPS_Time'*1e-9 - IMU_Time(1,1);
IMU_Time = Time;

% Once this script has finished running, hold CTRL and left click all the
% variables in the workspace that you would like to save in a .mat file.
% Right click, and click Save as... to save the extracted data. This
% approach was implemented to avoid accidentally overwriting saved .mat
% datasets that may share a same name.


