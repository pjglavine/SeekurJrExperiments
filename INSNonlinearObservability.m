%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nonlinear system observability analysis for the Seekur Jr inertial
% navigation system (INS) using position feedback.
%
% Authors: Patrick Glavine and Oscar De Silva
% Email address: pjglavine23@gmail.com
% Date: Aug. 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all
syms px py pz real                  %position vectors
syms vx vy vz real                  %velocity vectors
syms q0 q1 q2 q3 real               %quaternions
syms wmx wmy wmz real               %body gyro readings
syms bwx bwy bwz real               %gyro biases
syms vwmx vwmy vwmz real            %gyro white noise
syms vbwx vbwy vbwz real            %gyro bias noise
syms fmx fmy fmz real               %accelerometer measurements
syms bax bay baz real               %accelerometer biases
syms vfmx vfmy vfmz real            %accelerometer noise
syms vbax vbay vbaz real            %gyro bias noise
syms mgx mgy mgz real               %mag measurements
syms vmgx vmgy vmgz real            %magnetic measurement noise
syms pmx pmy pmz real               %GPS measurement
syms vpmx vpmy vpmz real            %GPS position measurement noise
syms gex gey gez mex mey mez real   %Reference vectors for gravity and magnetic north

% States
p=[px py pz]';                      %position
v=[vx vy vz]';                      %velocity
q=[q0 q1 q2 q3]';                   %quarternions
bw=[bwx bwy bwz]';                  %gyro bias
ba=[bax bay baz]';                  %accelerometer bias

%Measurements
wm = [wmx wmy wmz]';               %gyro measurement
fm = [fmx fmy fmz]';               %accelerometer measurement
mg = [mgx mgy mgz]';               %mag measurement
pm = [pmx pmy pmz]';              %position measurement

%Noises
vfm = [vfmx vfmy vfmz]';                  %Acc noise
vba = [vbax vbay vbaz]';              %Acc bias noise
vwm = [vwmx vwmy vwmz]';                   %Gyro noise
vbw = [vbwx vbwy vbwz]';               %Gyro bias noise
vpm = [vpmx vpmy vpmz]';             %GPS noise
vmg = [vmgx vmgy vmgz]';                 %mag noise

% Magnetometer and Gravity vectors
me=[mex mey mez]';                       %magnetic reference
ge=[gex gey gez]';                       %gravity reference

%state vectors
% x=[q' bw']';                   %states
w=[vwm' vbw' vfm' vba']';                %process noise
nu=[vpm' vmg']';               %measurement noise

x = [p' v' q' bw' ba']';
w = [vwm' vfm']';

n=size(x,1);
I34=[0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Qp = [-q1 -q2 -q3;
    q0 -q3 q2;
    q3 q0 -q1;
    -q2 q1 q0;];

%system model (with noise)
Reb=I34*skewql(q)*skewqr(q)'*I34';

dp = v;
dv = Reb*(fm-ba)+ge;
dq = 0.5*skewqr([0 (wm-bw)'])*q;
dbw = [vbwx vbwy vbwz]';
dba = [vbax vbay vbaz]';

xdot=[dp; dv; dq; dbw; dba];       %nonlinear set of equations

%measurement model

% y=[ -Reb'*ge + vfm;]

y = [p + vpm;
    (Reb')*me + vmg;];
%%
%linearize

%Fmatrix
%xdot2=subs(xdot,w,zeros(size(w)));  %noiseless system
y2=subs(y,nu,zeros(size(nu)));        %noiseless measurement
xdot2 = [xdot(1:10); [0 0 0 0 0 0]'];
%
for i=1:n
    for j=1:n
        F(i,j)=diff(xdot2(i),x(j));
    end
end
F;

%% Nonlinear Observability Analysis Affine Form

% Noiseless System without Inputs
fo = [v; -Reb*ba + ge; -0.5*Qp*bw; zeros(1,3)'; zeros(1,3)'];

f1 = [zeros(3,3)'; Reb; zeros(10,3)];  

f2 = [zeros(6,3); 0.5*Qp; zeros(6,3)];

%f_combined = fo+f1+f2;

% Zeroth Lie Derivative is just the function itself
% Gradient of the zeroth Lie Derivative

% Number of function components ( fo f1 f2 )
num_functions = 3;

y = [p ];
O=[]; %observability Matrix

L0H = y;
gradL0H = jacobian(L0H,x); %spans p, rank 3.
O=[O;gradL0H];
fprintf('Zeroth order Lie gradients. System Rank is \n')
disp(rank(O))
fprintf('Current null space \n')
disp(x'*null(O))

%First Tree Level
L1_fo_H=gradL0H*fo;
L1_f1_H=gradL0H*f1; %Is zero
L1_f2_H=gradL0H*f2; %Is zero

gradL1_fo_H = jacobian(gradL0H*fo,x); %spans v, increases rank by 3.
%gradL1_f1_H; %Is zero, doesnt matter
%gradL1_f2_H; %Is zero, doesnt matter
O=[O;gradL1_fo_H];
fprintf('First order Lie gradients. System Rank is \n')
disp(rank(O))
fprintf('Current null space \n')
disp(x'*null(O))

% Second Tree Level
L2_fofo_H=gradL1_fo_H*fo; %Is -Reb*ba + ge
L2_fof1_H=gradL1_fo_H*f1; %Is Rq
L2_fof2_H=gradL1_fo_H*f2; %Is zero, doesnt matter
%all other entries zero


gradL2_fof1_H = jacobian(L2_fof1_H(:),x); % when theres a matrix we stack the columns to get a stacked vector
%This spans q, increases rank by 4.
gradL2_fofo_H = jacobian(L2_fofo_H,x); % This spans ba because q is already resolved above increases rank by 3
O=[O;gradL2_fof1_H;gradL2_fofo_H];
fprintf('Second order Lie gradients. System Rank is \n')
disp(rank(O))
fprintf('Current null space \n')
disp(x'*null(O))

%The current null space: >>null(O)'*x results in bw only

% Third Tree Level
L3_fof1fo_H=gradL2_fof1_H*fo; 

gradL3_fof1fo_H = jacobian(L3_fof1fo_H,x);
O=[O;gradL3_fof1fo_H]; 
fprintf('Third order Lie gradients. System Rank is \n')
disp(rank(O))
fprintf('Current null space \n')
disp(x'*null(O))
