%% DC Motor Setup

J = 0.00062; 
b = 0.00030;
K = 0.074;
R = 0.0062;
L = 6.8e-5;

MaxMotorThrsut = 13.5*9.8;
MaxVolt = 48;
MaxRPM = 4350;

% Thrust = k_T * RPM^2: thrust and rpm relation.
k_T = MaxMotorThrsut/(MaxRPM^2);
k_Thrsut2Torque = 0.021;

%% Mass and Size
%m = 0.2
%I = [[0.1,0,0];[0,0.1,0];[0,0,0.08]]

m_drone = 20;
l = 0.5; % arm length of drone.

b = 0.4;
w = 0.4;
h = 0.1;
m_tank = 16;  % max liquid kg for water.

I_Drone = [0.3933,0.3933,0.76];
D_d2t = 0.2; %[0.2 0.4 0.5 0.8]

%% Water Level and Inertia
percent = 1; % percent of liquid height. [0~1]
m = m_drone + m_tank*percent;

I_Tank = CalItank(m_tank,b,h,w,percent);
CMz = CalCMz(m_drone,D_d2t,h,m_tank,percent);
CMz_drone = 0-CMz;
CMz_tank = D_d2t + CMz;
%%
figure
for i = 0:0.1:1
    I_Tank = CalItank(m_tank,0.4,0.1,0.4,i);
    CMz = CalCMz(m_drone,D_d2t,0.1,m_tank,i);
    CMz_drone = 0-CMz;
    CMz_tank = D_d2t + CMz;
    Ix = I_Drone(1) + m_drone*CMz_drone^2 + I_Tank(1) + (m_tank*i)*CMz_tank^2;
    scatter(i,Ix);
    hold on;
    scatter(i,CMz);
    hold on;
end
hold off;


%% Sample Time & Initial States
ts = 0.01;

% Initial States (Initial XYZ is generated by XYZsignal script)
Euler_0 = [0;0;0];
XYZ_0 = [0;0;0];
body_rate_0 = [0;0;0];

% Environment
g = [0;0;-9.8];

%% Way Points
WayPts = [[XYZ_0(1) XYZ_0(2) XYZ_0(3)];
          [0 0 3];
          [12 0 3]];
Target = WayPts(3,:);

%% Drone Performance
MaxVel = 1.5;
MaxZVel = 2;
MaxYawRate = 100*pi/180;
MaxThrust = 4*MaxMotorThrsut;
MaxTilt = 30*pi/180;
MaxTorque_XY = MaxThrust*0.5;

SafeZone = 1.0;

%% Velocity and Rate PID Controller Gains

% Vx, Vy, controller Gains
Px1 = 0.2; % 0.2 Velocity error to Angle
Px2 = 6;   % 6 Angle error to Angle rate
Px3 = 10;  % 10 Rate error to Torque

% Z Controller gain
Pz1 = 1; %2.5: Zpos error to Velocity
Pz2 = 100; %10: ZVel error to Thrust

%% Define or Load Obstacle Points
RecObstacle = [[5,4,3];
               [3,-4,3];
               [5,-6,3];
               [7,2,3]];
%% Calculate Boundaries
B1 = calBound(RecObstacle(1,:),RecObstacle(2,:));
B2 = calBound(RecObstacle(2,:),RecObstacle(3,:));  
B3 = calBound(RecObstacle(3,:),RecObstacle(4,:));  
Boundaries = [B1;B2;B3];
B1_Yaw = calYaw([B1(3),B1(4),0]);
B2_Yaw = calYaw([B2(3),B2(4),0]);
B3_Yaw = calYaw([B3(3),B3(4),0]);
RecObstacleYaw = [B1_Yaw,B2_Yaw,B3_Yaw];
clear B1;
clear B2;
clear B3;
clear B1_Yaw;
clear B2_Yaw;
clear B3_Yaw;


%% Local Function
function Ifulltank = CalItank(m,b,h,w,percent)
    realh = h*percent;
    realm = m*percent;
    Ix = realm*(realh^2+b^2)/12;
    Iy = Ix;
    Iz = realm*(b^2+w^2)/12;
    Ifulltank = [Ix,Iy,Iz];
end

function CMz = CalCMz(md,D_d2t,h,m,percent)
    l1 = D_d2t-0.5*h;
    Z = l1 + (1-percent)*h + 0.5*percent*h;
    mwater = m*percent;
    CMz = (0 -Z*mwater)/(md+mwater);
end
    

function MaxTorque_XY = CalMaxTorqueXY(m,MaxThrust,MaxTilt,l)
    W = m*9.8;
    T2 = MaxThrust/2;
    T1 = (W - T2*cos(MaxTilt) )/cos(MaxTilt);
    MaxTorque_XY = (T2-T1)*(l/sqrt(2));
end
    
function B = calBound(p1,p2)
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    tx = p2(1)-p1(1);
    ty = p2(2)-p1(2);
    t = [tx,ty,0]/sqrt(tx^2+ty^2);
    n = cross(t,[0,0,1]);
    L = sqrt((x2-x1)^2 + (y2-y1)^2);
    B = [x1,y1,t(1),t(2),n(1),n(2),L];
end

function yaw = calYaw(t)
    u = [1,0,0];
    yaw = acos(dot(u/norm(u),t/norm(t)));
    if t(2) < 0
        yaw = -1*yaw;
    end
end