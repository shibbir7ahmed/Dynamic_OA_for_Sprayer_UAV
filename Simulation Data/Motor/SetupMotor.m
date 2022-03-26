% J = 0.01; 
% b = 0.1;
% K = 0.01;
% R = 1;
% L = 0.5;

% The value is set by the motor in: Brushless DC Motor Model paper
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

%s = tf('s');
%P_motor = K/((J*s+b)*(L*s+R)+K^2);
%step(P_motor,10)