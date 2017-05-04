%% Introduction 

% Engineering Decisions
% 
% - Gear Ratio
% - Motor
% - Ball Material

% Performance Measures
% 
% - Control Effort
% - Steady State Error
% - Rise Time 
% - Sensitivity


%% Densities

%{
stainless_density = 7.97; % g/cm^3
aluminum_density = 2.7; % g/cm^3
%% Motors
m1 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .225, 'R', 8, 'L', .025);
m2 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .175, 'R', 6, 'L', .016);
m3 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .125, 'R', 4, 'L', .0075);
m4 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .275, 'R', 12, 'L', .032);
motors = [m1, m2, m3, m4];

clear m1 m2 m3 m4

%% Ball Properties
ball_radius = .01; % m
ball_volume = (4/3) * pi * (ball_radius*100)^3; % cm^3
ball_mass = aluminum_density * ball_volume / 1000; % kg
J_b = (2/5) * ball_mass * (ball_radius)^2; % kg*m^2

clear ball_volume

%% Gearbox & Other
J_g = 6.2e-6; % kg*m^2
J_s = 1.4e-7; % kg*m^2
G_v = 5; % V/V
g = 9.8; % m/s^2
K_s = 10; % V/m
N = 25; 

%% Bar Properties
bar_height = .8; % cm
bar_width = 2; % cm
bar_length = 50; % cm
bar_ch_depth = .6; % cm
bar_ch_width = 1.6; % cm
ch_volume = bar_ch_depth * bar_ch_width * bar_length;
bar_noch_volume = bar_height * bar_width * bar_length;
bar_volume = bar_noch_volume - ch_volume; % cm^3
bar_mass = aluminum_density * bar_volume / 1000; % kg
J_bar = (bar_mass * (bar_length / 100)^2) / 12; % kg*m^2

clear ch_volume aluminum_density bar_height bar_width bar_ch_depth
clear ch_volume bar_ch_width bar_noch_volume

%% Plant Transfer Function
motor = motors(2);
B_m = motor.B_m;
K_t = motor.K_t;
L = motor.L;
R = motor.R;
J_eff = motor.J_m + J_g + (J_bar + J_s) / N^2; % kg*m^2

s = tf('s');
  
num = G_v * ball_mass * g * K_s * K_t;

den = s^5 * N * (ball_mass + (J_b / ball_radius^2)) * ...
      (L * J_eff + s^-1*(B_m * L + R * J_eff) +  ...
      s^-2 * (K_t^2 + B_m * R));
  
G = num / den;
G = minreal(G)

%% Controller
G = zpk(G)

ts = .05;
os = 0;
n = 9;

[num,den] = stepshape(n,os,ts);
[D,T,Tu,Td,L]=lamdesign(G,den);
step(T)
%}
%%%%%%%%%% Version 2
% Talked with Hanselman: Bug in code prevents LAMDesign from working
%%%%%%%%%% Version 3
% pidTuner was not returning stable results
%%%%%%%%%% Version 4
% PID placing by hand
%% Densities
stainless_density = 7.97; % g/cm^3
aluminum_density = 2.7; % g/cm^3
%% Motors
m1 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .225, 'R', 8, 'L', .025);
m2 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .175, 'R', 6, 'L', .016);
m3 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .125, 'R', 4, 'L', .0075);
m4 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_t', .275, 'R', 12, 'L', .032);
motors = [m1, m2, m3, m4];

clear m1 m2 m3 m4

%% Ball Properties
ball_radius = .01; % m
ball_volume = (4/3) * pi * (ball_radius*100)^3; % cm^3
ball_mass = aluminum_density * ball_volume / 1000; % kg
J_b = (2/5) * ball_mass * (ball_radius)^2; % kg*m^2

clear ball_volume

%% Gearbox & Other
J_g = 6.2e-6; % kg*m^2
J_s = 1.4e-7; % kg*m^2
G_v = 5; % V/V
g = 9.8; % m/s^2
K_s = 10; % V/m
N = 25; 

%% Bar Properties
bar_height = .8; % cm
bar_width = 2; % cm
bar_length = 50; % cm
bar_ch_depth = .6; % cm
bar_ch_width = 1.6; % cm
ch_volume = bar_ch_depth * bar_ch_width * bar_length;
bar_noch_volume = bar_height * bar_width * bar_length;
bar_volume = bar_noch_volume - ch_volume; % cm^3
bar_mass = aluminum_density * bar_volume / 1000; % kg
J_bar = (bar_mass * (bar_length / 100)^2) / 12; % kg*m^2

clear ch_volume aluminum_density bar_height bar_width bar_ch_depth
clear ch_volume bar_ch_width bar_noch_volume

%% Plant Transfer Function
%{
motor = motors(1);
B_m = motor.B_m;
K_t = motor.K_t;
L = motor.L;
R = motor.R;
J_eff = motor.J_m + J_g + (J_bar + J_s) / N^2; % kg*m^2

s = tf('s');
  
num = G_v * ball_mass * g * K_s * K_t;

den = s^5 * N * (ball_mass + (J_b / ball_radius^2)) * ...
      (L * J_eff + s^-1*(B_m * L + R * J_eff) +  ...
      s^-2 * (K_t^2 + B_m * R));
  
G = num / den;
G = minreal(G)
%% Controller

G = zpk(G)

z1 = -.0001 + j*.0001;   % Pole, Zero, and K
z2 = -.0001 - j*.0001;
p1 = -1000;
p2 = -2000;
K = 10E9;


D = zpk([z1,z2,z3,z4,z5], [p1,p2,p3,p4,p5], 1);
figure(1)
rlocus(D*G); 

D = K*D;  % Multiply in K
T = feedback(D*G, 1);  % Get transfer function T
[Tu, umax] = controleffort(G, T);
S = stepinfo(T);

kd = K/p;   % convert from poles, zeros, and k to parameters
kp = z * kd;

D   % printing
fprintf('Kd:%.2f\nKp:%.2f\n',kd,kp);
figure(2)
step(T);
fprintf('umax:%.2f\nOvershoot:%.2f\nSettling Time:%.2f\n',umax, S.Overshoot, S.SettlingTime);
Tu
S
%}
%% Another Controller
%{
G = zpk(G)

z1 = -.0001 + j*.0001;   % Pole, Zero, and K
z2 = -.0001 - j*.0001;
p1 = -1000;
p2 = -2000;
K = 10E9;


D = zpk([z1,z2,z3,z4,z5], [p1,p2,p3,p4,p5], 1);
figure(1)
rlocus(D*G); 

D = K*D;  % Multiply in K
T = feedback(D*G, 1);  % Get transfer function T
[Tu, umax] = controleffort(G, T);
S = stepinfo(T);

kd = K/p;   % convert from poles, zeros, and k to parameters
kp = z * kd;

D   % printing
fprintf('Kd:%.2f\nKp:%.2f\n',kd,kp);
figure(2)
step(T);
fprintf('umax:%.2f\nOvershoot:%.2f\nSettling Time:%.2f\n',umax, S.Overshoot, S.SettlingTime);
Tu
S
%}
%% New Plant Design

motor = motors(1);
B_m = motor.B_m;
K_t = motor.K_t;
L = motor.L;
R = motor.R;
J_eff = motor.J_m + J_g + (J_bar + J_s) / N^2; % kg*m^2

s = tf('s');
  
num = G_v * K_t;

den = s^3 * N  * (L * J_eff + s^-1*(B_m * L + R * J_eff) +  ...
      s^-2 * (K_t^2 + B_m * R));
  
G1 = num / den;

num = ball_mass * g * K_s;

den = s^2 * (ball_mass + (J_b / ball_radius^2));

G2 = num / den;

G1 = minreal(G1);
G2 = minreal(G2);

%% Motor Controller for G1
G1 = zpk(G1)

z1 = -10;   % Pole, Zero, and K
p1 = -15;
K = 40;


D = zpk([z1], [p1], 1);
figure(1)
rlocus(D*G1); 

D = K*D;  % Multiply in K
T = feedback(D*G1, 1);  % Get transfer function T
[Tu, umax] = controleffort(G1, T);
S = stepinfo(T);
step(T);

%% Actual Plant (Motor Controller & Ball Dynamics)
%{
G = T*G2;

Controller = load('controller.mat');
% (from pidTuner, PDF)
% (saved as controller.mat)
%{
D =
 
  9.3617 (s+0.02085)
  ------------------
      (s+272.3)
%}

D = zpk(Controller.C); % convert from pid format to zpk

T = feedback(D*G, 1);  % Get transfer function T
[Tu, umax] = controleffort(G, T);
S = stepinfo(T);
figure(1);
step(T);
S
%}