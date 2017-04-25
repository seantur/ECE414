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

%% Plant Trasnfer Function
motor = motors(2);
B_m = motor.B_m;
K_t = motor.K_t;
L = motor.L;
R = motor.R;
J_eff = motor.J_m + J_g + (J_bar + J_s) / N^2; % kg*m^2


s = tf('s');
bar_eq = (ball_mass * ball_radius^2 + J_b);

num = G_v * ball_mass * g * K_s * K_t ; 
den = s^5 * N * J_eff * L * bar_eq + ...
      s^4 * N * (B_m * L + R * J_eff) * bar_eq + ...
      s^3 * N * (K_t^2 + B_m * R) * bar_eq;
  
G = num / den;
G = minreal(G)

