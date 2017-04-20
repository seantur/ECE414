%% Densities
stainless_density = 7.97; % g/cm^3
aluminum_density = 2.7; % g/cm^3
%% Motors
motor1 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_T', .225, 'R', 8, 'L', .025);
motor2 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_T', .175, 'R', 6, 'L', .016);
motor3 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_T', .125, 'R', 4, 'L', .0075);
motor4 = struct('J_m', 5.0e-5, 'B_m', 3.0e-6, 'K_T', .275, 'R', 12, 'L', .032);
motors = [motor1, motor2, motor2, motor4];
%% Ball Properties
ball_radius = .01; % m
ball_volume = (4/3) * pi * (ball_radius*100)^3; % cm^3
ball_mass = stainless_density * ball_volume / 1000; % kg
J_b= (2/5) * ball_mass * (ball_radius/100)^2; % kg*m^2
%% Gearbox & Other
J_g = 6.2e-6; % kg*m^2
J_s = 1.4e-7; % kg*m^2
G_v = 5; % V/V
g = 9.8; % m/s^2
K_s = 10; % V/m
%% Plant Trasnfer Function
