% Extended Kalman Filter Reinforcement learning
clear all;
clc;

% init
dt = 1/1; % 50hz
x = [-3 7]'; % x0=0m, v0 = 30km/h(= 0.166667m/hz)
P = [10 0;
    0 10];
A = [1 dt;
    0 1];
Q = [10 0;
    0 1];
H = [0 1];
    
R = 2; % var(pos) = 2m, var(vel) = 3km/h

z_pos_min=-2; z_pos_max=2;
z_vel_min=-0.01; z_vel_max=0.01;

t_length = 360/dt; % sec/dt
z_vel_true = 8.3333; % 30km/h(= 0.166667m/hz)

% save estimation value
estimated_x = zeros(1,t_length);
estimated_v = zeros(1,t_length);
% save measurement value
measurement_x = zeros(1,t_length);
measurement_v = zeros(1,t_length);

% save true value
z_pos_true = 0;
true_pos_set = zeros(1,t_length);
true_vel_set = zeros(1,t_length);

for i=1:t_length
    
    z_pos_true = z_pos_true; 
    z_pos_error = z_pos_min+rand(1)*(z_pos_max-z_pos_min);    
    z_vel_true = z_vel_true; 
    z_vel_error = z_vel_min+rand(1)*(z_vel_max-z_vel_min);    
    %z = [z_pos_true+z_pos_error z_vel_true+z_vel_error]';
    z = z_vel_true+z_vel_error;
    
    xp = A*x;
    Pp = A*P*A' + Q;
    
    y = z - H*xp;
    S = H*Pp*H' + R;
    K = Pp*H'*inv(S);
    
    x = xp + K*y;
    P = Pp-K*H*Pp;
    
    % estimation
    estimated_x(i) = x(1);
    estimated_v(i) = x(2);
    
    % measurement
    measurement_x(i) = z_pos_true+z_pos_error;
    measurement_v(i) = z_vel_true+z_vel_error;
    
    % true
    z_pos_true = z_pos_true + z_vel_true*dt;
    true_pos_set(i) = z_pos_true;
    true_vel_set(i) = z_vel_true;
%     plot(i,x(2),'b.');
%     hold on
%     drawnow
end
figure
plot(estimated_x); hold on;
plot(true_pos_set, 'r');
plot(measurement_x, 'k');
figure
plot(estimated_v); hold on;
plot(true_vel_set, 'r');
plot(measurement_v, 'k');