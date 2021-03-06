
%problem_3;
clear; clf; clc;
init;

% Continous time system
A_c = [0 1 0       0         0         0;
     0 0 -K_2      0         0         0;
     0 0 0         1         0         0;
     0 0 -K_1*K_pp -K_1*K_pd 0         0;
     0 0  0        0         0         1;
     0 0  0        0         -K_3*K_ep -K_3*K_ed];
 
B_c = [0 0 0 K_1*K_pp 0 0;
       0 0 0 0        0 K_3*K_ep]';
%% 
N = 40;
M = N;
mx = size(A_c,2);
mu = size(B_c,2);
x0 = [pi 0 0 0 0 0]';

% non linear constraint
alpha = 0.2;
betta = 20;
lambda_t = 2*pi/3;

% Discrete time system model
delta_t	= 0.25; % sampling time
A_d = eye(size(A_c)) + delta_t*A_c;
B_d = B_c*delta_t;

%%
z0 = [repmat(x0,N,1); repmat([0 0]',N,1)];

Q = diag([1 0 0 0 0 0]);
q1 = 1;     % pitch-cost
q2 = 1.5;   % elevation-cost
R = [q1 0; 
    0 q2];
Q11 = kron(eye(N),Q);                          % Weight on state x1
Q21 = zeros(N*mu, N*mx);
Q12 = Q21';
Q22 = kron(eye(N), R);
G = [ Q11 Q12;
      Q21 Q22]; 

phi = @(z) 0.5*z'*G*z;

%% Generate system matrixes for linear model
Aeq = gen_aeq(A_d,B_d,N,mx,mu);             % Generate A, hint: gen_aeq

beq = [A_d*x0;
       zeros(size(Aeq,1)-size(x0,1),1)];             % Generate b
   
   
%%   

% Bounds
ul = -Inf*ones(mu,1);
uu = Inf*ones(mu,1);
ul(1) = -30*pi/180;                   % Lower bound on pitch
uu(1) = +30*pi/180;                   % Upper bound on pitch
ul(2) = -30*pi/180;                   % Lower bound on elevation
uu(2) = +45*pi/180;                   % Upper bound on elevation

xl      = -Inf*ones(mx,1);            % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);             % Upper bound on states (no bound)
xl(3)   = ul(1);                      % Lower bound on state pitch
xu(3)   = uu(1);                      % Upper bound on state pitch
xl(5)   = ul(2);                      % Lower bound on state elevation
xu(5)   = uu(2);                      % Upper bound on state elevation

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,N,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+N*mu)  = 0;                   % We want the last input to be zero
vub(N*mx+N*mu)  = 0;                   % We want the last input to be zero


nonlincon = @(z) constraint(z,N,mx,alpha,betta, lambda_t);

options = optimoptions('fmincon')
options.MaxFunEvals = 10000;
z = fmincon(phi,z0,[],[],Aeq,beq,vlb,vub,nonlincon, options);

%% Plotting
%t = 0:delta_t:(N-1)*delta_t;

num_zeros = 5/delta_t;
zero_padding = zeros(num_zeros,1);
unit_padding = ones(num_zeros,1);

u_pitch = [zero_padding; z((N*mx+1):mu:end); zero_padding];
u_elevation = [zero_padding; z((N*mx+2):mu:end); zero_padding];

u = [u_pitch u_elevation];
t = 5 +(-5:delta_t:(N-1)*delta_t+5);

travel = [pi*unit_padding; z(1:mx:N*mx); zero_padding];
travel_rate = [zero_padding; z(2:mx:N*mx); zero_padding];
pitch = [zero_padding; z(3:mx:N*mx); zero_padding];
pitch_rate = [zero_padding; z(4:mx:N*mx); zero_padding];
elevation = [zero_padding; z(5:mx:N*mx); zero_padding];
elevation_rate = [zero_padding; z(6:mx:N*mx); zero_padding];

Q = diag([10 0 100 0 100 0]);
R = diag([1 1]);
[K,S,e] = dlqr(A_d,B_d,Q,R); % closed loop
%K = zeros(mu,mx); % open loop
x_star = [travel travel_rate pitch pitch_rate elevation elevation_rate];

figure(2)
clf;
hold on;
plot(t,pitch, 'r');
plot(t,travel, 'g');
plot(t,elevation, 'b');
legend({'p', '\lambda', 'e'})


figure(2)
subplot(811)
stairs(t,u_pitch),grid
ylabel('p_c')
subplot(812)
stairs(t,u_elevation),grid
ylabel('e_c')
subplot(813)
plot(t,travel,'m',t,travel,'mo'),grid
ylabel('lambda')
subplot(814)
plot(t,travel_rate,'m',t,travel_rate','mo'),grid
ylabel('r')
subplot(815)
plot(t,pitch,'m',t,pitch,'mo'),grid
ylabel('p')
subplot(816)
plot(t,pitch_rate,'m',t,pitch_rate','mo'),grid
ylabel('pdot')
subplot(817)
plot(t,elevation,'m',t,elevation','mo'),grid
ylabel('e')
subplot(818)
plot(t,elevation_rate,'m',t,elevation_rate','mo'),grid
ylabel('edot'), xlabel('tid (s)')


figure(4)
lambdas=0:0.01:pi;
plot(lambdas, alpha*exp(-betta*(lambdas - lambda_t).^2));
hold on;
plot(travel,elevation);

%% Real data
raw = load('p44_closedloop.mat');
%
data = raw.data';
real_t = data(:,1);
real_travel = data(:,2);
%travel_rate = data(:,3);
%pitch = data(:,4);
%pitch_rate = data(:,5);
real_elevation = data(:,6);
%elevation_rate = data(:,7);
%
%
%figure(4);
%lambdas=0:0.01:pi;
%plot(lambdas, alpha*exp(-betta*(lambdas - lambda_t).^2));
%hold on
plot(real_travel(real_t > 5), real_elevation(real_t > 5));
xlabel('Travel [rad]');
ylabel('Elevation [rad]');
title('Obstacle avoidance performance', 'fontweight', 'bold');







