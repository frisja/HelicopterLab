% Discrete LQR
problem_2;

Q = diag([10 0 1000 0]);
%Q = diag([10 0 0 1000]);
R = diag(1);
[K,S,e] = dlqr(A_d,B_d,Q,R); % closed loop
%K = zeros(mu,mx); % open loop

x_star = [x1 x2 x3 x4];

