function [c,ceq] = constraint(z, N,mx, alpha, betta, lambda_t)
ceq=[];
c = alpha*exp(-betta*(z(1:mx:N*mx) - lambda_t).^2) - z(5:mx:N*mx);
