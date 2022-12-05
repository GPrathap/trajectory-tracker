%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      rlo2.m
%
%  Purpose :   Solves the problem:
%
%              Maximize t 
%              subject to
%                        t <= sum(delta(j)*x(j)) -Omega*z,
%                        y(j) = sigma(j)*x(j), j=1,...,n,
%                        sum(x(j)) = 1,
%                        || y || <= z,
%

function rlo2(n, Omega, draw)

n = str2num(n)
Omega = str2num(Omega)
draw

[r, res] = mosekopt('symbcon echo(0)');
sym = res.symbcon;

% Set nominal returns and volatilities
delta = (0.96/(n-1))*[0:1:n-1]+1.04;
sigma = (1.152/(n-1))*[0:1:n-1];

% Set mosekopt description of the problem
prob.c             = [1;zeros(n+1,1)];
A                  = [-1, delta,     -Omega; ...
                       0, ones(1,n), 0];
prob.a             = sparse(A);
prob.blc           = [0;1];
prob.buc           = [inf;1];
prob.blx           = [-inf;zeros(n,1);0];
prob.bux           = inf*ones(n+2,1);
F                  = [zeros(1,n+1), 1; ...
                      zeros(n,1), diag(sigma), zeros(n,1)];
prob.f             = sparse(F);
prob.cones         = [ sym.MSK_CT_QUAD n+1 ];

% Run mosekopt
[r,res]=mosekopt('maximize echo(3)',prob);

xx = res.sol.itr.xx;
t  = xx(1);

disp(sprintf('Robust optimal value: %5.4f',t));
x = max(xx(2:1+n),zeros(n,1));

if draw == true 
    % Display the solution
    plot([1:1:n],x,'-m');
    grid on;
    
    disp('Press <Enter> to run simulations');
    pause
    
    % Run simulations
    
    Nsim = 10000;
    out  = zeros(Nsim,1);
    for i=1:Nsim,
        returns  = delta+(2*rand(1,n)-1).*sigma;
        out(i)   = returns*x;
    end;
    disp(sprintf('Actual returns over %d simulations:',Nsim));
    disp(sprintf('Min=%5.4f Mean=%5.4f Max=%5.4f StD=%5.2f',...
                 min(out),mean(out),max(out),std(out)));
    hist(out);
end