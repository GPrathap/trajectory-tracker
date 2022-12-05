%%
%  File : portfolio_2_frontier.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :  Implements a basic portfolio optimization model.
%                 Determines points on the efficient frontier.
% 
%%

function portfolio_2_frontier()

n      = 3;
w      = 1.0;   
mu     = [0.1073,0.0737,0.0627]';
x0     = [0.0,0.0,0.0]';
GT     = [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238  ; ...
           0.0              , 0.102863378954911  , -0.00222873156550421 ; ...
           0.0              , 0.0                ,  0.0338148677744977 ];

% Some predefined alphas are chosen
alphas = 0.6:0.1:15.0;
front = EfficientFrontier(n,mu,GT,x0,w,alphas);
disp(sprintf('\nEfficient frontier'));
disp(sprintf('%-12s  %-12s  %-12s', 'alpha', 'return', 'risk'));
for p = front.'
    disp(sprintf('%-12.4f  %-12.4e  %-12.4e', p(1), p(2), p(3)));
end
plot(front(:,3),front(:,2));


%
%    Purpose:
%        Computes several portfolios on the optimal portfolios by 
%
%            for alpha in alphas: 
%                maximize   expected return - alpha * variance
%                subject to the constraints  
%        
%    Input:
%        n: Number of assets
%        mu: An n dimmensional vector of expected returns
%        GT: A matrix with n columns so (GT')*GT  = covariance matrix
%        x0: Initial holdings 
%        w: Initial cash holding
%        alphas: List of the alphas
%                    
%    Output:
%        The efficient frontier as list of tuples (alpha, expected return, variance)
%
function frontier = EfficientFrontier(n,mu,GT,x0,w,alphas)

frontier = [];
[rcode, res] = mosekopt('symbcon');
prob = [];

% The budget constraint in terms of variables [x; s]
prob.a = [ones(1,n), 0.0];
prob.blc = w + sum(x0);
prob.buc = w + sum(x0);

% No shortselling
prob.blx = [zeros(n,1); -inf];
prob.bux = inf*ones(n+1,1); 

% An affine conic constraint: [s, 0.5, GT*x] in rotated quadratic cone
% In matrix form
% [ 0  1]  [ x ]     [ 0   ]
% [ 0  0]  [   ]  +  [ 0.5 ]  \in Q_r
% [ GT 0]  [ s ]     [ 0   ]  
prob.f = sparse([ [zeros(1,n), 1.0]; zeros(1, n+1); [GT, zeros(n,1)] ]);
prob.g = [ 0; 0.5; zeros(n, 1) ]
prob.cones = [ res.symbcon.MSK_CT_RQUAD n+2 ];

for alpha = alphas
    % Objective mu'*x - alpha*s
    prob.c = [mu; -alpha];

    [rcode,res] = mosekopt('maximize echo(0)',prob,[]);
    x = res.sol.itr.xx(1:n);
    s = res.sol.itr.xx(n+1);    
       
    frontier = [frontier; [alpha, mu'*x, s] ];
end