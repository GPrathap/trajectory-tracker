%%
%  File : portfolio_1_basic.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :
%    Implements a basic portfolio optimization model.
% 
%%
function portfolio_1_basic()

n      = 3;
w      = 1.0;   
mu     = [0.1073,0.0737,0.0627]';
x0     = [0.0,0.0,0.0]';
gammas = [0.035,0.040,0.050,0.060,0.070,0.080 ,0.090]';
GT     = [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238  ; ...
           0.0              , 0.102863378954911  , -0.00222873156550421 ; ...
           0.0              , 0.0                ,  0.0338148677744977 ];
         
disp('Basic Markowitz portfolio optimization')
for gamma = gammas'
    er = BasicMarkowitz(n,mu,GT,x0,w,gamma);
    disp(sprintf('Expected return: %.4e Std. deviation: %.4e', er, gamma));
end


%
%    Purpose:
%        Computes the optimal portfolio for a given risk 
%     
%    Input:
%        n: Number of assets
%        mu: An n dimmensional vector of expected returns
%        GT: A matrix with n columns so (GT')*GT  = covariance matrix
%        x0: Initial holdings 
%        w: Initial cash holding
%        gamma: Maximum risk (=std. dev) accepted
%     
%    Output:
%        Optimal expected return and the optimal portfolio     
%
function er = BasicMarkowitz(n,mu,GT,x0,w,gamma)

[rcode, res] = mosekopt('symbcon');
prob = [];

% Objective vector - expected return
prob.c = mu;

% The budget constraint  - e'x = w + sum(x0)
prob.a = ones(1,n);
prob.blc = w + sum(x0);
prob.buc = w + sum(x0);

% Bounds exclude shortselling
prob.blx = zeros(n,1);
prob.bux = inf*ones(n,1); 

% An affine conic constraint: [gamma, GT*x] in quadratic cone
prob.f = sparse([ zeros(1,n); GT ]);
prob.g = [gamma; zeros(n,1)];
prob.cones = [ res.symbcon.MSK_CT_QUAD n+1 ];

% Maximize problem and return the objective value
[rcode,res] = mosekopt('maximize echo(0)', prob, []);
x = res.sol.itr.xx;    
er = mu'*x;