%%
%  File : portfolio_3_impact.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :  Implements a basic portfolio optimization model
%                 with transaction costs of order x^(3/2).
% 
%%

function portfolio_3_impact()

n      = 3;
w      = 1.0;   
mu     = [0.1073,0.0737,0.0627]';
x0     = [0.0,0.0,0.0]';
gammas = [0.035,0.040,0.050,0.060,0.070,0.080 ,0.090]';
GT     = [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238  ; ...
           0.0              , 0.102863378954911  , -0.00222873156550421 ; ...
           0.0              , 0.0                ,  0.0338148677744977 ];

% Somewhat arbirtrary choice of m
m = 1.0e-2*ones(n,1);
gamma = gammas(1);
[x, t] = MarkowitzWithMarketImpact(n,mu,GT,x0,w,gamma,m);
disp(sprintf('\nMarkowitz portfolio optimization with market impact cost'));
disp(sprintf('Expected return: %.4e Std. deviation: %.4e Market impact cost: %.4e', mu'*x, gamma, m'*t));

%
%        Description:
%            Extends the basic Markowitz model with a market cost term.
%
%        Input:
%            n: Number of assets
%            mu: An n dimmensional vector of expected returns
%            GT: A matrix with n columns so (GT')*GT  = covariance matrix
%            x0: Initial holdings 
%            w: Initial cash holding
%            gamma: Maximum risk (=std. dev) accepted
%            m: It is assumed that  market impact cost for the j'th asset is
%               m_j|x_j-x0_j|^3/2
%
%        Output:
%           Optimal expected return and the optimal portfolio     
%
function [x, t] = MarkowitzWithMarketImpact(n,mu,GT,x0,w,gamma,m)

[rcode, res] = mosekopt('symbcon');

% unrolled variable ordered as (x, t)
prob = [];
prob.c = [mu; zeros(n,1)];

In = speye(n);
On = sparse([],[],[],n,n);

% Linear part
% [ e' m' ]  * [ x; t ]  =   w + e'*x0
prob.a   = [ ones(1,n), m' ]; 
prob.blc = [ w + sum(x0) ];
prob.buc = [ w + sum(x0) ];

% No shortselling and no other bounds
prob.blx = [ zeros(n,1); -inf*ones(n,1) ];
prob.bux = inf*ones(2*n,1);

% Affine conic constraints representing [ gamma, GT*x ] in quadratic cone
prob.f = sparse([ zeros(1,2*n); [GT On] ]);
prob.g = [gamma; zeros(n,1)];
prob.cones = [ res.symbcon.MSK_CT_QUAD n+1 ];

% Extend the affine conic constraints
% with power cones representing t(i) >= |x(i)-x0(i)|^1.5
fi = [];
fj = [];
g  = [];
fv = repmat([1; 1], n, 1);
for k=1:n
    fi = [fi; 3*k-2; 3*k];
    fj = [fj; n+k; k];
    g  = [g; 0; 1; -x0(k)];
end
prob.f = [prob.f; sparse(fi, fj, fv)];
prob.g = [prob.g; g];
prob.cones = [prob.cones repmat([res.symbcon.MSK_CT_PPOW, 3, 2, 2.0, 1.0], 1, n) ];

[rcode,res] = mosekopt('maximize echo(0)',prob,[]);

x = res.sol.itr.xx(1:n);
t = res.sol.itr.xx(n+(1:n));