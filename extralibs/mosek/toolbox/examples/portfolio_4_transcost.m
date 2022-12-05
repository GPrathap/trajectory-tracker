%%
%  File : portfolio_4_transcost.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :  Implements a basic portfolio optimization model
%                 with fixed setup costs and transaction costs
%                 as a mixed-integer problem.
% 
%%

function portfolio_4_transcost()

n      = 3;
w      = 1.0;   
mu     = [0.1073,0.0737,0.0627]';
x0     = [0.0,0.0,0.0]';
gammas = [0.035,0.040,0.050,0.060,0.070,0.080 ,0.090]';
GT     = [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238  ; ...
           0.0              , 0.102863378954911  , -0.00222873156550421 ; ...
           0.0              , 0.0                ,  0.0338148677744977 ];

f = 0.01*ones(n,1);
g = 0.001*ones(n,1);
gamma = gammas(1);
[x, z, y] = MarkowitzWithTransactionsCost(n,mu,GT,x0,w,gamma,f,g);
disp(sprintf('\nMarkowitz portfolio optimization with transactions cost'))
disp(sprintf('Expected return: %.4e Std. deviation: %.4e Transactions cost: %.4e', ...
                 mu'*x,gamma,f'*y+g'*z))

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
%            f: If asset j is traded then a fixed cost f_j must be paid
%            g: If asset j is traded then a cost g_j must be paid for each unit traded
%
%        Output:
%           Optimal expected return and the optimal portfolio     
%
function [x, z, y] = MarkowitzWithTransactionsCost(n,mu,GT,x0,w,gamma,f,g)

[rcode, res] = mosekopt('symbcon');

% Upper bound on the traded amount
u = w+sum(x0);

% unrolled variable ordered as (x, z, y)
prob = [];
prob.c = [mu; zeros(2*n,1)];
In = speye(n);
On = sparse([],[],[],n,n);

% Linear constraints
% [ e'  g'  f' ]   [ x ]  =   w + e'*x0
% [ I  -I   0  ] * [ z ]  <=  x0
% [ I   I   0  ]   [ y ]  >=  x0
% [ 0   I  -U  ]          <=  0
prob.a   = [ [ones(1,n), g', f']; In -In On; In In On; On In -u*In ]; 
prob.blc = [ w + sum(x0); -inf*ones(n,1); x0; -inf*ones(n,1) ];
prob.buc = [ w + sum(x0); x0; inf*ones(n,1); zeros(n,1) ];

% No shortselling and the linear bound 0 <= y <= 1 
prob.blx = [ zeros(n,1); -inf*ones(n,1); zeros(n,1) ];
prob.bux = [ inf*ones(2*n,1); ones(n,1) ];

% Affine conic constraints representing [ gamma, GT*x ] in quadratic cone
prob.f = sparse([ zeros(1,3*n); [GT On On];  ]);
prob.g = [gamma; zeros(n,1)];
prob.cones = [ res.symbcon.MSK_CT_QUAD n+1 ];

% Demand y to be integer (hence binary)
prob.ints.sub = 2*n+(1:n);

[rcode,res] = mosekopt('maximize echo(0)',prob,[]);

x = res.sol.int.xx(1:n);
z = res.sol.int.xx(n+(1:n));
y = res.sol.int.xx(2*n+(1:n));