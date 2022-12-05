%%
%  File : portfolio_5_card.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :  Implements a basic portfolio optimization model
%                 with cardinality constraints on number of assets traded.
% 
%%
function portfolio_5_card()

n      = 3;
w      = 1.0;   
mu     = [0.1073,0.0737,0.0627]';
x0     = [0.0,0.0,0.0]';
gammas = [0.035,0.040,0.050,0.060,0.070,0.080 ,0.090]';
GT     = [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238  ; ...
           0.0              , 0.102863378954911  , -0.00222873156550421 ; ...
           0.0              , 0.0                ,  0.0338148677744977 ];

disp(sprintf('\nMarkowitz portfolio optimization with cardinality constraints'));
for k=1:n
    x = MarkowitzWithCardinality(n,mu,GT,x0,w,gammas(1),k);
    disp(sprintf('Bound: %d   Expected return: %.4e  Solution:', k, mu'*x));
    disp(x);
end

%
%    Description:
%        Extends the basic Markowitz model with cardinality constraints.
%
%    Input:
%        n: Number of assets
%        mu: An n dimensional vector of expected returns
%        GT: A matrix with n columns so (GT')*GT  = covariance matrix
%        x0: Initial holdings 
%        w: Initial cash holding
%        gamma: Maximum risk (=std. dev) accepted
%        k: Maximum number of assets on which we allow to change position.
%
%    Output:
%       Optimal expected return and the optimal portfolio     
%
function x = MarkowitzWithCardinality(n,mu,GT,x0,w,gamma,k)

[rcode, res] = mosekopt('symbcon');

% Upper bound on the traded amount
u = w+sum(x0);

% unrolled variable ordered as (x, z, y)
prob = [];
prob.c = [mu; zeros(2*n,1)];
In = speye(n);
On = sparse([],[],[],n,n);

% Linear constraints
% [ e'  0   0  ]           =   w + e'*x0
% [ I  -I   0  ]   [ x ]  <=  x0
% [ I   I   0  ] * [ z ]  >=  x0
% [ 0   I  -U  ]   [ y ]  <=  0
% [ 0   0   e' ]          <=  k
prob.a   = [ [ones(1,n), zeros(1,2*n)]; In -In On; In In On; On In -u*In; zeros(1,2*n) ones(1,n) ]; 
prob.blc = [ w + sum(x0); -inf*ones(n,1); x0; -inf*ones(n,1); 0 ];
prob.buc = [ w + sum(x0); x0; inf*ones(n,1); zeros(n,1); k ];

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
