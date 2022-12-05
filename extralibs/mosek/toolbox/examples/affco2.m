%%
%  File : affco2.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :
%    Implements a basic tutorial example with affine conic constraints:
%
%    minimize t
%    st.      (d, z1*y1,... zn*yn) \in Q^{n+1}
%             (yi, 1, ai*t)        \in EXP, i=1,\ldots,n
%
%    with input ai<0, zi, d.
%
%    See also https://docs.mosek.com/modeling-cookbook/expo.html#hitting-time-of-a-linear-system
% 
%%
function affco2()

n = 2;
z = [2.2 1.3]';
a = [-0.3 -0.06]';
d = 0.5;

t = firstHittingTime(n, z, a, d);
disp(sprintf('\nt = %.4e', t))


function t = firstHittingTime(n, z, a, d)

[rcode, res] = mosekopt('symbcon echo(0)');
prob = [];

% Variables [t, y1, ..., yn]
prob.a = sparse(0, n+1);
prob.c = [1 zeros(1,n)];

% Quadratic cone
FQ = diag([0; z]);
gQ = [d; zeros(n,1)];

% All exponential cones
FE = sparse([1:3:3*n    3:3:3*n], ...
            [2:n+1      ones(1,n)], ...
            [ones(1,n)  a']);
gE = repmat([0; 1; 0], n, 1);

% Assemble input data
prob.f = [FQ; FE];
prob.g = [gQ; gE];
prob.cones = [res.symbcon.MSK_CT_QUAD n+1 repmat([res.symbcon.MSK_CT_PEXP 3], 1, n)];

% Solve
[r, res] = mosekopt('minimize', prob);
t = res.sol.itr.xx(1)