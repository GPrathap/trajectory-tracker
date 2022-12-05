%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      sdo1.m
%
%  Purpose :   Solves the mixed semidefinite and conic quadratic optimization problem
%
%                 minimize    Tr [2, 1, 0; 1, 2, 1; 0, 1, 2]*X + x(1)
%
%                 subject to  Tr [1, 0, 0; 0, 1, 0; 0, 0, 1]*X + x(1)               = 1
%                             Tr [1, 1, 1; 1, 1, 1; 1, 1, 1]*X        + x(2) + x(3) = 0.5
%                             X>=0,  x(1) >= sqrt(x(2)^2 + x(3)^2)
%


function sdo1()
[r, res] = mosekopt('symbcon');

prob.c         = [1, 0, 0];

prob.bardim    = [3];
prob.barc.subj = [1, 1, 1, 1, 1];
prob.barc.subk = [1, 2, 2, 3, 3];
prob.barc.subl = [1, 1, 2, 2, 3];
prob.barc.val  = [2.0, 1.0, 2.0, 1.0, 2.0];

prob.blc = [1, 0.5];
prob.buc = [1, 0.5];

% It is a good practice to provide the correct 
% dimmension of A as the last two arguments
% because it facilitates better error checking.
prob.a         = sparse([1, 2, 2], [1, 2, 3], [1, 1, 1], 2, 3);
prob.bara.subi = [1, 1, 1, 2, 2, 2, 2, 2, 2];
prob.bara.subj = [1, 1, 1, 1, 1, 1, 1, 1, 1];
prob.bara.subk = [1, 2, 3, 1, 2, 3, 2, 3, 3];
prob.bara.subl = [1, 2, 3, 1, 1, 1, 2, 2, 3];
prob.bara.val  = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];

prob.cones.type   = [res.symbcon.MSK_CT_QUAD];
prob.cones.sub    = [1, 2, 3];
prob.cones.subptr = [1];

[r,res] = mosekopt('minimize info',prob); 

X = zeros(3);
X([1,2,3,5,6,9]) = res.sol.itr.barx;
X = X + tril(X,-1)';

x = res.sol.itr.xx;