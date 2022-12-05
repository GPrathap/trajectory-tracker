%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      ceo1.m
%
%  Purpose:   To demonstrate how to solve a small conic exponential
%              optimization problem using the MOSEK Matlab Toolbox.
%

function ceo1()

clear prob;

[r, res] = mosekopt('symbcon');
% Specify the non-conic part of the problem.

prob.c   = [1 1 0];
prob.a   = sparse([1 1 1]);
prob.blc = 1;
prob.buc = 1;
prob.blx = [-inf -inf -inf];
prob.bux = [ inf  inf  inf];

% Specify the cones.

prob.cones.type   = [res.symbcon.MSK_CT_PEXP];
prob.cones.sub    = [1, 2, 3];
prob.cones.subptr = [1];
% The field 'type' specifies the cone types, in this case an exponential
% cone with key MSK_CT_PEXP.
%
% The fields 'sub' and 'subptr' specify the members of the cones,
% i.e., the above definitions imply that 
%   x(1) >= x(2)*exp(x(3)/x(2))

% Optimize the problem. 

[r,res]=mosekopt('minimize',prob);

% Display the primal solution.

res.sol.itr.xx'