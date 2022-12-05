%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      pow1.m
%
%  Purpose:   To demonstrate how to solve the problem
%
%     maximize x^0.2*y^0.8 + z^0.4 - x
%           st x + y + 0.5z = 2
%              x,y,z >= 0

function pow1()

clear prob;

[r, res] = mosekopt('symbcon');
% Specify the non-conic part of the problem.

prob.c   = [-1 0 0 1 1 0];
prob.a   = [1 1 0.5 0 0 0];
prob.blc = [2.0];
prob.buc = [2.0];
prob.blx = [-inf -inf -inf -inf -inf 1.0];
prob.bux = [ inf  inf  inf  inf  inf 1.0];

% Specify the cones.
prob.cones.type   = [res.symbcon.MSK_CT_PPOW res.symbcon.MSK_CT_PPOW];
prob.cones.conepar= [0.2 0.4];
prob.cones.sub    = [1 2 4 3 6 5];
prob.cones.subptr = [1 4];
% The field 'type' specifies the cone types, in this case power cones.
%
% The fields 'sub' and 'subptr' specify the members of the cones,
% i.e., the above definitions imply that 
%   (x(1), x(2), x(4))  and (x(3), x(6), x(5)) 
% are cones.
%
% The field 'conepar' specifies the alpha cone parameters (exponents)

% Optimize the problem. 

[r,res]=mosekopt('maximize',prob);

% Display the primal solution.

res.sol.itr.xx'