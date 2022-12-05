%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      simple.m
%
%  Purpose :   To demonstrate how solve a problem
%              read from file.
%

function simple(inputfile, solfile)

cmd      = sprintf('read(%s)', inputfile)
% Read the problem from file
[rcode, res] = mosekopt(cmd)

% Perform the optimization.
[r,res] = mosekopt('minimize', res.prob); 

% Show the optimal x solution.
res.sol.bas.xx



end