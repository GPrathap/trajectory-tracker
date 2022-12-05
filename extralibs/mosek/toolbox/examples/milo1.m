%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File      : milo1.m
%
%  Purpose   : Demonstrates how to solve a small mixed
%              integer linear optimization problem.
%

function milo1()
clear prob       
prob.c        = [1 0.64];
prob.a        = [[50 31];[3 -2]];
prob.blc      = [-inf -4];
prob.buc      = [250 inf];
prob.blx      = [0 0];
prob.bux      = [inf inf];

% Specify indexes of variables that are integer
% constrained.

prob.ints.sub = [1 2];

% Optimize the problem.
[r,res] = mosekopt('maximize',prob);

try 
  % Display the optimal solution.
  res.sol.int
  res.sol.int.xx'
catch
  fprintf('MSKERROR: Could not get solution')
end