%%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      mioinitsol.m
%
%   Purpose:   To demonstrate how to solve a MIP with a start
%   guess.
%%

function mioinitsol()
[r,res]       = mosekopt('symbcon');
sc            = res.symbcon;

clear prob

prob.c        = [7 10 1 5];
prob.a        = sparse([1 1 1 1 ]);
prob.blc      = -[inf];
prob.buc      = [2.5];
prob.blx      = [0 0 0 0];
prob.bux      = [inf inf inf inf];
prob.ints.sub = [1 2 3];

% Specify start guess for the integer variables.
prob.sol.int.xx  = [1 1 0 nan]';

[r,res] = mosekopt('maximize info',prob);

try
  % Display the optimal solution.
  res.sol.int.xx'

  % Was the initial solution used ?
  res.info.MSK_IINF_MIO_CONSTRUCT_SOLUTION
  res.info.MSK_DINF_MIO_CONSTRUCT_SOLUTION_OBJ

catch
  fprintf('MSKERROR: Could not get solution')
end