%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File      : mico1.m
%
%  Purpose   : Demonstrates how to solve a small mixed
%              integer conic optimization problem.
%
%              minimize    x^2 + y^2
%              subject to  x >= e^y + 3.8
%                          x, y - integer
%

function mico1()

[rcode, res] = mosekopt('symbcon echo(0)');
symbcon = res.symbcon;
clear prob       

% The full variable is [t; x; y]
prob.c        = [1 0 0];
prob.a        = sparse(0,3);   % No constraints

% Conic part of the problem
prob.f = sparse([ eye(3); 
                  0 1 0; 
                  0 0 0; 
                  0 0 1 ]);
prob.g = [0 0 0 -3.8 1 0]';
prob.cones = [symbcon.MSK_CT_QUAD 3 symbcon.MSK_CT_PEXP 3];

% Specify indexes of variables that are integers
prob.ints.sub = [2 3];

% It is as always possible (but not required) to input an initial solution
% to start the mixed-integer solver. 
prob.sol.int.xx = [0, 9, -1];

% Optimize the problem.
[r,res] = mosekopt('minimize',prob);

% The integer solution (x,y) 
res.sol.int.xx(2:3)
