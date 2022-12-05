%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      qcqo1.m
%
%  Purpose : Demonstrates a simple quadratically constrained quadratic problem 
%            minimize  x_1^2 + 0.1 x_2^2 +  x_3^2 - x_1 x_3 - x_2
%            s.t 1 <=  x_1 + x_2 + x_3 - x_1^2 - x_2^2 - 0.1 x_3^2 + 0.2 x_1 x_3
%            x >= 0
%
function qcqo1()
clear prob;

% Specify the linear objective terms.
prob.c      = [0, -1, 0];

% Specify the quadratic terms of the constraints.
prob.qcsubk = [1     1    1   1  ]';
prob.qcsubi = [1     2    3   3  ]';
prob.qcsubj = [1     2    3   1  ]';
prob.qcval  = [-2.0 -2.0 -0.2 0.2]';

% Specify the quadratic terms of the objective.
prob.qosubi = [1     2    3    3  ]';
prob.qosubj = [1     2    3    1  ]';
prob.qoval  = [2.0   0.2  2.0 -1.0]';

% Specify the linear constraint matrix
prob.a      = [1 1 1];

% Specify the lower bounds
prob.blc    = [1];
prob.blx    = zeros(3,1);

[r,res]     = mosekopt('minimize',prob);

% Display the solution.
fprintf('\nx:');
fprintf(' %-.4e',res.sol.itr.xx');
fprintf('\n||x||: %-.4e',norm(res.sol.itr.xx));