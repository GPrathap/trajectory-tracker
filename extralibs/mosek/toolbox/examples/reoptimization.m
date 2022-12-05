%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      reoptimization.m
%
%  Purpose : Demonstrates how to modify and re-optimize a linear problem
%

function reoptimization()
clear prob;

% Specify the c vector.
prob.c  = [1.5 2.5 3.0]';

% Specify a in sparse format.
subi   = [1 1 1 2 2 2 3 3 3];
subj   = [1 2 3 1 2 3 1 2 3];
valij  = [2 4 3 3 2 3 2 3 2];

prob.a = sparse(subi,subj,valij);

% Specify lower bounds of the constraints.
prob.blc = [-inf -inf -inf]';

% Specify  upper bounds of the constraints.
prob.buc = [100000 50000 60000]';

% Specify lower bounds of the variables.
prob.blx = zeros(3,1);

% Specify upper bounds of the variables.
prob.bux = [inf inf inf]';

% Perform the optimization.
param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_FREE_SIMPLEX'; 
[r,res] = mosekopt('maximize',prob,param); 

% Show the optimal x solution.
res.sol.bas.xx

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Change a coefficient
prob.a(1,1) = 3.0;

% Reoptimize with changed coefficient
% Use previous solution to perform very simple hot-start.
% This part can be skipped, but then the optimizer will start
% from scratch on the new problem, i.e. without any hot-start.
prob.sol = [];
prob.sol.bas = res.sol.bas;			
[r,res] = mosekopt('maximize',prob,param);
res.sol.bas.xx

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add a variable
prob.c       = [prob.c;1.0];
prob.a       = [prob.a,sparse([4.0 0.0 1.0]')];
prob.blx     = [prob.blx; 0.0];
prob.bux     = [prob.bux; inf];

% Reoptimize with a new variable and hot-start
% All parts of the solution must be extended to the new dimensions.
prob.sol = [];
prob.sol.bas = res.sol.bas;			
prob.sol.bas.xx  = [prob.sol.bas.xx; 0.0];
prob.sol.bas.slx = [prob.sol.bas.slx; 0.0];
prob.sol.bas.sux = [prob.sol.bas.sux; 0.0];
prob.sol.bas.skx = [prob.sol.bas.skx; 'UN'];
[r,res] = mosekopt('maximize',prob,param);
res.sol.bas.xx

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add a constraint
prob.a       = [prob.a;sparse([1.0 2.0 1.0 1.0])]; 
prob.blc     = [prob.blc; -inf]; 
prob.buc     = [prob.buc; 30000.0]; 

% Reoptimize with a new variable and hot-start
prob.sol = [];
prob.sol.bas = res.sol.bas;			
prob.sol.bas.y   = [prob.sol.bas.y; 0.0];
prob.sol.bas.xc  = [prob.sol.bas.xc; 0.0];
prob.sol.bas.slc = [prob.sol.bas.slc; 0.0];
prob.sol.bas.suc = [prob.sol.bas.suc; 0.0];
prob.sol.bas.skc = [prob.sol.bas.skc; 'UN'];
[r,res] = mosekopt('maximize',prob,param);
res.sol.bas.xx


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Change constraint bounds
prob.buc     = [80000 40000 50000 22000]';
prob.sol     = res.sol;
[r,res] = mosekopt('maximize',prob,param);
res.sol.bas.xx