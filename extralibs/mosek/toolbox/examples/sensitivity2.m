%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      sensitivity2.m
%

function sensitivity2()
% Setup problem data.

prob.a = sparse([1,    1,    0,    0,    0,    0,    0;
                 0,    0,    1,    1,    0,    0,    0;
                 0,    0,    0,    0,    1,    1,    1;
                 1,    0,    0,    0,    1,    0,    0;
                 0,    1,    0,    0,    0,    0,    0;
                 0,    0,    1,    0,    0,    1,    0;
                 0,    0,    0,    1,    0,    0,    1]);

prob.c =  [1,2,5,2,1,2,1];
prob.blc = [-Inf,-Inf,-Inf,800,100,500, 500];
prob.buc =[400,1200,1000,800,100,500,500];
prob.bux(1:7) = Inf; 
prob.blx(1:7) = 0;

% Analyze upper bound of constraint 1.
prob.prisen.cons.subu = [1];  

[r,res] = mosekopt('minimize echo(0)',prob); 
fprintf ('Optimal objective value: %e\n',prob.c * res.sol.bas.xx  );
fprintf('Sensitivity results for constraint 1:');
res.prisen.cons

% If we change the upper bound of constraint 1 with a
% value v in [res.prisen.cons.lr_bu(1),res.prisen.cons.rr_bu(1)] 
% then the optimal objective changes with - v * ls_bu(0) 
% e.g. changing prob.buc(1) with -1
prob.buc(1) =  prob.buc(1) - 1;
new_sol_predicted = prob.c * res.sol.bas.xx  + 1 * res.prisen.cons.ls_bu(1);
fprintf ('New optimal objective after changing bound predicted to:%e\n', ...
         new_sol_predicted);
[r,res] = mosekopt('minimize echo(0)',prob); 
fprintf ('New optimal objective value: %e\n',prob.c * res.sol.bas.xx  );