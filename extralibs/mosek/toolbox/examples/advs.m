%%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      advs.m
%
%  Demonstrates hot-start capabilities for the simplex optimizers.
%%

clear prob param bas

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Initial problem (with hotstart) %%%%%%%
% Specify an initial basic solution.
bas.skc      = ['LL';'LL'];
bas.skx      = ['BS';'LL';'BS'];
bas.xc       = [4 1]';
bas.xx       = [1 3 0]';

prob.sol.bas = bas;

% Specify the problem data.
prob.c       = [ 1 2 0]';
subi         = [1 2 2 1];
subj         = [1 1 2 3];
valij        = [1.0 1.0 1.0 1.0];
prob.a       = sparse(subi,subj,valij);
prob.blc     = [4.0 1.0]';
prob.buc     = [6.0 inf]';
prob.blx     = sparse(3,1);
prob.bux     = [];

% Use the primal simplex optimizer.
param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_PRIMAL_SIMPLEX';
[r,res] = mosekopt('minimize',prob,param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Add a variable %%%%%%%%%%%%%%%%%
prob.c       = [prob.c;-1.0];
prob.a       = [prob.a,sparse([1.0 0.0]')];
prob.blx     = sparse(4,1);

% Reuse the old optimal basic solution.
bas          = res.sol.bas;

% Add to the status key.
bas.skx      = [res.sol.bas.skx;'LL'];

% The new variable is at it lower bound.
bas.xx       = [res.sol.bas.xx;0.0];
bas.slx      = [res.sol.bas.slx;0.0];
bas.sux      = [res.sol.bas.sux;0.0];

prob.sol.bas = bas;

[rcode,res]  = mosekopt('minimize',prob,param);

% The new primal optimal solution
res.sol.bas.xx'


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Fixing a variable %%%%%%%%%%%%%%%%%%
prob.blx(4)  = 1;
prob.bux     = [inf inf inf 1]';

% Reuse the basis.
prob.sol.bas = res.sol.bas;

[rcode,res]  = mosekopt('minimize',prob,param);

% Display the optimal solution.
res.sol.bas.xx'


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Add a constraint %%%%%%%%%%%%%%%%%%
% Modify the problem.
prob.a       = [prob.a;sparse([1.0 1.0 0.0 0.0])];
prob.blc     = [prob.blc;2.0];
prob.buc     = [prob.buc;inf];

% Obtain the previous optimal basis.
bas          = res.sol.bas;

% Set the solution to the modified problem.
bas.skc      = [bas.skc;'BS'];
bas.xc       = [bas.xc;bas.xx(1)+bas.xx(2)];
bas.y        = [bas.y;0.0];
bas.slc      = [bas.slc;0.0];
bas.suc      = [bas.suc;0.0];

% Reuse the basis.
prob.sol.bas = bas;

% Reoptimize.
[rcode,res]  = mosekopt('minimize',prob,param);

res.sol.bas.xx'


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Remove a constraint %%%%%%%%%%%%%%%%%
% Modify the problem.
prob.a       = prob.a(1:end-1,:);
prob.blc     = prob.blc(1:end-1);
prob.buc     = prob.buc(1:end-1);

% Obtain the previous optimal basis.
bas          = res.sol.bas;

% Set the solution to the modified problem.
bas.skc      = bas.skc(1:end-1,:);
bas.xc       = bas.xc(1:end-1);
bas.y        = bas.y(1:end-1);
bas.slc      = bas.slc(1:end-1);
bas.suc      = bas.suc(1:end-1);

% Reuse the basis.
prob.sol.bas = bas;

% Reoptimize.
[rcode,res]  = mosekopt('minimize',prob,param);

res.sol.bas.xx'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Remove a variable %%%%%%%%%%%%%%%%
% Modify the problem.
prob.c       = prob.c(1:end-1);
prob.a       = prob.a(:,1:end-1);
prob.blx     = prob.blx(1:end-1);
prob.bux     = prob.bux(1:end-1);

% Obtain the previous optimal basis.
bas          = res.sol.bas;

% Set the solution to the modified problem.
bas.xx       = bas.xx(1:end-1);
bas.skx      = bas.skx(1:end-1,:);
bas.slx      = bas.slx(1:end-1);
bas.sux      = bas.sux(1:end-1);

% Reuse the basis.
prob.sol.bas = bas;

% Reoptimize.
[rcode,res]  = mosekopt('minimize',prob,param);

res.sol.bas.xx'