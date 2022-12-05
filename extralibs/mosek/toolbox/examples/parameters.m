%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      parameters.m
%
%  Purpose:    Demonstrates a very simple example about how to get/set
%              parameters with MOSEK Matlab Toolbox
%

function r = parameters()

fprintf('Test MOSEK parameter get/set functions\n\n');

% Set up a small problem
prob.c  = [3 1 5 1]';
subi   = [1 1 1 2 2 2 2 3 3];
subj   = [1 2 3 1 2 3 4 2 4];
valij  = [3 1 2 2 1 3 1 2 3];
prob.a = sparse(subi,subj,valij);
prob.blc = [30 15  -inf]';
prob.buc = [30 inf 25 ]';
prob.blx = zeros(4,1);
prob.bux = [inf 10 inf inf]';

% Get default parameter values
[r,resp]=mosekopt('param'); 
fprintf('Default value for parameter MSK_IPAR_LOG = %d\n', resp.param.MSK_IPAR_LOG);
    
% Set log level (integer parameter)
param.MSK_IPAR_LOG = 1;
% Select interior-point optimizer... (integer parameter)
param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_INTPNT';
% ... without basis identification (integer parameter)
param.MSK_IPAR_INTPNT_BASIS = 'MSK_BI_NEVER';
% Set relative gap tolerance (double parameter)
param.MSK_DPAR_INTPNT_CO_TOL_REL_GAP = 1.0e-7;

% Use in mosekopt
[r,resp] = mosekopt('minimize', prob, param);

% Demonstrate information items after optimization
[r,res] = mosekopt('minimize info', prob);

res.info.MSK_DINF_OPTIMIZER_TIME
res.info.MSK_IINF_INTPNT_ITER

end