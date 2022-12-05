%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      lo3.m
%
%  Purpose : Demonstrates a simple linear problem solved with MOSEK's linprog.
%
f     = - [3 1 5 1]';				% minus because we maximize
A     = [[-2 -1 -3 -1]; [0 2 0 3]];
b     = [-15  25]';
Aeq   = [3 1 2 0];
beq   = 30;
l     = zeros(4,1);
u     = [inf 10 inf inf]';

% Example of setting options for linprog
% Get default options
opt = mskoptimset('');
% Turn on diagnostic output
opt = mskoptimset(opt,'Diagnostics','on');
% Set a MOSEK option, in this case turn basic identification off.
opt = mskoptimset(opt,'MSK_IPAR_INTPNT_BASIS','MSK_OFF');
% Modify a MOSEK parameter with double value
opt = mskoptimset(opt,'MSK_DPAR_INTPNT_TOL_INFEAS',1e-12);

[x,fval,exitflag,output,lambda] = linprog(f,A,b,Aeq,beq,l,u,opt);

x
fval
exitflag
output
lambda
