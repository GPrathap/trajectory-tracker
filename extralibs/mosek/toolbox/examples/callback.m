%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      callback.m
%
%  Purpose :   To demonstrate how to use the progress
%              callback.

% Example of how to attach the callback function
function callback(arg1, arg2)

prob.c = [ 1 2 0]';
subi   = [1 2 2 1];
subj   = [1 1 2 3];
valij  = [1.0 1.0 1.0 1.0];
prob.a = sparse(subi,subj,valij);
prob.blc  = [4.0 1.0]';
prob.buc  = [6.0 inf]';
prob.blx  = sparse(3,1);
prob.bux = []; 

% Define user defined handle.
[r,res]             = mosekopt('echo(0) symbcon');                 
data.maxtime        = 100.0;
data.symbcon        = res.symbcon;

callback.iter       = 'callback_handler';	% Defined in callback_handler.m
callback.iterhandle = data;

% Perform the optimization.
[r,res] = mosekopt('minimize echo(0)',prob,[],callback); 