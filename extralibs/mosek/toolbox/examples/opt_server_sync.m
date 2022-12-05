%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      opt_server_sync.m
%
%  Purpose : Demonstrates solving a problem remotely using the OptServer.
%
%

function opt_server_sync(inputfile, addr)
clear prob;
clear param;
clear optserver;

% We read some problem from a file or set it up here
cmd = sprintf('read(%s)', inputfile);
[r,res] = mosekopt(cmd);
prob = res.prob;

% OptServer data
% "host" should be 'http://server:port`
optserver.host = addr;

% Perform the optimization with full log output.
[r,res] = mosekopt(sprintf('%s echo(10)', prob.objsense), prob, [], [], optserver); 

% Use the optimal x solution.
xx = res.sol.bas.xx;