function [r,res] = mosekopt(cmd,prob,param,callback)
%
% Syntax     : [r,res] = mosekopt(cmd,prob,param,callback)
%
% Purpose    : Interface to the MOSEK optimization tools.
%          
% Description: 
%
% Required arguments:
%   cmd       A command string. E.g 'minimize'.
%   prob      The optimization problem.   
%
% Optional arguments.
%   param     MOSEK parameters.
%   callback  A callback function.
%
% Please see "The MOSEK optimization toolbox manual" for a detailed
% description of MOSEKOPT.
%
% After a call to MOSEKOPT the argument r holds the return code and
% res holds the result of the optimization. In case of an error the
% files res.rmsg contains an error message and r.rcodestr holds the
% symbolic name of the error code. 
%
% The first option cmd is a string which consists of one or more of
% the following commands:
%
% * echo()
% Controls how much information is echoed to the screen. The syntax of
% the command is
%
% echo(level)
%
% where level is a non-negative integer. If level is identical to 0
% nothing is echoed. If level is equal to 3, then all messages and
% errors are echoed to the screen.
%
% * read()
% Data is read from a file. The command
%
% read(name)
%
% results in reading the file 'name'.  The file format is
% determined by the extension. (e.g .mps, .lp, .opf or .mbt). 
%
% * statuskeys()
% The command statuskeys(0) means that all the status keys such as
% the problem status in the solution is reported using string
% codes. Whereas the command statuskeys(1) means that all the
% status keys are reported using numeric codes.
%
% * minimize
% Run the optimizer, minimize the objective. 
%
% * maximize
% Run the optimizer, maximize the objective.
%
% * write()
% The problem data is written to a file. The file format is
% determined by the file extension (e.g .mps, .lp, .opf or .mbt). The command 
%               
% write(name)
% 
% writes the problem given in prob to the file 'name'.
%
% * param
% When this command is present, the parameter database is returned in res.param.
%
% * info
% When this command is present,  the task information database is
% returned in res.info. This database contains various task specific information.
%
% * symbcon
% When this command is present, then then the data structure
% symbcon is returned in res.symbcon.
%
% Examples:
%
% - Run the optimizer and minimize objective function of problem
% given in prob:
% [r,res] = mosekopt('minimize',prob)
%
% - Read (but do not optimize) the problem file afiro.mps:
%
% [r,res] = mosekopt('read(afiro.mps)')
%
% After the call the  problem is stored in res.prob.
% 
% See also: MSKLPOPT, MSKQPOPT, MSKSCOPT, MSKGPOPT
%
% Note:     The file mosekopt.m is only a help file and does
%           not contain any code. 
%

%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

fprintf('MOSEK ERROR. There is something wrong with MOSEK installation\n');
fprintf('             Please see the trouble shooting section the toolbox manual.\n');

