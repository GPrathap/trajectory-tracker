function [res] = msklpopt(c,a,blc,buc,blx,bux,param,cmd)
%
% Syntax:      [res] = msklpopt(c,a,blc,buc,blx,bux,param,cmd)
%
% Purpose:     Solves the optimization problem
%          
%                min c'*x
%                st. blc <= a*x <= buc
%                    blx <= x   <= bux
%
% Description: Required arguments.
%                c      Is a vector.
%                a      Is a (preferably sparse) matrix.   
%
%              Optional arguments.
%                blc    Lower bounds on constraints. 
%                buc    Upper bounds on constraints. 
%                blx    Lower bounds on variables.
%                bux    Upper bounds on variables.
%                param  New MOSEK parameters.
%                cmd    MOSEK commands. 
%
%              blc=[] and buc=[] means that the 
%              lower and upper bounds are plus and minus infinity
%              respectively. The same interpretation is used for
%              blx and bux. Note -inf is allowed in blc and blx.
%              Similarly, inf is allowed in buc and bux. 
%
% See also:  MOSEKOPT, MSKQPOPT
%

%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

prob   = [];
prob.c = c;
if issparse(a)
  prob.a = a;
else
  prob.a = sparse(a);
end

if nargin>2
  prob.blc = blc;
end

if nargin>3
  prob.buc = buc;
end

if nargin>4
  prob.blx = blx;
end

if nargin>5
  prob.bux = bux;
end

if nargin<7
  param=[];
end

if nargin<8
   cmd = 'minimize';
end   

[rcode,res] = mosekopt(cmd,prob,param);

res.rcode   = rcode;
