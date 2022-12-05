function [res] = mskqpopt(q,c,a,blc,buc,blx,bux,param,cmd)
%
% Syntax     : [res] = mskqpopt(q,c,a,blc,buc,blx,bux,param,cmd)  
%
% Purpose    : Solves the optimization problem
%          
%                min 0.5*x'*q*x+c'*x
%                st. blc <= a*x <= buc
%                    blx <= x   <= bux
%
% Description: Required arguments.
%                q      It is assumed that q is a symmetric matrix.
%                       q is required to be POSITIVE SEMI-DEFINITE. 
%                c      Is a vector. 
%                a      Is a (preferably) sparse matrix.  
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
%              lower and upper bounds are plus and minus infinite
%              respectively. The same interpretation is used for
%              blx and bux. Note -inf is allowed in blc and blx.
%              Similarly, inf is allowed in buc and bux.  
%
% Se also:  MOSEKOPT, MSKLPOPT

%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

prob  = [];

[m,n] = size(a);

if ~isempty(q)
   dim = size(q); 
   if n~=dim(1) | n~=dim(2)
      error('q is of inappropriate dimension');
   end   
   prob.qosubi = [];
   prob.qosubj = [];
   prob.qoval  = [];
   [prob.qosubi,prob.qosubj,prob.qoval] = find(tril(sparse(q)));    
end   

prob.c = c;
if issparse(a)
  prob.a = a;
else
  prob.a = sparse(a);
end

if nargin>3
  prob.blc = blc;
end

if nargin>4
  prob.buc = buc;
end

if nargin>5
  prob.blx = blx;
end

if nargin>6
  prob.bux = bux;
end

if nargin<8
  param=[];
end

if nargin<9
   cmd = 'minimize';
end  

[rcode,res] = mosekopt(cmd,prob,param);
