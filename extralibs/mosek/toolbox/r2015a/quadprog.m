function [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
%
% Syntax  : [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
% Purpose : Solves the problem                   
%   
%             minimize     0.5*x'*H*x+f'*x    
%             subject to         A*x          <= b 
%                                B*x           = c
%                             l <= x <= u 
%
%             The procedure is intended to be compatible with the function of
%             of the same name which is a part of the MATLAB optimization 
%             toolbox.
%
%             To set options for this function use the optimset
%             function.
%
% Examples: The command line
%         
%             x = quadprog(H,f,A,b,[],[]);
%
%           solves a problem without any equalities and bounds. 
%
% Se also:  MSKOPTIMSET, MSKQPOPT, MOSEKOPT

%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.


defaultopt = mskoptimset;

if ( nargin == 1 & nargout <= 1 & isequal(H,'defaults') )
   x = mskoptimset;
   return
end

% Handle missing arguments
if ( nargin < 10 )
   options = [];
end;   
if ( nargin < 9 )
   x0 = []; 
end   
if ( nargin < 8 )
  u = []; 
end
if ( nargin < 7 )
   l = []; 
end;   
if ( nargin < 6 )
   c = [];
end   
if ( nargin < 5 ) 
   B = [];
end;   
if ( nargin<4 )
   exitflag = -1;
   output   = []; 
   x        = x0; 
   fval     = []; 
   lambda   = [];
   mskerrmsg('quadprog','Too few input arguments. At least 4 are required.');
   return;
end   

if isempty(A)
  A=[];
end
if isempty(b)
  b=[];
end
if isempty(B)
  B=[];
end
if isempty(c)
  c=[];
end

options          = mskoptimset(defaultopt,options);
[cmd,verb,param] = msksetup(1,options);

% Setup the problem to feed to MOSEK.
prob   = [];
n      = length(f);
prob.c = f(:);
hsze   = size(H);
if ( ~isempty(H) )
   if ( hsze(1)~=hsze(2) | n~=hsze(1) )
      exitflag = -1;
      output   = []; 
      x        = x0; 
      fval     = []; 
      lambda   = [];
      mskerrmsg('quadprog','H is not a square matrix of dimension length(f)');
      return;
   end;
   [prob.qosubi,prob.qosubj,prob.qoval] = find(sparse(tril(H)));
end;

[r,b,c,l,u] = mskcheck('quadprog',verb,n,size(A),b,size(B),c,l,u);
if ( r~=0 )
   exitflag = r;
   output   = []; 
   x        = x0; 
   fval     = []; 
   lambda   = [];
   return;
end   
[numineq,t] = size(A);
[numeq,t]   = size(B);
prob.a      = [A;B];
if ( isempty(prob.a) )
   prob.a = sparse(0,n);
elseif ~issparse(prob.a)
   prob.a = sparse(prob.a);
end   
prob.blc    = [-inf*ones(size(b));c];
prob.buc    = [b;c];
prob.blx    = l;
prob.bux    = u;

clear f A b B c l u x0 options;

[rcode,res] = mosekopt(cmd,prob,param);

mskstatus('quadprog',verb,0,rcode,res);

if ( isfield(res,'sol') )
   x = res.sol.itr.xx;

   if nargout>1 
      fval = 0.5*x'*H*x+prob.c'*x; 
   end
else
   x = [];

   if nargout>1 
      fval = []; 
   end
end   

if nargout>2
   exitflag = mskeflag(rcode,res); 
end
if nargout>3
   output = mskoutput(res);
end
if nargout>4
   if ( isfield(res,'sol') )
      lambda.lower   = res.sol.itr.slx;
      lambda.upper   = res.sol.itr.sux;
      lambda.ineqlin = -res.sol.itr.y(1:numineq);
      lambda.eqlin   = -res.sol.itr.y((numineq+1):end);
   else
      lambda = [];
   end
end

