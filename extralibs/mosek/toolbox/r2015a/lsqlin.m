function [x,resnorm,residual,exitflag,output,lambda]=lsqlin(C,d,A,b,B,c,l,u,x0,options)
% Purpose : Solves the problem                   
%   
%            minimize     0.5*||C*x-d||^2
%            subject to         A*x          <= b 
%                               B*x           = c
%                            l <= x <= u 
%
%           The procedure is intended to be compatible with the function of
%           of the same name which is a part of the MATLAB optimization 
%           toolbox.
%
% Examples:
%            
%           x = lsqlin(C,d);     
%           Solves a problem without any constraints.  
%
%           x = lsqlin(C,d,A,b)
%           Solves a problem having linear equalities. 
%            
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

defaultopt = mskoptimset;

if ( nargin == 1 & nargout <= 1 & isequal(C,'defaults') )
   x = mskoptimset;
   return
end

% Handle missing arguments
if ( nargin < 10 )
   options = [];
end
if ( nargin < 9 )
   x0 = []; 
end   
if ( nargin < 8 )
  u = []; 
end
if ( nargin < 7 ) 
   l = []; 
end;
if  ( nargin < 6 )
   c = [];
end  
if ( nargin < 5 )
   B = [];
end   
if ( nargin < 4 )
   b = [];
end;   
if ( nargin<3 )
   A = [];
end   
if ( nargin<2 )
   exitflag = -1;
   output   = []; 
   x        = x0; 
   resnorm  = []; 
   residual = [];
   lambda   = [];
   mskerrmsg('lsqlin','Too few input arguments. At least 2 are required.');
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
[m,n]       = size(C);
[r,b,c,l,u] = mskcheck('lsqlin',verb,n,size(A),b,size(B),c,l,u);
if ( r~=0 )
   exitflag = r;
   output   = []; 
   x        = x0; 
   resnorm  = []; 
   residual = [];
   lambda   = [];
   return;
end   
[ma,na]     = size(A);
[mb,nb]     = size(B);
d           = d(:);
prob        = [];  
   
prob.qosubi = n+(1:m);
prob.qosubj = n+(1:m);
prob.qoval  = ones(m,1); 
prob.a      = [[A;B;C],[sparse(ma+mb,m);-speye(m,m)]];
prob.blc    = [-inf*ones(ma,1);c;d];
prob.buc    = [b;c;d];
prob.blx    = [l;-inf*ones(m,1)];
prob.bux    = [u;inf*ones(m,1)];

clear A b B c l u;

[rcode,res] = mosekopt(cmd,prob,param);

mskstatus('lsqlin',verb,0,rcode,res);

if ( isfield(res,'sol') )
  x = res.sol.itr.xx(1:n);
else
  x = []; 
end   

if nargout>5
  if ( isfield(res,'sol') )
	 lambda.lower   = res.sol.itr.slx(1:n);
	 lambda.upper   = res.sol.itr.sux(1:n);
	 lambda.ineqlin = res.sol.itr.suc(1:ma);
	 lambda.eqlin   = res.sol.itr.slc(ma+(1:mb))-res.sol.itr.suc(ma+(1:mb));
  else
	 lambda = [];
  end
end
 
if ( nargout>1 )
   resnorm = norm(C*x-d)^2; 
end
if ( nargout>2 )
   residual = C*x-d;
end   
if ( nargout>3 )
   exitflag = mskeflag(rcode,res); 
end
if ( nargout>4 )
   output = mskoutput(res);
end

