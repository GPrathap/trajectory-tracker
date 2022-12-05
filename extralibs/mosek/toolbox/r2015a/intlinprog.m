function [x,fval,exitflag,output] = intlinprog(f,intcon,A,b,B,c,l,u,options)
%intlinprog Integer linear programming.
%   intlinprog solves the integer linear programming problem
%
%       min f'*x  subject to:  A*x <= b,
%                              B*x  = c,
%                              l <= x <= u,
%                              x(intcon) are integers.
%
%   x = intlinprog(f,intcon,A,b) solves the problem min f'*x subject to the linear 
%   inequalities A*x <= b, where the elements x(intcon) are integers.
%
%   x = bintprog(f,A,b,B,c) solves the problem min f'*x subject to the
%   linear equalities B*x = c and the linear inequalities A*x <= b, where 
%   the elements x(intcon) are integers.
%
%   x = intlinprog(f,intcon,A,b,B,c,l,u) solves the problem min f'*x subject to the
%   linear equalities B*x = c, the linear inequalities A*x <= b, where l <= x <= u and 
%   the elements x(intcon) are integers.
%
%   x = intlinprog(f,intcon,A,b,B,c,l,u,options) solves the problem with the default 
%   optimization parameters replaced by values in the structure options, an
%   argument created with the mskoptimset function.  See mskoptimset for details.
%   Available options are AbsoluteGapTolerance, RelativeGapTolerance, MaxTime
%   Display and MaxNodes.  Other options supported by Matlab are currently ignored.
%    
%   x = intlinprog(problem) encapsulates the problem parameters in a structure 
%   with the fields "f", "A" and "b", and optionally the
%   fields "B", "c", "l", "u" and "options".
%
%   [x,fval,exitflag,output] = bintprog(...) also return the objective value,
%   an exitflag, as well as output from the solver. Possible values of 
%   exitflag and the corresponding exit conditions are
%
%     2    intlinprog stopped prematurely, but found a feasible x.
%     1    intlinprog converged to the solution x.
%     0    intlinprog stopped prematurely, and found no feasible x.
%    -2    No feasible point found.
%    -3    Root LP problem is unbounded.
%
%   Internally bintprog() is a wrapper for the mosekopt() function.
%
%   Example
%     f = [8;1];
%     intcon = 2;
%     A = [-1,-2;-4,-1;2,1];
%     b = [14;-33;20];
%     x = intlinprog(f,intcon,A,b);
%
%   The function is intended to be compatible with the function of
%   of the same name which is a part of the MATLAB optimization 
%   toolbox.
%
%
%  See also linprog, mosekopt.
%
%  

defaultopt = mskoptimset;

if ( nargin == 1)
   prob = f;
   if ( ~isstruct(prob) ) 
      exitflag = -1;
      output   = []; 
      fval     = []; 
      x        = [];
      mskerrmsg('intlinprog','Invalid input argument; problem must be a structure.');
      return; 
   end

   if ( ~all(isfield(prob, {'f','intcon','A','b'})) )
      exitflag = -1;
      output   = []; 
      fval     = []; 
      mskerrmsg('intlinprog','problem must contain at least "f", "intcon", A" and "b".');
      return; 
   end

   f      = prob.f;
   intcon = prob.intcon;
   A      = prob.A;
   b      = prob.b;

   if (isfield(prob, 'options'))
      options = prob.options;
   else
      options = struct();
   end

   if (isfield(prob, 'c'))
      c = prob.c;
   else
      c = [];
   end

   if (isfield(prob, 'B'))
      B = prob.B;
   else
      B = [];
   end

else
 % x = intlinprog(f,intcon,A,b,B,c,l,u,options)
 
   % Handle missing arguments
   if ( nargin < 9 )
      options = struct();
   end;   
   if ( nargin < 8 )
      u = [];
   end   
   if ( nargin < 7 )
      l = [];
   end   
   if ( nargin < 6 )
      c = [];
   end   
   if ( nargin < 5 )
      B = [];
   end   

   if ( nargin<4 )
      exitflag = -1;
      output   = []; 
      x        = []; 
      fval     = []; 
      mskerrmsg('intlinprog','Too few input arguments. At least 4 are required.');
      return;
   end   
end

options          = mskoptimset(defaultopt,options)

[cmd,verb,param] = msksetup(1,options);

n                = length(f); 
[r,b,c,l,u]      = mskcheck('intlinprog',verb,n,size(A),b,size(B),c,l,u);

if ( r~=0 )
   exitflag = r;
   output   = []; 
   x        = []; 
   fval     = []; 
   return;
end   

% Setup the problem that is feed into MOSEK.
prob        = [];
[numineq,t] = size(A);
[numeq,t]   = size(B);
prob.c      = reshape(f,n,1);
prob.a      = [A;B];
if ( isempty(prob.a) )
   prob.a = sparse(0,length(f));
elseif ~issparse(prob.a)
   prob.a = sparse(prob.a);
end   
prob.b      = b;
prob.blc    = [-inf*ones(size(b));c];
prob.buc    = [b;c];
prob.blx    = l;
prob.bux    = u;

prob.ints.sub    = intcon;

clear f intcon A b B c l u options;

[rcode,res] = mosekopt(cmd,prob,param);

mskstatus('intlinprog',verb,0,rcode,res);
 
if ( isfield(res,'sol') )
  x = res.sol.int.xx;
else
  x = [];
end

if nargout>1 & length(prob.c) == length(x)
   fval = prob.c'*x; 
else
  fval = [];
end

if nargout>2
   exitflag = mskeflag(rcode,res); 
end

if nargout>3
   output = res;
end


