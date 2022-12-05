function [x,fval,exitflag,output,lambda]=linprog(f,A,b,B,c,l,u,x0ignored,options)
%
% Syntax  : [x,fval,exitflag,output,lambda]=linprog(f,A,b,B,c,l,u,options)
%
%
% Purpose : Solves the problem                   
%   
%             min      f'*x    
%             st.      A*x    <= b 
%                      B*x     = c
%                   l <= x <= u 
%
%           The procedure is intended to be compatible with the function of
%           of the same name which is a part of the MATLAB optimization 
%           toolbox.
%
% Examples: The command line
%            
%             x = linprog(f,A,b,[],[],l); 
%
%           solves the problem 
%           
%            min      f'*x    
%            st.      A*x    <= b 
%                     l <= x 
%
%           Options for this function may be set with the mskoptimset
%           function. E.g, 
%
%           options = mskoptimset('');
%           options = mskoptimset(options,'Diagnostics','on');
%           [x] = linprog(f,A,b,B,c,l,u,options)
% 
%           Will make MOSEK print diagnostics messages to the
%           screen. 
%
% See also: MSKOPTIMSET, MSKLPOPT, MOSEKOPT

%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

defaultopt = mskoptimset;

if ( nargin == 1)
   prob = f;

   if isequal(f,'defaults')
       x = defaultopt;
       return
   elseif ( ~isstruct(prob) ) 
      exitflag = -1;
      output   = []; 
      fval     = []; 
      lambda   = [];
      x        = [];
      mskerrmsg('linprog','Invalid input argument; problem must be a structure.');
      return; 
   end

   if ( ~all(isfield(prob, {'f','A','b'})) )
      exitflag = -1;
      output   = []; 
      fval     = []; 
      lambda   = [];
      mskerrmsg('linprog','problem must contain at least "f", "A" and "b".');
      return; 
   end

   f = prob.f;
   A = prob.A;
   b = prob.b;

   if (isfield(prob, 'options'))
      options = prob.options;
   else
      options = [];
   end

   if (isfield(prob, 'u'))
      u = prob.options;
   else
      u = [];
   end

   if (isfield(prob, 'l'))
      l = prob.options;
   else
      l = [];
   end

   if (isfield(prob, 'c'))
      c = prob.options;
   else
      c = [];
   end

   if (isfield(prob, 'B'))
      B = prob.options;
   else
      B = [];
   end

else

   % Handle missing arguments
   if ( nargin == 8 )
      if ( isstruct(x0ignored) )
        options = x0ignored;
      else
        options = [];
      end;
   end;
   if ( nargin < 8 )
      options = [];
   end;   
   if ( nargin < 7 )
     u = []; 
   end
   if ( nargin < 6 )
      l = []; 
   end
   if ( nargin < 5 )
      c = [];
   end   
   if ( nargin < 4 )
      B = [];
   end   

   if ( nargin<3 )
      exitflag = -1;
      output   = []; 
      fval     = []; 
      lambda   = [];
      mskerrmsg('linprog','Too few input arguments. At least 3 are required.');
      return;
   end   
end

options          = mskoptimset(defaultopt,options);

[cmd,verb,param] = msksetup(1,options);

n                = length(f); 
[r,b,c,l,u]      = mskcheck('linprog',verb,n,size(A),b,size(B),c,l,u);

if ( r~=0 )
   exitflag = r;
   output   = []; 
   fval     = []; 
   lambda   = [];
   return;
end   

% Setup the problem that is fed into MOSEK.
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
prob.blc    = [-inf*ones(size(b));c];
prob.buc    = [b;c];
prob.blx    = l;
prob.bux    = u;

clear f A b B c l u options;
[rcode,res] = mosekopt(cmd,prob,param);

mskstatus('linprog',verb,0,rcode,res);
 
if ( isfield(res,'sol') )
  if ( isfield(res.sol,'itr') )
    x = res.sol.itr.xx;
  else
    x = res.sol.bas.xx;
  end
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
   output = mskoutput(res);
end

if nargout>4
   if ( isfield(res,'sol') )
      if ( isfield(res.sol,'itr') )
        lambda.lower   = res.sol.itr.slx;
        lambda.upper   = res.sol.itr.sux;
        lambda.ineqlin = -res.sol.itr.y(1:numineq);
        lambda.eqlin   = -res.sol.itr.y((numineq+1):end);
      else
        lambda.lower   = res.sol.bas.slx;
        lambda.upper   = res.sol.bas.sux;
        lambda.ineqlin = -res.sol.bas.y(1:numineq);
        lambda.eqlin   = -res.sol.bas.y((numineq+1):end);
      end
   else
      lambda = [];
   end
end

