function [varargout]=lsqnonneg(C,d,x0,options)
% Purpose: Solves the linear least squares problem
%
%          minimize   1/2||C x - d||_2^2
%          subject to        x>=0
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

[m,n] = size(C);

if ( nargin == 1 & nargout <= 1 & isequal(C,'defaults') )
   varargout{1} = mskoptimset;
   return
end

if ( nargin<4 )
   options = [];
end   

if ( nargin<3 )
   x0 = [];
end

if ( nargin<2 )
   exitflag = -1;
   output   = []; 
   x        = x0; 
   fval     = []; 
   lambda   = [];
   msgerrmsg('lsqnonneg','Too few input arguments. At least 3 are required.');
   return
end

if ( nargout~=0 )
   [x,resnorm,residual,exitflag,output,lambda] = ...
      lsqlin(C,d,[],[],[],[],sparse(n,1),[],x0,options);
   if ( nargout>0 )
      varargout(1) = {x};
   end
   if ( nargout>1 )
      varargout(2) = {resnorm};
   end
   if ( nargout>2 )
      varargout(3) = {residual};
   end
   if ( nargout>3 )
      varargout(4) = {exitflag};
   end
   if ( nargout>4 )
      varargout(5) = {output};
   end
   if ( nargout>5 )
      varargout(6) = {lambda};
   end
else
   lsqlin(C,d,[],[],[],[],sparse(n,1),[],x0,options)
end

