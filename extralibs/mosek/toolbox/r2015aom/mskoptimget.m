function o = mskoptimget(options,name,default,flag)
% Purpose: Obtains the value of an optimization option.
%
% See also: MSKOPTIMSET
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

if nargin < 2
  error('Not enough input arguments.');
end
if nargin < 3
  default = [];
end

if ( ~isempty(options) & ~isa(options,'struct') )
  error('First argument must be an options structure created with mskoptimset.');
end

if isempty(options)
  o = default;
  return;
end

[rcode,onam] = mskoptnam(name);

if ( rcode==0 )
   % Do nothing, because MOSEK does not recoginize most MATLAB
   % optimization options.
   o = default;
elseif ( rcode==1 )
   o = getfield(options,onam);
else      
   error(sprintf('%s is an ambigous optimization option name.',name));
end

