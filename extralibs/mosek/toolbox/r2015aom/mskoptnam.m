function [rcode,onam] = mskoptnam(name,defopt)
% Purpose: Check if name is in the optimization options structure.
%
% Output : code - A return code which has the interpretation. 
%                 0, name does not exists
%                 1, name exists
%                 2, name is ambiguos
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

rcode   = 0;
onam    = [];

if nargin==1
  defopt  = optimset;
end  
  
optnam  = fieldnames(defopt);
loptnam = lower(optnam);
lwrnam  = deblank(lower(name));

[m,n]   = size(optnam);
j       = strmatch(lwrnam,loptnam);

if isempty(j)                       % if no matches
   if strncmp(name,'MSK_',4)        % Should check if name is a valid MOSEK name.
     onam  = char(name);
     rcode = 1;  
   end; 
elseif length(j) > 1                % if more than one match
  % Check for any exact matches 
  k = strmatch(lwrnam,loptnam,'exact');
  if length(k) == 1
    rcode = 1; 
    onam  = char(optnam(k(1)));
  else
    rcode = 2;
  end;
else 
  rcode = 1; 
  onam  = char(optnam(j));
end  

