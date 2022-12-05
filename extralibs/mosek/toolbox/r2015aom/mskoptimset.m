function options = mskoptimset(varargin)
%
% Syntax:
%
% options = mskoptimset('param1',value1,'param2',value2,...)
% mskoptimset
% options = mskoptimset
%
%
% Purpose: Is used to create and modify optimization
%          options structure.  
%
% Examples:
%           options = mskoptimset('');
%           options = mskoptimset(options,'Diagnostics','on');
% 
%           Turns on diagnostics messages to the screen.
%            
% See also: MSKOPTIMGET, LINPROG, QUADPROG
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

if ( nargin == 0 ) 
   if ( nargout == 0 ) 
     fprintf(' Options. {} means default value.\n') 
     fprintf(' Diagnostics          : [ on | {off} ]\n');
     fprintf(' Display              : [ {off} | iter | final ]\n');
     fprintf(' MaxIter              : [ positive scalar ]\n');
     fprintf(' Simplex              : [ '''' | on | primal | dual]\n');
     fprintf(' MaxTime              : [ positive scalar ]\n');
     fprintf(' AbsoluteGapTolerance : [ positive scalar ]\n');
     fprintf(' RelativeGapTolerance : [ positive scalar ]\n');
     fprintf(' MaxNodes             : [ positive integer ]\n');
     fprintf(' Write                : [ '''' | {FILENAME} ]\n');
     fprintf('\n');
   else
     [rcode,res]         = mosekopt('param echo(0)');  
     options.Write       = '';
     options.Diagnostics = 'off';
     options.Display     = 'off';
     options.MaxIter     = res.param.MSK_IPAR_INTPNT_MAX_ITERATIONS;     
     options.Simplex     = '';
   end   
   return;
end

% Ask for default options for a function.
defopt  = mskoptimset;
options = defopt;

if ( ( nargin==1 ) & ischar(varargin{1}) )
   return
end

%optnames  = fieldnames(options);
%loptnames = lower(optnames);          
%[m,n]     = size(loptnames);

i         = 1;
while ( i <= nargin )
   arg = varargin{i};
   if isstr(arg)                       
      % Starting at name-value pairs.
      break;
   end
   if ~isempty(arg)                     
      if ~isa(arg,'struct')
         error(sprintf(['Expected argument %d to be a string parameter name ' ...
               'or an options structure\ncreated with mskoptimset.'], i));
      end
      names   = fieldnames(arg);
      [nm,nn] = size(names);
      for j=1:nm
         nam          = char(names(j));
         [rcode,onam] = mskoptnam(nam,defopt);
         if ( rcode==1 )
            val = getfield(arg,nam);
         else
            val = [];
         end
         if ( ~isempty(val) )
            options = setfield(options,onam,val);
         end
      end
   end
   i = i + 1;
end

% handle the name-value pairs.
if rem(nargin-i+1,2) ~= 0
   error('Arguments must occur in name-value pairs.');
end

expectval = 0;                        
while ( i <= nargin )
   arg = varargin{i};
   
   if ~expectval
      if ~isstr(arg)
         error(sprintf('Expected argument %d to be a string parameter name.', i));
      end
      
      [rcode,onam] = mskoptnam(arg,defopt);
      if ( rcode==0 )
         expectval = 2;
      elseif ( rcode==1 )
         expectval = 1;
      elseif ( rcode==2 )
         error(sprintf('Ambiguous parameter name ''%s'' ', arg));
      else
         error('Unexpected return code'); 
      end
   else
      if ( expectval==1 )
        options = setfield(options,onam,arg);
      end  
      expectval = 0;
   end
   i = i + 1;
end

if expectval
   error(sprintf('Expected value for parameter ''%s''.', arg));
end


