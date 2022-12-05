function [cmd,verbosity,param] = msksetup(domin,options)
% Purpose: Function used by the MOSEK compability toolbox.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

writefile = 0;
writename = '';

verbosity = 0;

param     = [];

echolev   = 0;
prlev     = 0;

if ~isempty(options) & isa(options,'struct') & isfield(options,'Display')
  dispstr = deblank(getfield(options,'Display'));
  if strcmp(dispstr,'off')
    echolev                           = 0;
  elseif strcmp(dispstr,'final')
    echolev                           = 3;
    verbosity                         = 1;
  elseif strcmp(dispstr,'iter')
    param.MSK_IPAR_LOG_INTPNT         = 1;
    param.MSK_IPAR_LOG_SIM            = 1;
    echolev                           = 3;
    verbosity                         = 1;
  else
    echolev                           = 0;
  end
end

if isa(options,'struct') & isfield(options,'Diagnostics')
  diagstr = lower(getfield(options,'Diagnostics'));
  if strcmp(diagstr,'on')
     fprintf('Diagnostics is on\n');
     echolev   = max(echolev,10);
     prlev     = max(prlev,1);
     verbosity = 1;
  else
     param.MSK_IPAR_MAX_NUM_WARNINGS = 0;
  end
else
   param.MSK_IPAR_MAX_NUM_WARNINGS = 0;
end

if isa(options,'struct') & isfield(options,'MaxTime')
  param.MSK_DPAR_OPTIMIZER_MAX_TIME = options.MaxTime;
end

if isa(options,'struct') & isfield(options,'MaxIter')
  param.MSK_IPAR_INTPNT_MAX_ITERATIONS = options.MaxIter;
end

if isa(options,'struct') & isfield(options,'MaxNodes')
  param.MSK_IPAR_MIO_MAX_NUM_BRANCHES = options.MaxNodes;
end

if isa(options,'struct') & isfield(options,'Write')
  writename = deblank(getfield(options,'Write'));
  writefile = 1;
end

if isa(options,'struct') & isfield(options,'Simplex')

  if isfield(options,'Diagnostics')
      diagstr = lower(getfield(options,'Diagnostics'));
  else
      diagstr = 'off'
  end

  simplstr = lower(getfield(options,'Simplex'));
  if strcmp(simplstr,'on')
     if strcmp(diagstr,'on')
        fprintf('Using free simplex optimizer\n')
     end
     param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_FREE_SIMPLEX';
  elseif strcmp(simplstr,'primal')
     if strcmp(diagstr,'on')
         fprintf('Using primal simplex optimizer\n')
     end
     param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_PRIMAL_SIMPLEX';
  elseif strcmp(simplstr,'dual')
     if strcmp(diagstr,'on')
         fprintf('Using dual simplex optimizer\n')
     end
     param.MSK_IPAR_OPTIMIZER = 'MSK_OPTIMIZER_DUAL_SIMPLEX';
  end
end

if echolev==0
   param.MSK_IPAR_LOG = 0;
end
param.MSK_IPAR_LOG_INTPNT    = prlev;
param.MSK_IPAR_LOG_SIM       = prlev;
param.MSK_IPAR_LOG_BI        = prlev;
param.MSK_IPAR_LOG_PRESOLVE  = prlev;

%switch ( optimget(options,'diagnostics') )
%  case 'on'
%     echolev = max(echolev,1);
%  otherwise
%     param.MSK_IPAR_MAX_NUM_WARNINGS = 0;
%end % switch

% fprintf('@ done\n');

%
% Copy Mosek options.
%

optnames = fieldnames(options);
[m,n]    = size(optnames);

for i=1:m
  if ( strncmp('MSK_',optnames(i),4) )
    % A MOSEK option.
    param = setfield(param,optnames{i},getfield(options,optnames{i}));
  end
end % for

if writefile==1 & ~strcmp(writename,'')
  writecmd = sprintf('write(%s)',writename);
else
  writecmd = '';
end

if ( domin )
   cmd = sprintf('minimize info echo(%d) statuskeys(1) symbcon %s',echolev,writecmd);
else
   cmd = sprintf('maximize info echo(%d) statuskeys(1) symbcon %s',echolev,writecmd);
end

