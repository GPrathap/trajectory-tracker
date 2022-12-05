function [exitflag] = mskeflag(rcode,res)
% Used by the MOSEK compatibility toolbox.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

if rcode==0 && isfield(res,'sol')   
  if ( isfield(res.sol,'itr') )
    if ( res.sol.itr.solsta==res.symbcon.MSK_SOL_STA_OPTIMAL) 
      exitflag = 1;
    elseif (res.sol.itr.solsta==res.symbcon.MSK_SOL_STA_PRIM_INFEAS_CER)
        exitflag = -2;
    elseif (res.sol.itr.solsta==res.symbcon.MSK_SOL_STA_DUAL_INFEAS_CER)
        exitflag = -3;        
    else
      warning(sprintf('unexpected MOSEK solution status: %d',res.sol.itr.solsta));
      exitflag = 2^31-1;
    end
  elseif ( isfield(res.sol,'int') )
    if ( res.sol.int.solsta==res.symbcon.MSK_SOL_STA_INTEGER_OPTIMAL) 
      exitflag = 1;
    elseif (res.sol.int.prosta==res.symbcon.MSK_SOL_STA_PRIM_INFEAS)
        exitflag = -2;
    else
      warning(sprintf('unexpected MOSEK solution status: %d',res.sol.int.solsta));
      exitflag = 2^31-1;        
    end
  elseif ( isfield(res.sol,'bas') )
    if ( res.sol.bas.solsta==res.symbcon.MSK_SOL_STA_OPTIMAL)
        exitflag = 1;
    elseif (res.sol.bas.solsta==res.symbcon.MSK_SOL_STA_PRIM_INFEAS_CER)
        exitflag = -2;
    elseif (res.sol.bas.solsta==res.symbcon.MSK_SOL_STA_DUAL_INFEAS_CER)
        exitflag = -3;        
    else
      warning(sprintf('unexpected MOSEK solution status: %d',res.sol.bas.solsta));
      exitflag = 2^31-1;                  
    end
  end
elseif isfield(res,'sol')        
  if ( isfield(res.sol,'int') )
    if (rcode==res.symbcon.MSK_RES_TRM_MIO_NUM_BRANCHES)
      exitflag = -4;
    end	
    if (rcode==res.symbcon.MSK_RES_TRM_MAX_TIME)
      exitflag = -5;
    end	
  else 
    if (isfield(res,'symbcon') && rcode==res.symbcon.MSK_RES_TRM_MAX_ITERATIONS)
      exitflag = 0;
    else
      exitflag = -1;
    end
  end
elseif rcode==res.symbcon.MSK_RES_ERR_OBJ_Q_NOT_PSD
  exitflag = -6;  
else
  exitflag = 2^31-1; 
end

