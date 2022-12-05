function output = mskoutput(res)
% Used by the MOSEK compability toolkit.

if ( isfield(res,'info') )
   output.iterations = res.info.MSK_IINF_INTPNT_ITER + ...
                       res.info.MSK_IINF_SIM_PRIMAL_ITER + ...
                       res.info.MSK_IINF_SIM_DUAL_ITER;
else
   output.iterations = 0;
end   

output.algorithm = 'MOSEK';
