%%
%  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File:      feasrepairex1.m
%
%  Purpose:   To demonstrate how to use the MSK_primalrepair function to
%             repair an infeasible problem.
%
%%

function feasrepairex1(inputfile)

cmd = sprintf('read(%s)', inputfile);
[r,res]=mosekopt(cmd);

res.prob.primalrepair = [];
res.prob.primalrepair.wux = [1,1];
res.prob.primalrepair.wlx = [1,1];
res.prob.primalrepair.wuc = [1,1,1,1];
res.prob.primalrepair.wlc = [1,1,1,1];

param.MSK_IPAR_LOG_FEAS_REPAIR = 3;
[r,res]=mosekopt('minimize primalrepair',res.prob,param);
fprintf('Return code: %d\n',r);

end