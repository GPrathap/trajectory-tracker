% The callback function. It will display some information
% during optimization.
% handle: Is a user-defined data structure.
% where : Is an integer indicating from where in the optimization
%         process the callback was invoked.
% info  : A MATLAB structure containing information about the state of the
%         optimization.
function [r] = callback_handler(handle,where,info)

r = 0;   % r should always be assigned a value.

if handle.symbcon.MSK_CALLBACK_BEGIN_INTPNT==where
    disp(sprintf('Interior point optimizer started\n'));
end

if handle.symbcon.MSK_CALLBACK_END_INTPNT==where
  disp(sprintf('Interior-point optimizer terminated\n'));
  disp(sprintf('Interior-point primal obj.: %e\n', info.MSK_DINF_INTPNT_PRIMAL_OBJ));
  disp(sprintf('Iterations: %d\n', info.MSK_IINF_INTPNT_ITER));
end

if handle.symbcon.MSK_CALLBACK_NEW_INT_MIO==where
  disp(sprintf('New mixed-integer solution found\n'));
  disp(sprintf('Best objective.: %e\n', info.MSK_DINF_MIO_OBJ_BOUND));
end

% Decide if to terminate the optimization
% Terminate when cputime > handle.maxtime
if info.MSK_DINF_INTPNT_TIME > handle.maxtime
  r = 1;
else
  r = 0;
end
