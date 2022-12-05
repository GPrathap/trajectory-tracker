function mskerrmsg(caller,msg)
% Purpose: Prints out an error message and stops the execution.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

disp(sprintf('ERROR - (%s): %s',caller,msg));

