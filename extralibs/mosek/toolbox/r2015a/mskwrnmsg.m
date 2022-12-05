function mskwrnmsg(caller,msg)
% Purpose: Prints out an error message.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved. 

warning(sprintf('(%s): %s',caller,msg));

