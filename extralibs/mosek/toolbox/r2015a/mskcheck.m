function [rcode,b,c,l,u] = mskcheck(caller,verb,n,sizea,b,sizeb,c,l,u)
% Purpose: Check problem data.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

rcode = 0;

if ( ( sizea(1)~=0 | sizea(2)~=0 ) & sizea(2)~=n )
   rcode = -1;
   mskerrmsg(caller,'Incorrect number of columns in A (inequalities)'); 
end

if ( ( sizeb(1)~=0 | sizeb(2)~=0 ) & sizeb(2)~=n )
   rcode = -1;
   mskerrmsg(caller,'Incorrect number of columns in B (equalities)');
end   

if ( length(b)~=sizea(1) )
   rcode = -1;
   mskerrmsg(caller,'b is of incorrect dimension compared to A (inequalities)');
else
   b = b(:);
end   

if ( length(c)~=sizeb(1) )
   rcode = -1;
   mskerrmsg(caller,'c is of incorrect dimension compared to B (inequalities)');
else
   c = c(:);
end   

l = l(:);
if ( length(l)>n )
   mskwrnmsg(caller,'Length of lower bounds l is too long; ignoring extra bounds');
   l = l(1:n);
else 
   l = [l;-inf*ones(n-length(l),1)];
end

u = u(:);
if ( length(u)>n )
   mskwrnmsg(caller,'Length of upper bounds u is too long; ignoring extra bounds');
   u = u(1:n);
else
   u = [u;inf*ones(n-length(u),1)];
end   

if ( any(l>u) )
   rcode = -1;
   count = full(sum(l>u));
   if ( verb )
     disp(sprintf(['\nExiting due to infeasibility:  %i lower bound exceeds the' ...
                   ' corresponding upper bound.\n'],count));
   end 
end

if ( any(eq(u,-inf)) )
   rcode = -1;
   mskerrmsg(caller,'-Inf detected in upper bound: upper bounds must be > -Inf.');
elseif ( any(eq(l,inf)) )
   rocde = -1;
   mskerrmsg(caller,'+Inf detected in lower bound: lower bounds must be < Inf.');
end
