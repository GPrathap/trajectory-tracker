%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      sdo2.m
%
%  Purpose :   Solves the semidefinite problem with two symmetric variables:
%
%                 min   <C1,X1> + <C2,X2>
%                 st.   <A1,X1> + <A2,X2> = b
%                             (X2)_{1,2} <= k
%                
%                 where X1, X2 are symmetric positive semidefinite,
%
%                 C1, C2, A1, A2 are assumed to be constant symmetric matrices,
%                 and b, k are constants.
                
function sdo2()

% Sample data
C1 = [ 1 0 0; 0 0 0; 0 0 6 ];
A1 = [ 1 0 1; 0 0 0; 1 0 2 ];
C2 = [ 1 -3 0 0; -3 2 0 0; 0 0 1 0; 0 0 0 0 ];
A2 = [ 0 1 0 0; 1 -1 0 0; 0 0 0 0; 0 0 0 -3 ];
b = 23;
k = -3;

% The scalar part, as in linear optimization examples
prob.c = [];
prob.a = sparse([], [], [], 2, 0);          % 2 constraints, no scalar variables
prob.blc = [b -inf];                        % Bounds
prob.buc = [b k];

% Dimensions of PSD variables
prob.bardim = [3, 4];

% Coefficients in the objective
[r1,c1,v1] = find(tril(C1));
[r2,c2,v2] = find(tril(C2));

prob.barc.subj = [repmat(1,length(v1),1);                % Which PSD variable (j)
                  repmat(2,length(v2),1)];           
prob.barc.subk = [r1; r2];                               % Which matrix entry and value ((k,l)->v)
prob.barc.subl = [c1; c2];
prob.barc.val = [v1; v2];

% Coefficients in the constraints
[r1,c1,v1] = find(tril(A1));
[r2,c2,v2] = find(tril(A2));

prob.bara.subi = [ones(length(v1)+length(v2),1);         % Which constraint (i)
                  2];     
prob.bara.subj = [repmat(1,length(v1),1); 
                  repmat(2,length(v2),1); 
                  2];                                    % Which PSD variable (j)
prob.bara.subk = [r1; r2; 2];                            % Which matrix entry and value ((k,l)->v)
prob.bara.subl = [c1; c2; 1];
prob.bara.val = [v1; v2; 0.5];

% Solve with log output
[r, res] = mosekopt('write(test.ptf) minimize echo(10)', prob);

% Retrieve the result assuming primal and dual feasible
X1 = zeros(3);
X1([1,2,3,5,6,9]) = res.sol.itr.barx(1:6);
X1 = X1 + tril(X1,-1)';

X2 = zeros(4);
X2([1,2,3,4,6,7,8,11,12,16]) = res.sol.itr.barx(7:16);
X2 = X2 + tril(X2,-1)';

X1
X2
