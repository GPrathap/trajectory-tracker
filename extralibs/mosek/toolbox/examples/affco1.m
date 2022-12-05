%%
%  File : affco1.m
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  Description :
%    Implements a basic tutorial example with affine conic constraints:
%
%    maximize x_1^(1/3) + (x_1+x_2+0.1)^(1/4)
%    st.      (x_1-0.5)^2 + (x_2-0.6)^2 <= 1
%             0 <= x_1 <= x_2 + 1
% 
%%
function affco1()

[rcode, res] = mosekopt('symbcon echo(0)');
prob = [];

% Variables [x1; x2; t1; t2]
prob.c = [0, 0, 1, 1];

% Linear inequality x_1 - x_2 <= 1
prob.a = sparse([1, -1, 0, 0]);
prob.buc = 1;
prob.blc = [];

% The quadratic cone
FQ = sparse([zeros(1,4); speye(2) zeros(2,2)]);
gQ = [1 -0.5 -0.6]';
cQ = [res.symbcon.MSK_CT_QUAD 3];

% The power cone for (x_1, 1, t_1) \in POW3^(1/3,2/3)
FP1 = sparse([1 0 zeros(1,2); zeros(1,4); zeros(1,2) 1 0]);
gP1 = [0 1 0]';
cP1 = [res.symbcon.MSK_CT_PPOW 3 2 1/3 2/3];

% The power cone for (x_1+x_2+0.1, 1, t_2) \in POW3^(1/4,3/4)
FP2 = sparse([1 1 zeros(1,2); zeros(1,4); zeros(1,2) 0 1]);
gP2 = [0.1 1 0]';
cP2 = [res.symbcon.MSK_CT_PPOW 3 2 1.0 3.0];

% All cones
prob.f = [FQ; FP1; FP2];
prob.g = [gQ; gP1; gP2];
prob.cones = [cQ cP1 cP2];

[r, res] = mosekopt('maximize', prob);

res.sol.itr.pobjval
res.sol.itr.xx(1:2)