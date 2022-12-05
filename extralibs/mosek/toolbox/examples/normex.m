%%
%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      normex.m
%
%  Purpose:   Demonstrates various norm minimization problems
%
%             * least squares regression
%             * ridge regularization
%             * lasso regularization
%             * p-norm minimization
%
%%

% This function runs a few examples
function normex()
% Create some random data
N  = 100;
x1 = sort(rand(N,1));
p  = @(t) 1.5*t.^3-2*t.^2+0.5*t.^1-1;
y1 = p(x1) + 0.01*randn(N,1);
xp = smallVander(x1, 5);
figure(1);
scatter(x1, y1);
hold on

% Least squares regression
x = norm_lse(xp, y1, zeros(0,6), []);
plot(0:0.01:1, smallVander((0:0.01:1)', 5)*x);

% With ridge regularization
x = norm_lse_reg(xp, y1, zeros(0,6), [], 0.1);
plot(0:0.01:1, smallVander((0:0.01:1)', 5)*x);

% Quadratic ridge
x = norm_lse_reg_quad(xp, y1, zeros(0,6), [], 0.2);
plot(0:0.01:1, smallVander((0:0.01:1)', 5)*x);

% Completely random large data for lasso example
N = 100;
K = 3000;
F = sprandn(K, N, 0.5);
g = randn(K, 1);
disp(sprintf('Lasso regularization'));
for gamma=[0.01, 0.1, 0.3, 0.6, 0.9, 1.3]
   x = norm_lse_lasso(F, g, zeros(0,N), [], gamma);
   disp(sprintf('Gamma %.4f  density %.0f%%   |Fx-g|_2: %.4f', gamma, sum(abs(x)>1e-6)/N*100, norm(F*x-g)));
end

% Example with the p-norm cone for various p
% We add a far outlier to the first example
x12 = [x1; 0.73];
y12 = [y1; -0.99];
xp2 = [x12.^5, x12.^4, x12.^3, x12.^2, x12.^1, x12.^0];
figure(2);
parts = [];
labels = [];
scatter(x12, y12);
hold on
for p=[1.1, 2.0, 3.0, 6.0]
   x = norm_p_norm(xp2, y12, zeros(0,6), [], p);
   parts = [parts; plot(0:0.01:1, smallVander((0:0.01:1)', 5)*x);];
   labels = [labels; sprintf('%.1f', p)];
end
legend(parts, labels);
end

% Least squares regression
% minimize \|Fx-g\|_2
function x = norm_lse(F,g,A,b)
clear prob;
[r, res] = mosekopt('symbcon');
n = size(F,2);
k = size(g,1);
m = size(A,1);

% Linear constraints in [x; t]
prob.a   = [A, zeros(m,1)];
prob.buc = b;
prob.blc = b;
prob.blx = -inf*ones(n+1,1);
prob.bux = inf*ones(n+1,1);
prob.c   = [zeros(n,1); 1];

% Affine conic constraint
prob.f = sparse([zeros(1,n), 1; F, zeros(k,1)]);
prob.g = [0; -g];
prob.cones = [ res.symbcon.MSK_CT_QUAD k+1 ];

% Solve
[r, res] = mosekopt('minimize echo(0)', prob);
x = res.sol.itr.xx(1:n);
end

% Least squares regression with regularization
% minimize \|Fx-g\|_2 + gamma*\|x\|_2
function x = norm_lse_reg(F,g,A,b,gamma)
clear prob;
[r, res] = mosekopt('symbcon');
n = size(F,2);
k = size(g,1);
m = size(A,1);

% Linear constraints in [x; t1; t2]
prob.a   = [A, zeros(m,2)];
prob.buc = b;
prob.blc = b;
prob.blx = -inf*ones(n+2,1);
prob.bux = inf*ones(n+2,1);
prob.c   = [zeros(n,1); 1; gamma];

% Affine conic constraint
prob.f = sparse([zeros(1,n),        1, 0; ...
                 F,           zeros(k,2); ...
                 zeros(1,n),        0, 1; ...
                 eye(n),      zeros(n,2) ]);
prob.g = [0; -g; zeros(n+1,1)];
prob.cones = [ res.symbcon.MSK_CT_QUAD k+1 res.symbcon.MSK_CT_QUAD n+1 ];

% Solve
[r, res] = mosekopt('minimize echo(0)', prob);
x = res.sol.itr.xx(1:n);
end

% Least squares regression with regularization
% The "classical" quadratic version
% minimize \|Fx-g\|_2^2 + gamma*\|x\|_2^2
function x = norm_lse_reg_quad(F,g,A,b,gamma)
clear prob;
[r, res] = mosekopt('symbcon');
n = size(F,2);
k = size(g,1);
m = size(A,1);

% Linear constraints in [x; t1; t2]
prob.a   = [A, zeros(m,2)];
prob.buc = b;
prob.blc = b;
prob.blx = -inf*ones(n+2,1);
prob.bux = inf*ones(n+2,1);
prob.c   = [zeros(n,1); 1; gamma];

% Affine conic constraint
prob.f = sparse([zeros(1,n),        1, 0; ...
                 zeros(1,n+2)           ; ...
                 F,           zeros(k,2); ...
                 zeros(1,n),        0, 1; ...
                 zeros(1,n+2)           ; ...                
                 eye(n),      zeros(n,2) ]);
prob.g = [0; 0.5; -g; 0; 0.5; zeros(n,1)];
prob.cones = [ res.symbcon.MSK_CT_RQUAD k+2 res.symbcon.MSK_CT_RQUAD n+2 ];

% Solve
[r, res] = mosekopt('minimize echo(0)', prob);
x = res.sol.itr.xx(1:n);
end

% Least squares regression with lasso regularization
% minimize \|Fx-g\|_2 + gamma*\|x\|_1
function x = norm_lse_lasso(F,g,A,b,gamma)
clear prob;
[r, res] = mosekopt('symbcon');
n = size(F,2);
k = size(g,1);
m = size(A,1);

% Linear constraints in [x; u; t1; t2]
prob.a   = [A,         zeros(m,n+2)      ; ...
            eye(n),    eye(n), zeros(n,2); ...
            -eye(n),   eye(n), zeros(n,2); ...
            zeros(1,n) -ones(1,n), 0, 1 ];
prob.buc = [b; inf*ones(2*n+1,1)];
prob.blc = [b; zeros(2*n+1,1)];
prob.blx = -inf*ones(2*n+2,1);
prob.bux = inf*ones(2*n+2,1);
prob.c   = [zeros(2*n,1); 1; gamma];

% Affine conic constraint
prob.f = sparse([zeros(1,2*n), 1, 0; F, zeros(k,n+2)]);
prob.g = [0; -g];
prob.cones = [ res.symbcon.MSK_CT_QUAD k+1 ];

% Solve
[r, res] = mosekopt('minimize echo(0)', prob);
x = res.sol.itr.xx(1:n);
end

% P-norm minimization
% minimize \|Fx-g\|_p
function x = norm_p_norm(F,g,A,b,p)
clear prob;
[r, res] = mosekopt('symbcon');
n = size(F,2);
k = size(g,1);
m = size(A,1);

% Linear constraints in [x; r; t]
prob.a   = [A, zeros(m,k+1); zeros(1,n), ones(1,k), -1];
prob.buc = [b; 0];
prob.blc = [b; 0];
prob.blx = -inf*ones(n+k+1,1);
prob.bux = inf*ones(n+k+1,1);
prob.c   = [zeros(n+k,1); 1];

% Permutation matrix which picks triples (r_i, t, F_ix-g_i)
M = [];
for i=1:3
  M = [M, sparse(i:3:3*k, 1:k, ones(k,1), 3*k, k)];
end

% Affine conic constraint
prob.f = M * sparse([zeros(k,n), eye(k), zeros(k,1); zeros(k,n+k), ones(k,1); F, zeros(k,k+1)]);
prob.g = M * [zeros(2*k,1); -g];
prob.cones = [ repmat([res.symbcon.MSK_CT_PPOW, 3, 2, 1.0, p-1], 1, k) ];

% Solve
[r, res] = mosekopt('minimize echo(0)', prob);
x = res.sol.itr.xx(1:n);
end

% Just a few columns of the Vandermode matrix
function M = smallVander(v, k)
M = [];
for i=0:k 
    M = [v.^i, M];
end
end