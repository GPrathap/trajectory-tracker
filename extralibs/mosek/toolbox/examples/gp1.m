%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      gp1.m
%
%  Purpose:    Demonstrates how to solve a simple Geometric Program (GP)
%              cast into conic form with exponential cones and log-sum-exp.
%
%              Example from
%                https://gpkit.readthedocs.io/en/latest/examples.html//maximizing-the-volume-of-a-box
%

%
% maximize     h*w*d
% subjecto to  2*(h*w + h*d) <= Awall
%                       w*d <= Afloor
%              alpha <= h/w <= beta
%              gamma <= d/w <= delta
%
% Variable substitutions:  h = exp(x), w = exp(y), d = exp(z).
%
% maximize     x+y+z
% subject      log( exp(x+y+log(2/Awall)) + exp(x+z+log(2/Awall)) ) <= 0
%                              y+z <= log(Afloor)  
%              log( alpha ) <= x-y <= log( beta )
%              log( gamma ) <= z-y <= log( delta )
%
%
% log(sum(exp(A*x+b))) <= 0 is equivalent to sum(u) == 1, (ui,1,ai*x+bi) in Kexp,
% so we have two exp-cones and two auxilliary variables u1,u2.
%
% We order variables as (x,y,z,u1,u2),

[r,res] = mosekopt('symbcon');

% Input data
Awall = 200;
Afloor = 50;
alpha = 2;
beta = 10;
gamma = 2;
delta = 10;

% Objective
prob = [];
prob.c = [1, 1, 1, 0, 0]';

% Linear constraints:
% [ 0  0  0  1  1 ]                    == 1
% [ 0  1  1  0  0 ]                    <= log(Afloor)     
% [ 1 -1  0  0  0 ]                    in [log(alpha), log(beta)]
% [ 0 -1  1  0  0 ]                    in [log(gamma), log(delta)]
%
prob.a = [ 0  0  0  1  1;
           0  1  1  0  0;
           1 -1  0  0  0;
           0 -1  1  0  0 ];
           
prob.blc = [ 1; -inf;        log(alpha); log(gamma) ];
prob.buc = [ 1; log(Afloor); log(beta);  log(delta) ];

prob.blx = [ -inf; -inf; -inf; -inf; -inf];
prob.bux = [ inf; inf; inf; inf; inf];

% The conic part FX+g \in Kexp x Kexp
%   x  y  z  u  v
% [ 0  0  0  1  0 ]    0 
% [ 0  0  0  0  0 ]    1               in Kexp
% [ 1  1  0  0  0 ]    log(2/Awall)
%
% [ 0  0  0  0  1 ]    0
% [ 0  0  0  0  0 ]    1               in Kexp
% [ 1  0  1  0  0 ] +  log(2/Awall)
%
%         
prob.f = sparse([0 0 0 1 0; 
                 0 0 0 0 0;
                 1 1 0 0 0;
                 0 0 0 0 1;
                 0 0 0 0 0;
                 1 0 1 0 0]);
            
prob.g = [ 0; 1; log(2/Awall); 0; 1; log(2/Awall)];

prob.cones = [ res.symbcon.MSK_CT_PEXP, 3, res.symbcon.MSK_CT_PEXP, 3 ];

% Optimize and print results
[r,res]=mosekopt('maximize',prob);
exp(res.sol.itr.xx(1:3))