%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      sensitivity.m
%
%  Purpose:   To demonstrate how to perform sensitivity
%             analysis from the MOSEK Toolbox on a small problem:
%
%  minimize
%
%  obj: +1 x11 + 2 x12 + 5 x23 + 2 x24 + 1 x31 + 2 x33 + 1 x34
%  st
%  c1:   +  x11 +   x12                                          <= 400
%  c2:                  +   x23 +   x24                          <= 1200
%  c3:                                  +   x31 +   x33 +   x34  <= 1000
%  c4:   +  x11                         +   x31                   = 800
%  c5:          +   x12                                           = 100
%  c6:                  +   x23                 +   x33           = 500
%  c7:                          +   x24                 +   x34   = 500
%
function sensitivity()

clear prob;

% Obtain all symbolic constants
% defined by MOSEK.
[r,res]  = mosekopt('symbcon');
sc       = res.symbcon;

prob.blc = [-Inf, -Inf, -Inf, 800,100,500,500];
prob.buc = [ 400, 1200, 1000, 800,100,500,500];
prob.c   = [1.0,2.0,5.0,2.0,1.0,2.0,1.0]';
prob.blx = [0.0,0.0,0.0,0.0,0.0,0.0,0.0];
prob.bux = [Inf,Inf,Inf,Inf, Inf,Inf,Inf];

subi     = [  1,  1,  2,  2,  3,  3,  3,  4,  4,  5,  6,  6,  7,  7];
subj     = [  1,  2,  3,  4,  5,  6,  7,  1,  5,  6,  3,  6,  4,  7];
val      = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0];

prob.a = sparse(subi,subj,val);

% analyse upper bound 1:7
prob.prisen.cons.subl = [];
prob.prisen.cons.subu = [1:7];
% analyse lower bound on variables 1:7
prob.prisen.vars.subl = [1:7];
prob.prisen.vars.subu = [];
% analyse coeficient 1:7
prob.duasen.sub = [1:7];
[r,res] = mosekopt('minimize echo(0)',prob);

%Print results

fprintf('\nBasis sensitivity results:\n')
fprintf('\nSensitivity for bounds on constraints:\n')
for i = 1:length(prob.prisen.cons.subl)
  fprintf (...
  'con = %d, beta_1 = %.1f, beta_2 = %.1f, delta_1 = %.1f,delta_2 = %.1f\n', ...
  prob.prisen.cons.subl(i),res.prisen.cons.lr_bl(i), ...
  res.prisen.cons.rr_bl(i),...
  res.prisen.cons.ls_bl(i),...
  res.prisen.cons.rs_bl(i));
end

for i = 1:length(prob.prisen.cons.subu)
  fprintf (...
  'con = %d, beta_1 = %.1f, beta_2 = %.1f, delta_1 = %.1f,delta_2 = %.1f\n', ...
  prob.prisen.cons.subu(i),res.prisen.cons.lr_bu(i), ...
  res.prisen.cons.rr_bu(i),...
  res.prisen.cons.ls_bu(i),...
  res.prisen.cons.rs_bu(i));
end
fprintf('Sensitivity for bounds on variables:\n')
for i = 1:length(prob.prisen.vars.subl)
fprintf (...
'var = %d, beta_1 = %.1f, beta_2 = %.1f, delta_1 = %.1f,delta_2 = %.1f\n', ...
 prob.prisen.vars.subl(i),res.prisen.vars.lr_bl(i), ...
 res.prisen.vars.rr_bl(i),...
 res.prisen.vars.ls_bl(i),...
 res.prisen.vars.rs_bl(i));
end

for i = 1:length(prob.prisen.vars.subu)
  fprintf (...
  'var = %d, beta_1 = %.1f, beta_2 = %.1f, delta_1 = %.1f,delta_2 = %.1f\n', ...
  prob.prisen.vars.subu(i),res.prisen.vars.lr_bu(i), ...
  res.prisen.vars.rr_bu(i),...
  res.prisen.vars.ls_bu(i),...
  res.prisen.vars.rs_bu(i));
end

fprintf('Sensitivity for coefficients in objective:\n')
for i = 1:length(prob.duasen.sub)
  fprintf (...
  'var = %d, beta_1 = %.1f, beta_2 = %.1f, delta_1 = %.1f,delta_2 = %.1f\n', ...
  prob.duasen.sub(i),res.duasen.lr_c(i), ...
  res.duasen.rr_c(i),...
  res.duasen.ls_c(i),...
  res.duasen.rs_c(i));
end