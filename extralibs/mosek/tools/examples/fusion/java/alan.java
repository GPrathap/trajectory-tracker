//
// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
//  File:     alan.java
//
//  Purpose: This file contains an implementation of the alan.gms (as
//  found in the GAMS online model collection) using Fusion.
//
//  The model is a simple portfolio choice model. The objective is to
//  invest in a number of assets such that we minimize the risk, while
//  requiring a certain expected return.
//
//  We operate with 4 assets (hardware,software, show-biz and treasure
//  bill). The risk is defined by the covariance matrix
//    Q = [[  4.0, 3.0, -1.0, 0.0 ],
//         [  3.0, 6.0,  1.0, 0.0 ],
//         [ -1.0, 1.0, 10.0, 0.0 ],
//         [  0.0, 0.0,  0.0, 0.0 ]]
//
//
//  We use the form Q = U^T * U, where U is a Cholesky factor of Q.
//

package com.mosek.fusion.examples;
import mosek.fusion.*;

public class alan {
  /////////////////////////////////////////////////////////////////////
  // Problem data.
  // Security names
  private static final String[]
    securities = { "hardware", "software", "show-biz", "t-bills" };
  // Two sample mean returns on securities
  private static final double[]
    mean1      = {        9.0,        7.0,       11.0,       5.0 };
  private static final double[]
    mean2      = {        8.0,        9.0,       12.0,       7.0 };

  // Target mean return
  private static final double
    target     = 10.0;

  private static final int numsec = securities.length;

  // Factor of covariance matrix.
  private static final double[][] U_data = {
    { 2.0       ,  1.5       , -0.5       , 0.0 },
    { 0.0       ,  1.93649167,  0.90369611, 0.0 },
    { 0.0       ,  0.0       ,  2.98886824, 0.0 },
    { 0.0       ,  0.0       ,  0.0       , 0.0 }
  };
  private static final Matrix U = Matrix.dense(U_data);

  public static void main(String[] args)
  throws SolutionError {
    // Create a parametrized model
    Model M = new Model("alan");
    try {

      Variable x = M.variable("x", numsec, Domain.greaterThan(0.0));
      Variable t = M.variable("t", 1,      Domain.greaterThan(0.0));
      M.objective("minvar", ObjectiveSense.Minimize, t);

      // sum securities to 1.0
      M.constraint("wealth",  Expr.sum(x), Domain.equalsTo(1.0));
      // define target expected return
      Parameter mean = M.parameter(numsec);
      M.constraint("dmean", Expr.dot(mean, x), Domain.greaterThan(target));

      M.constraint("t > ||Ux||^2", Expr.vstack(0.5, t, Expr.mul(U, x)), Domain.inRotatedQCone());

      // Solve with two values of mean
      solve(M, mean, x, mean1);
      solve(M, mean, x, mean2);
    } finally {
      M.dispose();
    }
  }

  // Solve the model given some expected return vector
  public static void solve(Model M, Parameter mean, Variable x, double[] meanVal)
  throws SolutionError {
    mean.setValue(meanVal);

    System.out.print("Solve with mean = ");
    for (int i = 0; i < numsec; ++i)
      System.out.printf(" %e", meanVal[i]);
    System.out.print("\n");

    M.solve();

    double[] solx = x.level();
    System.out.print("Solution = ");
    for (int i = 0; i < numsec; ++i)
      System.out.printf(" %e", solx[i]);
    System.out.print("\n");
  }

}