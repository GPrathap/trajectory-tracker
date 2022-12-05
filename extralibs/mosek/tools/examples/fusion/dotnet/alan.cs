//
// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
//  File:     alan.cs
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

using System;
using mosek.fusion;

namespace mosek.fusion.example
{
  public class alan
  {
    /////////////////////////////////////////////////////////////////////
    // Problem data.

    // Security names
    private static string[]
    securities = { "hardware", "software", "show-biz", "t-bills" };
    // Two examples of mean returns on securities
    private static double[]
    mean1      =  {        9.0,        7.0,       11.0,       5.0 };
    private static double[]
    mean2       = {        8.0,        9.0,       12.0,       7.0 };
    // Target mean return
    private static double
    target     = 10.0;

    // Factor of covariance matrix.
    private static Matrix U =
      Matrix.Dense(
        new double[,]
    { {  2.0       ,  1.5       , -0.5       , 0.0 },
      {  0.0       ,  1.93649167,  0.90369611, 0.0 },
      {  0.0       ,  0.0       ,  2.98886824, 0.0 },
      {  0.0       ,  0.0       ,  0.0       , 0.0 }
    });
    private static int numsec = securities.Length;
    public static void Main(String[] args)
    {
      // Create a parametrized model
      using (Model M = new Model("alan"))
      {
        Variable x = M.Variable("x",        numsec, Domain.GreaterThan(0.0));
        Variable t = M.Variable("variance", Domain.GreaterThan(0.0));
        M.Objective("minvar", ObjectiveSense.Minimize, t.AsExpr());

        // sum securities to 1.0
        M.Constraint("wealth",  Expr.Sum(x), Domain.EqualsTo(1.0));
        // define target expected return
        Parameter mean = M.Parameter(numsec);
        M.Constraint("dmean", Expr.Dot(mean, x), Domain.GreaterThan(target));

        M.Constraint("q",Expr.Vstack(Expr.ConstTerm(1, 0.5),
                                 t.AsExpr(),
                                 Expr.Mul(U, x)),
                     Domain.InRotatedQCone());

        // Solve two instances of the problem
        solve(M, mean, x, mean1);
        solve(M, mean, x, mean2);
      }
    }

    public static void solve(Model M, Parameter mean, Variable x, double[] meanVal) {
      Console.Write("Solve with mean = ");
      for (int i = 0; i < numsec; ++i)
        Console.Write("  {0}", meanVal[i]);
      Console.WriteLine();

      mean.SetValue(meanVal);
      M.Solve();        

      double[] solx = x.Level();

      Console.Write("Solution = ");
      for (int i = 0; i < numsec; ++i)
        Console.Write(" {0}", solx[i]);
      Console.WriteLine("");        
    }
  }
}
