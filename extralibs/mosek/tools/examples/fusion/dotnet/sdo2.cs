/*
   Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
 
   File :      sdo2.cs
 
   Purpose :   Solves the semidefinite problem with two symmetric variables:
 
                  min   <C1,X1> + <C2,X2>
                  st.   <A1,X1> + <A2,X2> = b
                              (X2)_{1,2} <= k
                 
                  where X1, X2 are symmetric positive semidefinite,
 
                  C1, C2, A1, A2 are assumed to be constant symmetric matrices,
                  and b, k are constants.
*/
using System;
using mosek.fusion;

namespace mosek.fusion.example
{
  public class sdo2
  {
    public static void Main(string[] args)
    {

      // Sample data in sparse, symmetric triplet format
      int[]    C1_k = {0, 2};
      int[]    C1_l = {0, 2};
      double[] C1_v = {1, 6};
      int[]    A1_k = {0, 2, 0, 2};
      int[]    A1_l = {0, 0, 2, 2};
      double[] A1_v = {1, 1, 1, 2};
      int[]    C2_k = {0, 1, 0, 1, 2};
      int[]    C2_l = {0, 0, 1, 1, 2};
      double[] C2_v = {1, -3, -3, 2, 1};
      int[]    A2_k = {1, 0, 1, 3};
      int[]    A2_l = {0, 1, 1, 3};
      double[] A2_v = {1, 1, -1, -3};
      double b = 23;
      double k = -3;

      // Convert input data into Fusion sparse matrices
      Matrix C1 = Matrix.Sparse(3, 3, C1_k, C1_l, C1_v);
      Matrix C2 = Matrix.Sparse(4, 4, C2_k, C2_l, C2_v);
      Matrix A1 = Matrix.Sparse(3, 3, A1_k, A1_l, A1_v);
      Matrix A2 = Matrix.Sparse(4, 4, A2_k, A2_l, A2_v);

      using (Model M  = new Model("sdo2"))
      {
        // Two semidefinite variables
        Variable X1 = M.Variable(Domain.InPSDCone(3));
        Variable X2 = M.Variable(Domain.InPSDCone(4));

        // Objective
        M.Objective(ObjectiveSense.Minimize, Expr.Add(Expr.Dot(C1,X1), Expr.Dot(C2,X2)));

        // Equality constraint
        M.Constraint(Expr.Add(Expr.Dot(A1,X1), Expr.Dot(A2,X2)), Domain.EqualsTo(b));

        // Inequality constraint
        M.Constraint(X2.Index(new int[] {0,1}), Domain.LessThan(k));

        // Solve
        M.SetLogHandler(Console.Out);
        M.Solve();

        // Print solution
        Console.WriteLine("Solution (vectorized):");
        Console.WriteLine("[{0}]", (new Utils.StringBuffer()).A(X1.Level()).ToString());
        Console.WriteLine("[{0}]", (new Utils.StringBuffer()).A(X2.Level()).ToString());
      }
    }
  }
}