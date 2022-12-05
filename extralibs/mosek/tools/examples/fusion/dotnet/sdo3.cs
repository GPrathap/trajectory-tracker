/*
  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.

  File :      sdo3.cs

  Purpose :   Solves the semidefinite problem:

                 min   tr(X_1) + ... + tr(X_n)
                 st.   <A_11,X_1> + ... + <A_1n,X_n> >= b_1
                       ...
                       <A_k1,X_1> + ... + <A_kn,X_n> >= b_k
                
                 where X_i are symmetric positive semidefinite of dimension d,

                 A_ji are constant symmetric matrices and b_i are constant.

              This example is to demonstrate creating and using 
              many matrix variables of the same dimension.
*/

using System;
using mosek.fusion;

namespace mosek.fusion.example
{
  public class sdo3
  {

    // A helper method computing a semidefinite slice of a 3-dim variable
    public static Variable Slice(Variable X, int d, int j) {
      return
        X.Slice(new int[] {j,0,0}, new int[] {j+1,d,d})
         .Reshape(new int[] {d,d});
    }

    public static void Main(string[] args)
    {

      // Sample input data
      int n = 100;
      int d = 4;
      int k = 3;
      double[] b = {9,10,11};
      double[][,] A = new double[n*k][,];
      var rand = new Random();
      for(int i=0; i<n*k; i++) {
        A[i] = new double[d,d];
        for(int s1=0; s1<d; s1++)
          for(int s2=0; s2<=s1; s2++)
            A[i][s1,s2] = A[i][s2,s1] = rand.NextDouble();
      }

      // Create a model with n semidefinite variables od dimension d x d
      using (Model M  = new Model("sdo3"))
      {
        Variable X = M.Variable(Domain.InPSDCone(d, n));

        // Pick indexes of diagonal entries for the objective
        int[,] alldiag = new int[d*n,3];
        for(int j=0; j<n; j++) for(int s=0; s<d; s++) {
          alldiag[j*d+s,0] = j; 
          alldiag[j*d+s,1] = alldiag[j*d+s,2] = s;
        }
        M.Objective(ObjectiveSense.Minimize, Expr.Sum( X.Pick(alldiag) ));

        // Each constraint is a sum of inner products
        // Each semidefinite variable is a slice of X
        for(int i=0; i< k; i++) {
          Expression[] addlist = new Expression[n];
          for(int j=0; j<n; j++) 
            addlist[j] = Expr.Dot(A[i*n+j], Slice(X, d, j));
          M.Constraint(Expr.Add(addlist), Domain.GreaterThan(b[i]));
        }

        // Solve
        M.SetLogHandler(Console.Out);           // Add logging
        M.WriteTask("sdo3.ptf");                // Save problem in readable format
        M.Solve();

        // Get results. Each variable is a slice of X
        Console.WriteLine("Contributing variables:");
        for(int j=0; j<n; j++) {
          double[] Xj = Slice(X, d, j).Level();
          double maxval = 0;
          for(int s=0; s<d*d; s++) maxval = Math.Max(maxval, Xj[s]);
          if (maxval > 1e-6) {
            Console.WriteLine("X" + j + "=");
            for(int s1=0; s1<d; s1++) {
              for(int s2=0; s2<d; s2++)
                Console.Write(Xj[s1*d+s1] + "  ");    
              Console.WriteLine();
            }
          }
        }
      } 
    }
  }
}