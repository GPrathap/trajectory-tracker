//
// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
// File:      total_variation.cs
//
// Purpose:   Demonstrates how to solve a total
//            variation problem using the Fusion API.using System;
using System;
using mosek.fusion;

namespace mosek.fusion.example
{
  public class total_variation
  {
    public static Model total_var(int n, int m) {
      Model M = new Model("TV");

      Variable u = M.Variable("u", new int[] {n + 1, m + 1}, Domain.InRange(0.0, 1.0));
      Variable t = M.Variable("t", new int[] {n, m}, Domain.Unbounded());

      // In this example we define sigma and the input image f as parameters
      // to demonstrate how to solve the same model with many data variants.
      // Of course they could simply be passed as ordinary arrays if that is not needed.
      Parameter sigma = M.Parameter("sigma");
      Parameter f = M.Parameter("f", n, m);

      Variable ucore = u.Slice(new int[] {0, 0}, new int[] {n, m});

      Expression deltax = Expr.Sub( u.Slice( new int[] {1, 0}, new int[] {n + 1, m} ), ucore );
      Expression deltay = Expr.Sub( u.Slice( new int[] {0, 1}, new int[] {n, m + 1} ), ucore );

      M.Constraint( Expr.Stack(2, t, deltax, deltay), Domain.InQCone().Axis(2) );

      M.Constraint( Expr.Vstack(sigma, Expr.Flatten( Expr.Sub(f,  ucore) ) ),
                    Domain.InQCone() );

      M.Objective( ObjectiveSense.Minimize, Expr.Sum(t) );

      return M;
    }

    public static void Main(string[] args)
    {
      Random randGen = new Random(0);
      
      int n = 100;
      int m = 200;
      double[] sigmas = { 0.0004, 0.0005, 0.0006 };

      // Create a parametrized model with given shape
      Model M = total_var(n, m);
      Parameter sigma = M.GetParameter("sigma");
      Parameter f     = M.GetParameter("f");
      Variable  ucore = M.GetVariable("u").Slice(new int[] {0,0}, new int[] {n,m});

      // Example: Linear signal with Gaussian noise    
      double[,] signal = new double[n,m];
      double[,] noise  = new double[n,m];
      double[,] fVal   = new double[n,m];
      double[,] sol    = new double[n,m];

      for(int i=0; i<n; i++) for(int j=0; j<m; j++) {
        signal[i,j] = 1.0*(i+j)/(n+m);
        noise[i,j] = Math.Sqrt(-2.0 * Math.Log(randGen.NextDouble())) * 
                     Math.Sin(2.0 * Math.PI * randGen.NextDouble()) *
                     0.08;
        fVal[i,j] = Math.Max( Math.Min(1.0, signal[i,j] + noise[i,j]), .0 );
      }
     
      // Set value for f
      f.SetValue(fVal);

      for(int iter=0; iter<3; iter++) {
        // Set new value for sigma and solve
        sigma.SetValue(sigmas[iter]*n*m);

        M.Solve();

        // Retrieve solution from ucore
        double[] ucoreLev = ucore.Level();
        for(int i=0; i<n; i++) for(int j=0; j<m; j++)
          sol[i,j] = ucoreLev[i*n+m];

        // Use the solution
        // ...

        Console.WriteLine("rel_sigma = " + sigmas[iter] + " total_var = " + M.PrimalObjValue());
      }

      M.Dispose();
    }
  }
}