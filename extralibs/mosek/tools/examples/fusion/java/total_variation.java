//
// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
// File:      total_variation.java
//
// Purpose:   Demonstrates how to solve a total
//            variation problem using the Fusion API.
//

package com.mosek.fusion.examples;
import java.util.Random;
import mosek.fusion.*;

public class total_variation {

  public static Model total_var(int n, int m) {
    Model M = new Model("TV");

    Variable u = M.variable("u", new int[] {n + 1, m + 1}, Domain.inRange(0., 1.0));
    Variable t = M.variable("t", new int[] {n, m}, Domain.unbounded());

    // In this example we define sigma and the input image f as parameters
    // to demonstrate how to solve the same model with many data variants.
    // Of course they could simply be passed as ordinary arrays if that is not needed.
    Parameter sigma = M.parameter("sigma");
    Parameter f = M.parameter("f", n, m);

    Variable ucore = u.slice(new int[] {0, 0}, new int[] {n, m});

    Expression deltax = Expr.sub( u.slice( new int[] {1, 0}, new int[] {n + 1, m} ), ucore );
    Expression deltay = Expr.sub( u.slice( new int[] {0, 1}, new int[] {n, m + 1} ), ucore );

    M.constraint( Expr.stack(2, t, deltax, deltay), Domain.inQCone().axis(2) );

    M.constraint( Expr.vstack(sigma, Expr.flatten( Expr.sub(f,  ucore) ) ),
                  Domain.inQCone() );

    M.objective( ObjectiveSense.Minimize, Expr.sum(t) );

    return M;
  }

  public static void main(String[] args)
  throws SolutionError {
    Random randGen = new Random(0);
    
    int n = 100;
    int m = 200;
    double[] sigmas = { 0.0004, 0.0005, 0.0006 };

    // Create a parametrized model with given shape
    Model M = total_var(n, m);
    Parameter sigma = M.getParameter("sigma");
    Parameter f     = M.getParameter("f");
    Variable  ucore = M.getVariable("u").slice(new int[] {0,0}, new int[] {n,m});

    // Example: Linear signal with Gaussian noise    
    double[][] signal = new double[n][m];
    double[][] noise  = new double[n][m];
    double[][] fVal   = new double[n][m];
    double[][] sol    = new double[n][m];

    for(int i=0; i<n; i++) for(int j=0; j<m; j++) {
      signal[i][j] = 1.0*(i+j)/(n+m);
      noise[i][j] = randGen.nextGaussian() * 0.08;
      fVal[i][j] = Math.max( Math.min(1.0, signal[i][j] + noise[i][j]), .0 );
    }
   
    // Set value for f
    f.setValue(fVal);

    for(int iter=0; iter<3; iter++) {
      // Set new value for sigma and solve
      sigma.setValue(sigmas[iter]*n*m);

      M.solve();

      // Retrieve solution from ucore
      double[] ucoreLev = ucore.level();
      for(int i=0; i<n; i++) for(int j=0; j<m; j++)
        sol[i][j] = ucoreLev[i*n+m];

      // Use the solution
      // ...

      System.out.printf("rel_sigma = %f  total_var = %.3f\n", sigmas[iter], M.primalObjValue());
    }

    M.dispose();
  }
}