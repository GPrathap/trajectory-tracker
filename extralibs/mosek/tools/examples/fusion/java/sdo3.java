/*
  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.

  File :      sdo3.java

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

package com.mosek.fusion.examples;
import mosek.fusion.*;

public class sdo3 {

  // A helper method computing a semidefinite slice of a 3-dim variable
  public static Variable slice(Variable X, int d, int j) {
    return
      X.slice(new int[] {j,0,0}, new int[] {j+1,d,d})
       .reshape(new int[] {d,d});
  }

  public static void main(String[] args) throws SolutionError {

    // Sample input data
    int n = 100;
    int d = 4;
    int k = 3;
    double[] b = {9,10,11};
    double[][][] A = new double[n*k][d][d];
    for(int i=0; i<n*k; i++)
      for(int s1=0; s1<d; s1++)
        for(int s2=0; s2<=s1; s2++)
          A[i][s1][s2] = A[i][s2][s1] = Math.random();

    // Create a model with n semidefinite variables od dimension d x d
    Model M  = new Model("sdo3");
    try {
      Variable X = M.variable(Domain.inPSDCone(d, n));

      // Pick indexes of diagonal entries for the objective
      int[][] alldiag = new int[d*n][3];
      for(int j=0; j<n; j++) for(int s=0; s<d; s++) {
        alldiag[j*d+s][0] = j; 
        alldiag[j*d+s][1] = alldiag[j*d+s][2] = s;
      }
      M.objective(ObjectiveSense.Minimize, Expr.sum( X.pick(alldiag) ));

      // Each constraint is a sum of inner products
      // Each semidefinite variable is a slice of X
      for(int i=0; i< k; i++) {
        Expression[] addlist = new Expression[n];
        for(int j=0; j<n; j++) 
          addlist[j] = Expr.dot(A[i*n+j], slice(X, d, j));
        M.constraint(Expr.add(addlist), Domain.greaterThan(b[i]));
      }

      // Solve
      M.setLogHandler(new java.io.PrintWriter(System.out)); // Add logging
      M.writeTask("sdo3.ptf");                              // Save problem in readable format
      M.solve();

      // Get results. Each variable is a slice of X
      System.out.println("Contributing variables:");
      for(int j=0; j<n; j++) {
        double[] Xj = slice(X, d, j).level();
        double maxval = 0;
        for(int s=0; s<d*d; s++) maxval = Math.max(maxval, Xj[s]);
        if (maxval > 1e-6) {
          System.out.println("X" + j + "=");
          for(int s1=0; s1<d; s1++) {
            for(int s2=0; s2<d; s2++)
              System.out.print(Xj[s1*d+s1] + "  ");    
            System.out.println();
          }
        }
      }
    } 
    finally {
      M.dispose();
    }
  }
}