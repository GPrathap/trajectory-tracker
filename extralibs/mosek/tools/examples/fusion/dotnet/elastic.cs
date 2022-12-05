// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
// File:      elastic.cs
//
// Purpose: Demonstrates model parametrization on the example of an elastic net linear regression:
//
//          min_x  |Ax-b|_2 + lambda1*|x|_1 + lambda2*|x|_2

using System;
using mosek.fusion;

namespace mosek.fusion.example
{
  public class elastic
  {
    // Construct the model with parameters b, lambda1, lambda2
    // and with prescribed matrix A
    public static Model initializeModel(int m, int n, double[,] A) {
      Model M = new Model();
      Variable x = M.Variable("x", n);

      // t >= |Ax-b|_2 where b is a parameter
      Parameter b = M.Parameter("b", m);
      Variable  t = M.Variable();
      M.Constraint(Expr.Vstack(t, Expr.Sub(Expr.Mul(A, x), b)), Domain.InQCone());

      // p_i >= |x_i|, i=1..n
      Variable p = M.Variable(n);
      M.Constraint(Expr.Hstack(p, x), Domain.InQCone());

      // q >= |x|_2
      Variable q = M.Variable();
      M.Constraint(Expr.Vstack(q, x), Domain.InQCone());

      // Objective, parametrized with lambda1, lambda2
      // t + lambda1*sum(p) + lambda2*q
      Parameter lambda1 = M.Parameter("lambda1");
      Parameter lambda2 = M.Parameter("lambda2");
      Expression obj = Expr.Add(new Expression[] {t, Expr.Mul(lambda1, Expr.Sum(p)), Expr.Mul(lambda2, q)});
      M.Objective(ObjectiveSense.Minimize, obj);

      // Return the ready model
      return M;
    }

    public static void smallExample() {
      //Create a small example
      int m = 4;
      int n = 2;
      double[,] A = { {1.0,   2.0},
                      {3.0,   4.0},
                      {-2.0, -1.0},
                      {-4.0, -3.0} };
      double[] sol;
      Model M = initializeModel(m, n, A);

      // For convenience retrieve some elements of the model
      Parameter b = M.GetParameter("b");
      Parameter lambda1 = M.GetParameter("lambda1");
      Parameter lambda2 = M.GetParameter("lambda2");
      Variable x = M.GetVariable("x");

      // First solve
      b.SetValue(new double[]{0.1, 1.2, -1.1, 3.0});
      lambda1.SetValue(0.1);
      lambda2.SetValue(0.01);

      M.Solve();
      sol = x.Level();
      Console.WriteLine("Objective {0}, solution {1}, {2}", M.PrimalObjValue(), sol[0], sol[1]);

      // Increase lambda1
      lambda1.SetValue(0.5);
      
      M.Solve();
      sol = x.Level();
      Console.WriteLine("Objective {0}, solution {1}, {2}", M.PrimalObjValue(), sol[0], sol[1]);

      // Now change the data completely
      b.SetValue(new double[] {1.0, 1.0, 1.0, 1.0});
      lambda1.SetValue(0.0);
      lambda2.SetValue(0.0);
      
      M.Solve();
      sol = x.Level();
      Console.WriteLine("Objective {0}, solution {1}, {2}", M.PrimalObjValue(), sol[0], sol[1]);

      // And increase lamda2
      lambda2.SetValue(1.4145);
      
      M.Solve();
      sol = x.Level();
      Console.WriteLine("Objective {0}, solution {1}, {2}", M.PrimalObjValue(), sol[0], sol[1]);

      M.Dispose();
    }

    public static void Main() {
      smallExample();
    }
  }
}
