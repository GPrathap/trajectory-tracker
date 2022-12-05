/*
  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.

  File:    opt_server_sync.cs

  Purpose :   Demonstrates how to use MOSEK OptServer
              to solve optimization problem synchronously
*/
using System;

namespace mosek.example
{
  class msgclass : mosek.Stream
  {
    public override void streamCB (string msg)
    {
      Console.Write ("{0}", msg);
    }
  }

  public class simple
  {
    public static void Main (string[] args)
    {
      if (args.Length < 2)
      {
        Console.WriteLine ("Missing arguments, syntax is:");
        Console.WriteLine ("  opt_server_sync inputfile serveraddr");
      }
      else
      {
        String inputfile = args[0];
        String addr      = args[1];

        mosek.rescode trm;

        using (mosek.Env env = new mosek.Env())
        {
          using (mosek.Task task = new mosek.Task(env))
          {
            task.set_Stream (mosek.streamtype.log, new msgclass ());

            task.readdata (inputfile);

            // Set OptServer URL
            task.putoptserverhost(addr);

            // Optimize 
            task.optimize (out trm);

            task.solutionsummary (mosek.streamtype.log);
          }
        }
      }
    }
  }
}