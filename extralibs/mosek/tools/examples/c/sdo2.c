/*
  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.

  File :      sdo2.c

  Purpose :   Solves the semidefinite problem with two symmetric variables:

                 min   <C1,X1> + <C2,X2>
                 st.   <A1,X1> + <A2,X2> = b
                             (X2)_{1,2} <= k
                
                 where X1, X2 are symmetric positive semidefinite,

                 C1, C2, A1, A2 are assumed to be constant symmetric matrices,
                 and b, k are constants.
*/

#include <stdio.h>
#include "mosek.h"    /* Include the MOSEK definition file.  */

static void MSKAPI printstr(void *handle,
                            const char str[])
{
  printf("%s", str);
} /* printstr */

int main(int argc, const char *argv[])
{
  MSKrescodee  r;

  /* Input data */
  MSKint32t    numbarvar = 2;
  MSKint32t    dimbarvar[] = {3, 4};          /* Dimension of semidefinite variables */

  /* Objective coefficients concatenated */
  MSKint32t    Cj[] = { 0, 0, 1, 1, 1, 1 };   /* Which symmetric variable (j) */
  MSKint32t    Ck[] = { 0, 2, 0, 1, 1, 2 };   /* Which entry (k,l)->v */
  MSKint32t    Cl[] = { 0, 2, 0, 0, 1, 2 };
  MSKrealt     Cv[] = { 1.0, 6.0, 1.0, -3.0, 2.0, 1.0 };

  /* Equality constraints coefficients concatenated */
  MSKint32t    Ai[] = { 0, 0, 0, 0, 0, 0 };   /* Which constraint (i = 0) */
  MSKint32t    Aj[] = { 0, 0, 0, 1, 1, 1 };   /* Which symmetric variable (j) */
  MSKint32t    Ak[] = { 0, 2, 2, 1, 1, 3 };   /* Which entry (k,l)->v */
  MSKint32t    Al[] = { 0, 0, 2, 0, 1, 3 };
  MSKrealt     Av[] = { 1.0, 1.0, 2.0, 1.0, -1.0, -3.0 };

  /* The second constraint - one-term inequality */
  MSKint32t    A2i = 1;                        /* Which constraint (i = 1) */
  MSKint32t    A2j = 1;                        /* Which symmetric variable (j = 1) */
  MSKint32t    A2k = 1;                        /* Which entry A(1,0) = A(0,1) = 0.5 */
  MSKint32t    A2l = 0;
  MSKrealt     A2v = 0.5;

  /* Constraint bounds and values */
  MSKint32t    numcon = 2;
  MSKboundkeye bkc[] = { MSK_BK_FX, MSK_BK_UP };
  double       blc[] = { 23.0, -MSK_INFINITY };
  double       buc[] = { 23.0, -3.0 };

  MSKint32t    i, j, dim;
  MSKrealt     *barx;
  MSKenv_t     env = NULL;
  MSKtask_t    task = NULL;

  /* Create the mosek environment. */
  r = MSK_makeenv(&env, NULL);

  if (r == MSK_RES_OK)
  {
    /* Create the optimization task. */
    r = MSK_maketask(env, 0, 0, &task);

    if (r == MSK_RES_OK)
    {
      MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

      /* Append empty constraints.
       The constraints will initially have no bounds. */
      if (r == MSK_RES_OK)
        r = MSK_appendcons(task, numcon);

      /* Append semidefinite variables. */
      if (r == MSK_RES_OK)
        r = MSK_appendbarvars(task, numbarvar, dimbarvar);

      /* Set objective (6 nonzeros).*/
      if (r == MSK_RES_OK)
        r = MSK_putbarcblocktriplet(task, 6, Cj, Ck, Cl, Cv);

      /* Set the equality constraint (6 nonzeros).*/
      if (r == MSK_RES_OK)
        r = MSK_putbarablocktriplet(task, 6, Ai, Aj, Ak, Al, Av);

      /* Set the inequality constraint (1 nonzero).*/
      if (r == MSK_RES_OK)
        r = MSK_putbarablocktriplet(task, 1, &A2i, &A2j, &A2k, &A2l, &A2v);

      /* Set constraint bounds */
      if (r == MSK_RES_OK)
        r = MSK_putconboundslice(task, 0, 2, bkc, blc, buc);

      if (r == MSK_RES_OK)
      {
        MSKrescodee trmcode;

        /* Run optimizer */
        r = MSK_optimizetrm(task, &trmcode);
        MSK_solutionsummary(task, MSK_STREAM_MSG);

        if (r == MSK_RES_OK)
        {
          MSKsolstae solsta;

          MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

          switch (solsta)
          {
            case MSK_SOL_STA_OPTIMAL:

              /* Retrieve the soution for all symmetric variables */
              printf("Solution (lower triangular part vectorized):\n");
              for(i = 0; i < numbarvar; i++) {
                dim = dimbarvar[i] * (dimbarvar[i] + 1) / 2;
                barx = (MSKrealt *) MSK_calloctask(task, dim, sizeof(MSKrealt));

                MSK_getbarxj(task, MSK_SOL_ITR, i, barx);
  
                printf("X%d: ", i + 1);
                for (j = 0; j < dim; ++j)
                  printf("%.3f ", barx[j]);
                printf("\n");

                MSK_freetask(task, barx);                
              }

              break;

            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER:
              printf("Primal or dual infeasibility certificate found.\n");
              break;

            case MSK_SOL_STA_UNKNOWN:
              printf("The status of the solution could not be determined. Termination code: %d.\n", trmcode);
              break;

            default:
              printf("Other solution status.");
              break;
          }
        }
        else
        {
          printf("Error while optimizing.\n");
        }
      }

      if (r != MSK_RES_OK)
      {
        /* In case of an error print error code and description. */
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];

        printf("An error occurred while optimizing.\n");
        MSK_getcodedesc(r,
                        symname,
                        desc);
        printf("Error %s - '%s'\n", symname, desc);
      }
    }
    /* Delete the task and the associated data. */
    MSK_deletetask(&task);
  }

  /* Delete the environment and the associated data. */
  MSK_deleteenv(&env);

  return (r);
} /* main */