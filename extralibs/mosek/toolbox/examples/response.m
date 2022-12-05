%%
%  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File:      response.m
%
%  Purpose:   This example demonstrates proper response handling
%             for problems solved with the interior-point optimizers.
%

function response(inputfile)

  cmd = sprintf('read(%s)', inputfile)
  % In this example we read the problem from file
  [r, res] = mosekopt(cmd)

  % Read was successful
  if strcmp(res.rcodestr, 'MSK_RES_OK')

      prob = res.prob;
      param = []

      % (Optionally) Uncomment the next line to get solution status Unknown
      % param.MSK_IPAR_INTPNT_MAX_ITERATIONS = 1

      % Perform the optimization.
      [r, res] = mosekopt('minimize', prob, param); 

      % Expected result: The solution status of the interior-point solution is optimal.

      % Check if we have non-error termination code or OK
      if isempty(strfind(res.rcodestr, 'MSK_RES_ERR'))
      
          solsta = strcat('MSK_SOL_STA_', res.sol.itr.solsta)

          if strcmp(solsta, 'MSK_SOL_STA_OPTIMAL')
              fprintf('An optimal interior-point solution is located:\n');
              res.sol.itr.xx
              
          elseif strcmp(solsta, 'MSK_SOL_STA_DUAL_INFEASIBLE_CER')
              fprintf('Dual infeasibility certificate found.');

          elseif strcmp(solsta, 'MSK_SOL_STA_PRIMAL_INFEASIBLE_CER')
              fprintf('Primal infeasibility certificate found.');

          elseif strcmp(solsta, 'MSK_SOL_STA_UNKNOWN') 
              % The solutions status is unknown. The termination code 
              % indicates why the optimizer terminated prematurely. 
              fprintf('The solution status is unknown.\n');
              fprintf('Termination code: %s (%d) %s.\n', res.rcodestr, res.rcode, res.rmsg);  
          else
            fprintf('An unexpected solution status is obtained.');
          end
      
      else
          fprintf('Error during optimization: %s (%d) %s.\n', res.rcodestr, res.rcode, res.rmsg);  
      end

  else
      fprintf('Could not read input file, error: %s (%d) %s.\n', res.rcodestr, res.rcode, res.rmsg)
  end

end