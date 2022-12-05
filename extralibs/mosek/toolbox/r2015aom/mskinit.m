function mskinit(islicstr,istr,usepause)
% Purpose: Generates the Mosek license file.
%
%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.

disp('Initialize the MOSEK license.');

if nargin<=2 | usepause
   disp('Press any key to continue.');
   pause;
end

if islicstr
   licensestr = istr;
else
   licensestr = [];

   disp(['License file: ',istr]);

   fid = fopen(istr,'rt');

   if fid~=-1
      disp('License file found');
      while ~feof(fid)
        line = fgets(fid);
        if strncmp(line,'LICENSESTR',10)
           p = findstr(line,'"');
           if length(p)==2
             licensestr = line((p(1)+1):(p(2)-1));
             break
           end
         end
      end
      fclose(fid);
   else
      error('Could not open license file');
   end
end

file = '';
path = which('mskinit.m');
l    = length(path);

fprintf('Path to mskinit.m: %s\n',path);

if l>=9
  if strcmp(path((l-8):l),'mskinit.m')
     file = [path(1:(l-9)),'msklic.m'];
  end
else
  fprintf('Unable to locate the file mskinit.m\n');
end

if length(file)

  f = fopen(file,'wt');

  fprintf(f,'function [licstr]=msklic\n');
  fprintf(f,'%% Purpose: Reports the MOSEK license string.\n\n');
  fprintf(f,'%% Copyright (c) MOSEK ApS, Denmark. All rights reserved.: EKA Consulting ApS, Denmark \n');
  fprintf(f,'licstr=''MOSEKLICENSE=%s'';\n',licensestr);

  fclose(f);

  disp('');
  disp('The MOSEK license is now initialized.');

else

  disp('Initializing of the MOSEK license was NOT succesful');
  disp('');
  disp('The reason is MOSEK could not not locate mskinit.m');
  disp('Did you remember to do');
  disp('');
  disp('addpath <root>mosetb\solvers');
  disp('');
  disp('where <root> repersents the directory where you installed MOSEK.');
  disp('Please consult the manual.');

end
