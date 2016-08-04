%% Scott Kelly Example
clear

% Directories should be the same, and should point to **sba** folder 
dir_sba = 'C:\Users\dylan93\Documents\GitHub\Quad_CV-SLAM\Kelly_SLAM_Matlab\sba-1.6';
dir_external = 'C:\Users\dylan93\Documents\GitHub\Quad_CV-SLAM\Kelly_SLAM_Matlab\sba-1.6';
% Cd into the **matlab** folder inside the **sba** folder
cd 'C:\Users\dylan93\Documents\GitHub\Quad_CV-SLAM\Kelly_SLAM_Matlab\sba-1.6\matlab'

sba_files = ['sba.c ../sba_chkjac.c ../sba_levmar_wrap.c ../sba_levmar.c ../sba_lapack.c ../sba_crsm.c' ];
 
ext = mexext;
ext = ext(5:6);

% Adding libraries to the linker
library_flags = [ '-D_WIN32 -I' dir_sba ' -L' dir_sba ' -L' dir_external 'blasLapack/win' ext ' -output sba.' mexext ' '];
 
eval([ 'mex -largeArrayDims LINKFLAGS="$LINKFLAGS /NODEFAULTLIB:libcmt.lib" ' library_flags '-lclapack_nowrap -lf2c -lBLAS_nowrap ' sba_files ]);