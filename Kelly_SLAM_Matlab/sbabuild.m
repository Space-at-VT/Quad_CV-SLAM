%% Scott Kelly Example
clear

dir_sba = 'C:\Users\ianelli\Documents\MATLAB\KELLY_SLAM_CV\sba-1.6\sba-1.6';
dir_external = 'C:\Users\ianelli\Documents\MATLAB\KELLY_SLAM_CV\sba-1.6\sba-1.6';

sba_files = [ 'sba.c ../sba_chkjac.c ../sba_levmar_wrap.c ../sba_levmar.c ' ...
    '../sba_lapack.c ../sba_crsm.c' ];

ext = mexext;
ext = ext(5:6);

library_flags = [ '-D_WIN32 -I' dir_sba ' -L' dir_sba ...
    ' -L' dir_external 'blasLapack/win' ext ' -output sba.' mexext ' '];

eval([ '-v mex -largeArrayDims LINKFLAGS="$LINKFLAGS /NODEFAULTLIB:libcmt.lib" ' ...
    library_flags '-lclapack_nowrap -lf2c -lBLAS_nowrap ' sba_files ]);
