#ifndef CHAIN_LINK_ALIGNMENT_H_
#define CHAIN_LINK_ALIGNMENT_H_

#include <Eigen/Dense>
/*
Eigen::Quaternion View0 <<
           0,    -0.850651,    -0.525731,   0.00145556,
           1,            0,            0, -0.000636778,
           0,    -0.525731,     0.850651,     0.765217,
           0,            0,            0,            1;
View1.pcd<<
          0   -0.850651    0.525731 -0.00145556
         -1           0           0 0.000636778
          0   -0.525731   -0.850651    0.769927
          0           0           0           1
View2.pcd<<
1.11741e-17   -0.850651   -0.525731  0.00145556
         -1           0           0 0.000636778
          0    0.525731   -0.850651    0.769927
          0           0           0           1
View3.pcd<<
  -0.850651   -0.525731           0 0.000541676
          0           0          -1  0.00276865
   0.525731   -0.850651           0    0.767237
          0           0           0           1
View4.pcd<<
    0.850651    -0.525731            0 -0.000541676
           0            0           -1   0.00276865
    0.525731     0.850651            0     0.767237
           0            0            0            1
View5.pcd<<
    0.850651    -0.525731            0 -0.000541676
           0            0            1  -0.00276865
   -0.525731    -0.850651            0     0.767907
           0            0            0            1
View6.pcd<<
  -0.850651   -0.525731           0 0.000541676
          0           0           1 -0.00276865
  -0.525731    0.850651           0    0.767907
          0           0           0           1
View7.pcd<<
-1.11741e-17    -0.850651     0.525731  -0.00145556
           1            0            0 -0.000636778
           0     0.525731     0.850651     0.765217
           0            0            0            1
View8.pcd<<
          0          -1           0 5.45686e-11
  -0.525731           0    0.850651 -0.00202038
  -0.850651           0   -0.525731    0.769569
          0           0           0           1
View9.pcd<<
          0          -1           0 5.45686e-11
  -0.525731           0   -0.850651  0.00268993
   0.850651           0   -0.525731    0.768486
          0           0           0           1
View10.pcd<<
          0          -1           0 5.45686e-11
   0.525731           0   -0.850651  0.00202038
   0.850651           0    0.525731    0.765575
          0           0           0           1
View11.pcd<<
          0          -1           0 5.45686e-11
   0.525731           0    0.850651 -0.00268993
  -0.850651           0    0.525731    0.766658
          0           0           0           1
View12.pcd<<
  -0.425325   -0.587785   -0.688191   0.0021762
   0.850651           0   -0.525731 0.000913888
   0.309017   -0.809017         0.5     0.76784
          0           0           0           1
View13.pcd<<
   0.425325   -0.587785   -0.688191  0.00163452
   0.850651           0    0.525731 -0.00199724
  -0.309017   -0.809017         0.5    0.768233
          0           0           0           1
View14.pcd<<
           0            0           -1   0.00276865
           1            0            0 -0.000636778
           0           -1            0     0.769421
           0            0            0            1
View15.pcd<<
    0.425325    -0.587785     0.688191   -0.0021762
   -0.850651            0     0.525731 -0.000913888
   -0.309017    -0.809017         -0.5     0.771002
           0            0            0            1
View16.pcd<<
  -0.425325   -0.587785    0.688191 -0.00163452
  -0.850651           0   -0.525731  0.00199724
   0.309017   -0.809017        -0.5    0.770608
          0           0           0           1
View17.pcd<<
    -0.16246    -0.951057     0.262866 -0.000624331
   -0.850651            0    -0.525731   0.00199724
         0.5    -0.309017    -0.809017     0.771342
           0            0            0            1
View18.pcd<<
          0          -1           0 5.45686e-11
         -1           0           0 0.000636778
          0           0          -1    0.772189
          0           0           0           1
View19.pcd<<
    0.16246   -0.951057   -0.262866 0.000624331
  -0.850651           0   -0.525731  0.00199724
        0.5    0.309017   -0.809017    0.771342
          0           0           0           1
View20.pcd<<
     0.16246    -0.951057     0.262866 -0.000831233
   -0.850651            0     0.525731 -0.000913888
        -0.5    -0.309017    -0.809017     0.771979
           0            0            0            1
View21.pcd<<
    -0.16246    -0.951057    -0.262866  0.000831233
   -0.850651            0     0.525731 -0.000913888
        -0.5     0.309017    -0.809017     0.771979
           0            0            0            1
View22.pcd<<
    0.16246   -0.951057   -0.262866 0.000624331
   0.850651           0    0.525731 -0.00199724
       -0.5   -0.309017    0.809017    0.767499
          0           0           0           1
View23.pcd<<
           0           -1            0  5.45686e-11
           1            0            0 -0.000636778
           0            0            1     0.766652
           0            0            0            1
View24.pcd<<
    -0.16246    -0.951057     0.262866 -0.000624331
    0.850651            0     0.525731  -0.00199724
        -0.5     0.309017     0.809017     0.767499
           0            0            0            1
View25.pcd<<
   -0.16246   -0.951057   -0.262866 0.000831233
   0.850651           0   -0.525731 0.000913888
        0.5   -0.309017    0.809017    0.766863
          0           0           0           1
View26.pcd<<
     0.16246    -0.951057     0.262866 -0.000831233
    0.850651            0    -0.525731  0.000913888
         0.5     0.309017     0.809017     0.766863
           0            0            0            1
View27.pcd<<
  0.425325  -0.587785  -0.688191 0.00163452
 -0.850651          0  -0.525731 0.00199724
  0.309017   0.809017       -0.5   0.770608
         0          0          0          1
View28.pcd<<
   -0.425325    -0.587785    -0.688191    0.0021762
   -0.850651            0     0.525731 -0.000913888
   -0.309017     0.809017         -0.5     0.771002
           0            0            0            1
View29.pcd<<
          0           0          -1  0.00276865
         -1           0           0 0.000636778
          0           1           0    0.769421
          0           0           0           1
View30.pcd<<
  -0.425325   -0.587785    0.688191 -0.00163452
   0.850651           0    0.525731 -0.00199724
  -0.309017    0.809017         0.5    0.768233
          0           0           0           1
View31.pcd<<
   0.425325   -0.587785    0.688191  -0.0021762
   0.850651           0   -0.525731 0.000913888
   0.309017    0.809017         0.5     0.76784
          0           0           0           1
View32.pcd<<
  -0.467086   -0.866025   -0.178411 0.000791388
   0.356822           0   -0.934172  0.00235918
   0.809017        -0.5    0.309017     0.76805
          0           0           0           1
View33.pcd<<
   -0.467086    -0.866025     0.178411 -0.000196527
   -0.356822            0    -0.934172   0.00281361
    0.809017         -0.5    -0.309017     0.769761
           0            0            0            1
View34.pcd<<
          0          -1           0 5.45686e-11
          0           0          -1  0.00276865
          1           0           0    0.768784
          0           0           0           1
View35.pcd<<
   0.467086   -0.866025   -0.178411 0.000196527
  -0.356822           0   -0.934172  0.00281361
   0.809017         0.5   -0.309017    0.769761
          0           0           0           1
View36.pcd<<
    0.467086    -0.866025     0.178411 -0.000791388
    0.356822            0    -0.934172   0.00235918
    0.809017          0.5     0.309017      0.76805
           0            0            0            1
View37.pcd<<
    0.467086    -0.866025     0.178411 -0.000791388
   -0.356822            0     0.934172  -0.00235918
   -0.809017         -0.5    -0.309017     0.770792
           0            0            0            1
View38.pcd<<
   0.467086   -0.866025   -0.178411 0.000196527
   0.356822           0    0.934172 -0.00281361
  -0.809017        -0.5    0.309017     0.76908
          0           0           0           1
View39.pcd<<
          0          -1           0 5.45686e-11
          0           0           1 -0.00276865
         -1           0           0    0.770058
          0           0           0           1
View40.pcd<<
   -0.467086    -0.866025     0.178411 -0.000196527
    0.356822            0     0.934172  -0.00281361
   -0.809017          0.5     0.309017      0.76908
           0            0            0            1
View41.pcd<<
  -0.467086   -0.866025   -0.178411 0.000791388
  -0.356822           0    0.934172 -0.00235918
  -0.809017         0.5   -0.309017    0.770792
          0           0           0           1
*/
#endif /* CHAIN_LINK_ALIGNMENT_H_ */
