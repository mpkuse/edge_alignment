# Standalone Edge ALignment

rgb-d folder contains some grabs from TUM-RGBD sequence. Demonstrates the edge alignment
on using 2 images and depth map from 1 image.

Can easily adopt this code in your application. The core problem is setup
as non-linear least squares and solved using CERES-solver. The user of this code
just need to get the 3d points from im1 and the distance transform of im2.
(Work in progress)

## How to run ?
```
$ mkdir build
$ cmake ..
$ make
$ ./edge_align
```


## How long it takes to run per image pair ?
Roughly speaking, with my inefficient ceres implementation takes
~60ms with 1500 residue terms of a 640x480 image and ~30 iterations.
There definately is a lot of potential
to improve its performance. Contributions welcome!

Here is my solver summary:
```
Solver Summary (v 1.12.0-eigen-(3.2.92)-lapack-suitesparse-(4.4.6)-cxsparse-(3.1.4)-openmp)

                                     Original                  Reduced
Parameter blocks                            2                        2
Parameters                                  7                        7
Effective parameters                        6                        6
Residual blocks                          1482                     1482
Residual                                 1482                     1482

Minimizer                        TRUST_REGION

Dense linear algebra library            EIGEN
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver                        DENSE_QR                 DENSE_QR
Threads                                     1                        1
Linear solver threads                       1                        1
Linear solver ordering              AUTOMATIC                        2

Cost:
Initial                          8.743202e+00
Final                            5.418352e-01
Change                           8.201366e+00

Minimizer iterations                       30
Successful steps                           30
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0001

  Residual evaluation                  0.0089
  Jacobian evaluation                  0.0444
  Linear solver                        0.0015
Minimizer                              0.0556

Postprocessor                          0.0000
Total                                  0.0557

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 8.861920e-07 <= 1.000000e-06)

Final Guess : :YPR=(-0.32,1.52,2.50)  :TxTyTz=(-0.01,0.00,-0.05)
```
