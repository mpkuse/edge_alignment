# Edge Alignment - CERES

Reimplementation of edge alignment [1] using CERES solver

## Required Packages
- Boost
- Eigen3
- CERES
- OpenCV
- ROS

[1] Kuse M., Shen S. “Robust Camera Motion Estimation using Direct Edge Alignment and Sub-gradient Method“. In Proc. of IEEE International Conference on Robotics and Automation (ICRA), 2016 in Stockholm, Sweden.

[2] CERES http://ceres-solver.org/

## Sample Alignment
![](readme/output_7ewdbs.gif)


## Note
This repo was an attempt for me to have a standalone
implementation of edge alignment. This is not ready yet (i didn't find time to complete it).
It has some bugs that need fixing. In particular, this uses google's ceres solver and the issue is in defining a grid-based cost function. Contributions welcome!

I believe, Yongyen [https://github.com/ygling2008/direct_edge_imu](https://github.com/ygling2008/direct_edge_imu) in his repo had
fixed it and is usable. You are better off trying to
run his code.

You may look at [https://kusemanohar.wordpress.com/2015/09/14/icra2015-submission/](https://kusemanohar.wordpress.com/2015/09/14/icra2015-submission/)
for more resources on this project.

If you are keen on using this, I suggest you implement the gradient decent yourself for this. Once you have a point cloud it is really not that hard. I can try and implement a shortcode but cannot guarantee.


