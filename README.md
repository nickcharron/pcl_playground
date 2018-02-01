# pcl_playground
This repo is for learning and testing work with with PCL.

Here are some of the things included:
 - iterative_closest_point.cpp :
      identical code that is in PCL tutorials, this is just for playing around
      the code and figuring out how everything works

 - incremental_iterative_closest_point.cpp:
      This is a custom version of the incremental ICP code. I wrote it from
      scratch to suit my needs and do everything the way I want.
      It reads PCD point cloud files based on the naming, and plots the clouds
      at each step including one viewport with each new scan compared to old
      scan and one viewport showing the combined scan. You can keep visualizing
      it step by step as it incrementally adds your clouds.
