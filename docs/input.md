<!--
This includes your top-level README as you index page i.e. homepage.

This will not be the best approach for all exemplars, so feel free to customise
your index page as you see fit.
-->
# Input files explanation

## Kinematics related
-`pos_act.txt`
A list of calculated robot joint 4 position in the camera frame.<br />
-`pos_des.txt`
A list of recorded robot joint 4 position in the base robot frame.<br />
-`q_history.txt`
It stores the joint angle of the robot when it was moved to different poses.

## Computer vision related
-`input/Acusense_RGB_K.txt`
It stores the Intrinsic parameters of the colour lens of the Acusense camera.<br />
-`input/ExtrinsicMat.txt`
It stores the Extrinsic parameters that consists the transformation matrix between the depth and colour lens of the Acusense camera.<br />
-`input/IR/`
It contains the infrared frames of the scene captured by the camera as the robot was moved to different poses.<br />
-`input/RGB/`
It contains the colour frames of the scene captured by the camera as the robot was moved to different poses.<br />
-`input/pcd/`
It contains the point cloud captured by the camera as the robot was moved to different poses.
<!-- Add more files in the `docs/` directory for them to be automatically
included in the Mkdocs documentation -->
