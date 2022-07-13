# IKBT
git clone https://github.com/uw-biorobotics/IKBT.git
ik_robots.py -> fill out DH params for PAPRAS robot
python3 ikSolver.py PAPRAS

mkdir -p rb_ws/src & cd rb_ws/src
git clone https://github.com/KavrakiLab/robowflex.git
git checkout cmake_fixes 
git clone https://github.com/KavrakiLab/robowflex_resources.git
catkin_init_workspace
catkin build

# dart
sudo add-apt-repository ppa:dartsim/ppa
sudo apt-get update
sudo apt-get install libdart6-dev
sudo apt-get install libdart6-collision-bullet-dev
catkin build robowflex_dart 

# fetch ros 
git clone https://github.com/fetchrobotics/fetch_ros
sudo apt-get install ros-noetic-costmap-2d
cd fetch_depth_layer/ & touch CATKIN_IGNORE <!-- Need OpenCV 3.2 -->
catkin build fetch_ros

# gnu plot 
sudo apt-get install gnuplot

rosrun robowflex_library fetch_benchmark 