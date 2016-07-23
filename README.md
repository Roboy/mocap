## markertracker
tracks a marker model consisting of 4 UV-leds 

## dependencies 
Eigen, glew, SFML, image magick, assimp, opengl, OpenCV, sdformat, pcl, boost, ros, ncurses
### on Ubuntu 14.04
please follow the install instructions for [ros jade](http://wiki.ros.org/jade/Installation/Ubuntu) before continueing.
```
#!bash
sudo apt-get install libeigen3-dev libglew-dev libsfml-dev libmagick++-dev libassimp-dev libglm-dev libopencv-dev libpcl-1.7-all-dev ros-jade-controller-manager mercurial libncurses-dev
```
### ignition math 2
```
#!bash
hg clone https://bitbucket.org/ignitionrobotics/ign-math
cd ign-math
hg up ign-math2
mkdir build
cd build
cmake ..
make -j4 
sudo make install
```
### sdformat 4
```
#!bash
hg clone https://bitbucket.org/osrf/sdformat
cd sdformat
hg up sdf4
mkdir build
cd build
sudo apt-get install libruby
cmake ..
make -j4
sudo make install
```
## checkout 
```
#!bash
git clone https://github.com/Roboy/mocap
cd path/to/mocap
git submodule init
git submoulde update
```
## build
Wheather the build is on host or raspberry pi is checked in the CMakeLists.txt and only the respective code is build. So just run:
```
#!bash
cd path/to/mocap
catkin_make
```
## run
###On the host 
add to ~/.bashrc your IP address (you can check you IP with ifconfig), eg:
```
#!bash
export ROS_MASTER_URI=http://192.168.2.106:113311
```
Then start the tracking node:
```
#!bash
cd path/to/mocap
source devel/setup.bash
rosrun tracking_node tracking_node
```
###On the raspberry pi 
enter root shell (for access to camera), export same IP as your host and start the node:
```
#!bash
cd path/to/mocap/devel/lib/vision_node
sudo -s
export ROS_MASTER_URI=http://192.168.2.106:113311
./vision_node
```
