## markertracker
tracks a marker model consisting of 4 UV-leds 

## dependencies 
Eigen, glew, SFML, image magick, assimp, opengl, OpenCV, sdformat, pcl, boost, ros
### on Ubuntu 14.04
please follow the install instructions for [ros jade](http://wiki.ros.org/jade/Installation/Ubuntu) before continueing.
```
#!bash
sudo apt-get install libeigen3-dev libglew-dev libsfml-dev libmagick++-dev libassimp-dev libglm-dev libopencv-dev libpcl-1.7-all-dev ros-jade-controller-manager mercurial
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
```
#!bash
cd path/to/mocap
catkin_make
```
## run
```
#!bash
cd path/to/mocap
source devel/setup.bash
rosrun mocap trackingNode
rosrun mocap visionNode
```
you need to add a subscriber to the ros topic '/visualization_marker', otherwise the tracking does not start. You can use a custom subscriber or rviz (in rviz click on add -> by topic -> marker, green oriented cubes should appear while tracking). 
