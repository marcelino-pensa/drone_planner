# drone_planner

Motion planner solutions for quadcopters.

## Build Requirements
1) Install [Eigen](http://eigen.tuxfamily.org)
2) Clone this repository to a catkin workspace:

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/marcelino-pensa/drone_planner
``

If you don't need the minimum time motion planner, just run ```catkin_make``` in ```~/catkin_ws```. Otherwise, do the following:

3) Install OSQP

```
cd ~/catkin_ws/src/drone_planner/dependencies/P4/dependencies/osqp
git submode update --init --recursive
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
```

4) Install ECOS

```
cd ~/catkin_ws/src/drone_planner/dependencies/ecos
make
```

5) Set the ```COMPILE_MIN_TIME_SERVICE``` flag to ```TRUE``` in ```~/catkin_ws/src/drone_planner/CMakeLists.txt```.

6) Compile by running ```catkin_make``` in ```~/catkin_ws```.
