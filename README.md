# ARIAC 2019 Entry
This package is the first part of an [ARIAC 2019](https://bitbucket.org/osrf/ariac/wiki/2019/Home) entry (only 4 years late).
It starts the competition, and processes orders as well as printing information about them to the screen.
On the reception of an order, the location where the first material in that order can be found is printed,
and its coordinates in the camera and robot's reference frames is printed.

## Usage
```shell
source /opt/ros/<version>/setup.sh
source devel/setup.zsh # assuming you are in the top-level catkin dir
roslaunch ariac_entry entry.launch
```

> here's a block quote.