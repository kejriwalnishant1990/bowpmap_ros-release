## bowpmap_ros-release (indigo) - 0.1.2-1

The packages in the `bowpmap_ros-release` repository were released into the `indigo` distro by running `/usr/bin/bloom-release --rosdistro indigo --track indigo bowpmap_ros-release --edit` on `Sat, 08 Oct 2016 10:02:03 -0000`

The `bowpmap_ros` package was released.

Version of package(s) in repository `bowpmap_ros-release`:

- upstream repository: https://github.com/kejriwalnishant1990/bowpmap_ros.git
- release repository: unknown
- rosdistro version: `null`
- old version: `0.1.2-0`
- new version: `0.1.2-1`

Versions of tools used:

- bloom version: `0.5.22`
- catkin_pkg version: `0.2.10`
- rosdep version: `0.11.5`
- rosdistro version: `0.4.7`
- vcstools version: `0.1.38`


## bowpmap_ros-release (indigo) - 0.1.2-0

The packages in the `bowpmap_ros-release` repository were released into the `indigo` distro by running `/usr/bin/bloom-release --rosdistro indigo --track indigo bowpmap_ros-release --edit` on `Sat, 08 Oct 2016 09:49:35 -0000`

The `bowpmap_ros` package was released.

Version of package(s) in repository `bowpmap_ros-release`:

- upstream repository: https://github.com/kejriwalnishant1990/bowpmap_ros.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `0.1.2-0`

Versions of tools used:

- bloom version: `0.5.22`
- catkin_pkg version: `0.2.10`
- rosdep version: `0.11.5`
- rosdistro version: `0.4.7`
- vcstools version: `0.1.38`


# bowpmap_ros

BoWP-Map's ROS package.

For more information, demos and tutorials about this package, visit the [bowpmap_ros](http://wiki.ros.org/bowpmap_ros) page on the ROS wiki.

## Installation


### Build from source

This section shows how to install BoWP-Map ros-pkg on **ROS Hydro/Indigo** (Catkin build).

* **Install OpenCV from source in indigo/hydro**: If you want SURF/SIFT, you have to build OpenCV from source to have access to **nonfree** module. 
Install it in /usr/local (default) and the bowpmap_ros should link with it instead of the one installed in ROS. However, currently for SURF features
we are using opensurf which is already built in with bowpmap_ros. But still for other opencv requirements, you can download the opencv-2.4.9 according to
the tutorial given [here](http://www.samontab.com/web/2014/06/installing-opencv-2-4-9-in-ubuntu-14-04-lts/).

* Install flann library : Download flann library (Version : 1.8.4) from its [homepage](http://www.cs.ubc.ca/research/flann/).
	
	* Follow the below instructions to build the library:

	 ```bash
	$ unzip flann-1.8.4-src.zip -d ~/
	$ cd ~/flann-1.8.4-src
	$ mkdir build
	$ cd build
	$ cmake ..
	$ make
	```

Note that we are not using opencv flann library because it does not support incremental addition of points in the indexing data structures. However flann library has been updated with this feature but it is not incorporated in the opencv2.4.9 library.

* The next instructions assume that you have setup your ROS workspace using this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
The workspace path is ~/catkin_ws and your ~/.bashrc contains:

  ```bash
source /opt/ros/hydro/setup.bash
source ~/catkin_ws/devel/setup.bash
```

1. Install the BoWP-Map ros-pkg in your src folder of your Catkin workspace.

 ```bash
$ cd ~/catkin_ws/src
$ git clone -b prasun https://gitlab.com/nishant1990/bowpmap_ros.git 
$ cd ..
$ catkin_make

#### Update to new version

$ roscd bowpmap_ros
$ git pull origin master
$ cd ~/catkin_ws
$ catkin_make
```
