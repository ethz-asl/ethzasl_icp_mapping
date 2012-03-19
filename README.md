This [ROS] stack provides 3D mapping tools for robotic applications.
Look into the individual packages for more information about these tools.

Compilation
-----------

Get the [ethzasl_mapping] stack:

	git clone --recursive git://github.com/ethz-asl/ros-mapping.git ethzasl_mapping

Make sure that it is included in your `ROS_PACKAGE_PATH`:

	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ethzasl_mapping

Then, to compile all packages, just run:

	rosmake --rosdep-install ethzasl_mapping

Information about this stack is available on the [ROS] wiki at [http://www.ros.org/wiki/ethzasl_mapping](http://www.ros.org/wiki/ethzasl_mapping).

[ROS]: http://www.ros.org
[ethzasl_mapping]: http://www.ros.org/wiki/ethzasl_mapping
