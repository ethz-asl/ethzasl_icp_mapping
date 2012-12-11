This [ROS] stack provides a real-time 2D and 3D [ICP]-based SLAM system that
can fit a large variety of robots and application scenarios, without any code
change or recompilation.

Compilation
-----------

Get the [ethzasl_icp_mapping] stack:

	git clone --recursive git://github.com/ethz-asl/ethzasl_icp_mapping

Make sure that it is included in your `ROS_PACKAGE_PATH`:

	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ethzasl_icp_mapping

Install dependencies:

	rosdep install ethzasl_icp_mapping

Build stack:

	rosmake ethzasl_icp_mapping

Information about this stack is available on the [ROS] wiki at [http://www.ros.org/wiki/ethzasl_icp_mapping](http://www.ros.org/wiki/ethzasl_icp_mapping).

[ROS]: http://www.ros.org
[ICP]: http://en.wikipedia.org/wiki/Iterative_Closest_Point
[ethzasl_icp_mapping]: http://www.ros.org/wiki/ethzasl_icp_mapping
