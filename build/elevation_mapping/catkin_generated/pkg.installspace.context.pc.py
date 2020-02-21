# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "grid_map_core;grid_map_ros;grid_map_msgs;pcl_ros;roscpp;message_filters;sensor_msgs;std_msgs;tf;tf_conversions;eigen_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lelevation_mapping_library".split(';') if "-lelevation_mapping_library" != "" else []
PROJECT_NAME = "elevation_mapping"
PROJECT_SPACE_DIR = "/home/ieee/catkin_ws/install"
PROJECT_VERSION = "0.3.0"
