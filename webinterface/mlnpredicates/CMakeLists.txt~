cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_find_ros_package(mlnpredicates)
set( ROSPRAC_PATH ${rosprac_PACKAGE_PATH} )
set( PRAC_HOME $ENV{PRAC_HOME} )

file(GLOB inputs scripts/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

add_java_source_dir(${PROJECT_SOURCE_DIR}/java)
