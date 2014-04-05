FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_genmsg_java_rosgraph_msgs"
  "../msg_gen/java/ros/pkg/rosgraph_msgs/msg/Clock.java"
  "../msg_gen/java/ros/pkg/rosgraph_msgs/msg/Log.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_java_rosgraph_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
