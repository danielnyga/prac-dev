FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_genmsg_java_mlnpredicates"
  "../msg_gen/java/ros/pkg/mlnpredicates/msg/MLNPredicate.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_java_mlnpredicates.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
