FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_gensrv_java_rosmln"
  "../srv_gen/java/ros/pkg/rosmln/srv/MLNInterface.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_java_rosmln.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
