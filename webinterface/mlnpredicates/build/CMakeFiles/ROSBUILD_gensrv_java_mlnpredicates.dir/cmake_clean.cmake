FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_gensrv_java_mlnpredicates"
  "../srv_gen/java/ros/pkg/mlnpredicates/srv/MLNPredicates.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_java_mlnpredicates.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
