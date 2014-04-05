FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_genmsg_java_rosbuild"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_java_rosbuild.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
