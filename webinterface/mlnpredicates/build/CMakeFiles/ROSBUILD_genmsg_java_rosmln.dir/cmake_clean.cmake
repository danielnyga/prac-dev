FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/mlnpredicates/msg"
  "../src/mlnpredicates/srv"
  "CMakeFiles/ROSBUILD_genmsg_java_rosmln"
  "../msg_gen/java/ros/pkg/rosmln/msg/AtomProbPair.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/MLNQuery.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/MLNConfig.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/AtomTruthValuePair.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/MLNDatabase.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/MRFClique.java"
  "../msg_gen/java/ros/pkg/rosmln/msg/MLNInference.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_java_rosmln.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
