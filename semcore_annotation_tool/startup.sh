#! /bin/bash
exec roscore &
exec rosrun rosprolog run_with_prolog_env annotation_tool $(rospack find json_prolog)/bin/json_prolog &
