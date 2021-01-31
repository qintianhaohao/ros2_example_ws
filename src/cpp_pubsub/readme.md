# project 1:
put cpp_pubsub folder on ~/your_ws/src/

`cd ~/your_ws`

`colcon build --packages-select cpp_pubsub`

`ros2 run cpp_pubsub display_topic_statistics`

change statistic time out limit: locate to cpp_pubsub/src/topic_statistics_listener.cpp line:57
change string talker delay: locate to cpp_pubsub/include/topic_statistics_demo/string_talker_listener_nodes.hpp line:26

# project 2:
## compile
put "cpp_pubsub" and "py_pubsub" folder on ~/your_ws/src/

open a terminal:

`cd ~/your_ws`

`colcon build --packages-select cpp_pubsub`

`colcon build --packages-select py_pubsub`

## run
open first terminal:

`cd ~/your_ws`

`ros2 run py_pubsub people_detect`

open second terminal:

`cd ~/your_ws`

`ros2 run cpp_pubsub display_topic_statistics`

## result: 
topic statistics show a warning message when the number of people is greater than 7