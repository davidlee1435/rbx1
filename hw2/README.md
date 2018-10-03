### To Run:

In one terminal, run:
```
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<path_to_bug_file>
```

In another terminal, run:
```
$  python hw2.py cmd_vel:=cmd_vel_mux/input/teleop
```

This will run the main pathfinder script
