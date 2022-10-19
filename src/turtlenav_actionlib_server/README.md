# turtlenav_actionlib_server

This is the server component, this publishes / listens to turtlesim's cmd_vel and pose.

Run the server using:

```bash
rosrun turtlenav_actionlib_server server.py
```

There are two scripts to demo the package:

- `input_client.py`: prompt for coordinates to move turtle to
- `square_client.py`: move in a set square path once
