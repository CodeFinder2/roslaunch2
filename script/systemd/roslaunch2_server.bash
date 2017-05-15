#!/bin/bash -l

# The special shebang line in this script is important in order to execute this in a login shell,
# effectively sourcing the ~/.bashrc and setting up Python and ROS correctly. Since this is started
# in a login shell, no output is visible in "sudo journalctl -ru $SERVICE_UNIT_NAME"

# Run roslaunch2 server instance for remote execution:
roslaunch2_server
