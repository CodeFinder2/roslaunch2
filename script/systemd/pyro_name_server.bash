#!/bin/bash -l

# The special shebang line in this script is important in order to execute this in a login shell,
# effectively sourcing the ~/.bashrc and setting up Python and ROS correctly. Since this is started
# in a login shell, no output is visible in "sudo journalctl -ru $SERVICE_UNIT_NAME"

# Run name server:
python -m Pyro4.naming -n $(hostname -I)
