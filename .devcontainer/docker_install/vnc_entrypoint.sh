#!/bin/bash

set -e

USERNAME=DEFALTUSER
DEFAULT_USER_ID=1000

#
# Ensure host and container have the same user ID. This is to allow both sides
# to read and write the shared directories.
#
if [ -v USER_ID ] && [ "$USER_ID" != "$DEFAULT_USER_ID" ]; then
    echo "Changing ${USERNAME} user ID to match your host's user ID ($USER_ID)." 
    echo "This operation can take a while..."

    usermod --uid $USER_ID ${USERNAME}

    # Ensure all files in the home directory are owned by the new user ID
    find /home/${USERNAME} -user $DEFAULT_USER_ID -exec chown -h $USER_ID {} \;
fi

cd /home/${USERNAME}

# If no command is provided, set bash to start interactive shell
if [ -z "$1" ]; then
    set - "/bin/bash" -l
fi

source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [ -f /home/${USERNAME}/sim_ws/install/setup.bash ]; then
    source /home/${USERNAME}/sim_ws/install/setup.bash
fi
if [ -f /home/${USERNAME}/ros2_ws/install/setup.bash ]; then
    source /home/${USERNAME}/ros2_ws/install/setup.bash
fi

# Start supervisord and services and store logs in /var/log/supervisor/
# check if supervisord is already running
if [ ! -e /tmp/supervisord.pid ]; then
    supervisord -c /etc/supervisor/supervisord.conf -j /tmp/supervisord.pid -l /var/log/supervisor/supervisord.log
fi
if [ "$TERM_PROGRAM" != "vcode" ]; then
    exec gosu ${USERNAME} bash
fi