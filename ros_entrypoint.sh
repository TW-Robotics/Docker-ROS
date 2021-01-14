#!/bin/bash

set -e

DEFAULT_USER_ID=1000

#
# Ensure host and container have the same user ID. This is to allow both sides
# to read and write the shared directories.
#
if [ -v USER_ID ] && [ "$USER_ID" != "$DEFAULT_USER_ID" ]; then
    echo "Changing fhtw_user user ID to match your host's user ID ($USER_ID)." 
    echo "This operation can take a while..."

    usermod --uid $USER_ID fhtw_user

    # Ensure all files in the home directory are owned by the new user ID
    find /home/fhtw_user -user $DEFAULT_USER_ID -exec chown -h $USER_ID {} \;
fi

cd /home/fhtw_user

# If no command is provided, set bash to start interactive shell
if [ -z "$1" ]; then
    set - "/bin/bash" -l
fi

source /opt/ros/noetic/setup.bash
if [ -f /home/fhtw_user/catkin_ws/devel/setup.bash ]; then
    source /home/fhtw_user/catkin_ws/devel/setup.bash
fi

# Run the provided command using user 'fhtw_user'
exec gosu fhtw_user "$@"
