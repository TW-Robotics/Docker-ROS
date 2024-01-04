#!/usr/bin/env bash

# Get the container ID
CONTAINER=$(docker container ls | grep -E "ros|fhtw" | awk '{print $1}')

# Execute the entrypoint script inside the container
docker container exec -it $CONTAINER  /entrypoint.sh tmux