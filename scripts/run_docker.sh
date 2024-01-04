#!/bin/sh
BASE_DIR=$(git rev-parse --show-toplevel)
if [ $? -ne 0 ]; then
    echo "Error: Could not find base directory of repository."
    exit 1
fi

pushd ${BASE_DIR}/
SHARED_DIR=/home/fhtw_user/ros2_ws/src
HOST_DIR=$BASE_DIR/ros2_ws/src

echo -e "\e[32mMounting fodler:
    $HOST_DIR    to
    $SHARED_DIR\e[0m"

ROS_DISTRO="iron"
BASE="desktop"
UI="terminal"

if lspci | grep VGA | grep -i nvidia &> /dev/null; then
    GRAPHICS_PLATFORM="nvidia"
elif lspci | grep VGA | grep -i amd &> /dev/null; then
    GRAPHICS_PLATFORM="amd"
else
    GRAPHICS_PLATFORM="standard" # standard, nvidia, amd
fi


while getopts d:p:t:g:h flag
do
    case "${flag}" in
        d) # CHECK IF ROS_DISTRO IS VALID [iron, humble]
            if [ ${OPTARG} == "iron" ] || [ ${OPTARG} == "humble" ]; then
                ROS_DISTRO=${OPTARG}
            else
                echo "Invalid ROS_DISTRO: ${OPTARG}"
                exit 1
            fi
            ;;
        p) # CHECK IF BASE IS VALID [desktop, base]
            if [ ${OPTARG} == "desktop" ] || [ ${OPTARG} == "base" ]; then
                BASE=${OPTARG}
            else
                echo "Invalid BASE: ${OPTARG}"
                exit 1
            fi
            ;;
        t) # CHECK IF UI IS VALID [terminal, vnc]
            if [ ${OPTARG} == "terminal" ] || [ ${OPTARG} == "vnc" ]; then
                UI=${OPTARG}
            else
                echo "Invalid UI: ${OPTARG}"
                exit 1
            fi
            ;;
        g) # CHECK IF GRAPHICS_PLATFORM IS VALID [standard, nvidia, amd]
            if [ ${OPTARG} == "standard" ] || [ ${OPTARG} == "nvidia" ] || [ ${OPTARG} == "amd" ]; then
                GRAPHICS_PLATFORM=${OPTARG}
            else
                echo "Invalid GRAPHICS_PLATFORM: ${OPTARG}"
                exit 1
            fi
            ;;
        h) # HELP
            echo "Usage: ./run_docker.sh -d [iron, humble] -p [desktop, base] -t [terminal, vnc]"
            exit 0
            ;;
    esac
done


IMAGE="fhtw:${ROS_DISTRO}-${BASE}-ros_${UI}-${GRAPHICS_PLATFORM}"

RUN_ARGS='--rm --shm-size=512m \
    --volume="$HOST_DIR":"$SHARED_DIR":rw \
    --privileged \
    --env="DISPLAY=${DISPLAY:=0.0}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    '

if [ ${UI} == "vnc" ]; then
    RUN_ARGS="${RUN_ARGS} --publish 6080:80 \
        --publish 6900:5901 \
        "
fi

if [ ${GRAPHICS_PLATFORM} == "nvidia" ]; then
    RUN_ARGS="${RUN_ARGS} --gpus all"
elif [ ${GRAPHICS_PLATFORM} == "amd" ]; then
    RUN_ARGS="${RUN_ARGS} --device=/dev/dri:/dev/dri"
elif [ ${GRAPHICS_PLATFORM} == "standard" ]; then
    RUN_ARGS="${RUN_ARGS} --device=/dev/dri:/dev/dri --env=LIBGL_ALWAYS_SOFTWARE=1"
fi

echo -e "${RUN_ARGS}"
exit 0
xhost +
docker run \
    ${RUN_ARGS} \
    --name "fhtw_ros' \
    $IMAGE
xhost -
popd