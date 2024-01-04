#!/bin/bash
BASE_DIR=$(git rev-parse --show-toplevel)
if [ $? -ne 0 ]; then
    echo "Error: Could not find base directory of repository."
    exit 1
fi

# Set default values
ROS_DISTRO="iron"  # iron, humble
INSTALL_PACKAGE="desktop"  # desktop, base
TARGET=ros_terminal # ros_terminal, ros_vnc

if lspci | grep VGA | grep -i nvidia &> /dev/null; then
    GRAPHICS_PLATFORM="nvidia"
elif lspci | grep VGA | grep -i amd &> /dev/null; then
    GRAPHICS_PLATFORM="amd"
else
    GRAPHICS_PLATFORM="standard" # standard, nvidia, amd
fi
# Get opts for ROS_DISTRO, INSTALL_PACKAGE, and TARGEt
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
        p) # CHECK IF INSTALL_PACKAGE IS VALID [desktop, base]
            if [ ${OPTARG} == "desktop" ] || [ ${OPTARG} == "base" ]; then
                INSTALL_PACKAGE=${OPTARG}
            else
                echo "Invalid INSTALL_PACKAGE: ${OPTARG}"
                exit 1
            fi
            ;;
        t) # CHECK IF TARGET IS VALID [ros_terminal, ros_vnc]
            if [ ${OPTARG} == "ros_terminal" ] || [ ${OPTARG} == "ros_vnc" ]; then
                TARGET=${OPTARG}
            else
                echo "Invalid TARGET: ${OPTARG}"
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
            echo "Usage: ./build.sh -d [iron, humble] -p [desktop, base] -t [ros_terminal, ros_vnc]"
            exit 0
            ;;
    esac
done



pushd ${BASE_DIR}/.devcontainer
echo "Building ROS ${ROS_DISTRO} ${INSTALL_PACKAGE} image..."
docker build \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --build-arg INSTALL_PACKAGE=${INSTALL_PACKAGE} \
    --build-arg GRAPHICS_PLATFORM=${GRAPHICS_PLATFORM} \
    --target ${TARGET} \
    -t fhtw:${ROS_DISTRO}-${INSTALL_PACKAGE}-${TARGET}-${GRAPHICS_PLATFORM} \
    -f Dockerfile \
    .
popd