#!/bin/sh

VNC_CMD="vncserver :1 -fg -geometry 1920x1080 -depth 24"
if [ "$GRAPHICS_PLATFORM" = "nvidia" ]; then
    VNC_CMD="/opt/VirtualGL/bin/vglrun $VNC_CMD"
else
    $VNC_CMD
fi