@echo off
echo ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
echo +Make sure you have a X11 server runing                                +
echo +Make sure that "Native opengl" is disabled for the running X11 server +
for /f "delims=[] tokens=2" %%a in ('ping -4 -n 1 %ComputerName% ^| findstr [') do set NetworkIP=%%a
echo +Network IP: %NetworkIP%                                                 +
set DISPLAY=%NetworkIP%:0.0
echo +DISPLAY=%DISPLAY%                                                 +
set pfad=%CD%\catkin_ws\src\
echo +Path to catkin_ws=%pfad%
echo ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

docker run -it --rm --network bridge  -p 5000:5000 --mount type=bind,source="%pfad%\",target=/home/fhtw_user/catkin_ws/src/fhtw/ --name "fhtw_ros" --privileged -e DISPLAY=%DISPLAY%  "ghcr.io/tw-robotics/docker-ros:latest" "bash"

cmd /k
