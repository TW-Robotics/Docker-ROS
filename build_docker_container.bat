@echo off
echo Building the Dockercontainer
echo ...this will take a while...
docker build -t "fhtw/ros:latest" --rm .
echo .
echo =================================================
echo Finished installation, this window is going to close now
timeout /t 10
