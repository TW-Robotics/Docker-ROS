@echo off
echo Building the Dockercontainer
echo ...this will take a while...
docker build -t "fhtw/ros:latest" --rm .
IF %ERRORLEVEL% == 1 (
    echo Something went wrong
    echo Check above for error 
    echo For detailed output run 'docker build  --progress=plain  -t "fhtw/ros:latest" --rm .' instead of 'docker build -t "fhtw/ros:latest" --rm .'
    pause
) ELSE (
    echo .
    echo =================================================
    echo Finished installation, this window is going to close now
    timeout /t 10
)

