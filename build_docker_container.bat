@echo off
echo Building the Dockercontainer
echo ...this will take a while...
docker build -t "georgno/fhtw-ros:melodic" --rm .
IF %ERRORLEVEL% == 1 (
    echo Something went wrong
    echo Check above for error
    pause
) ELSE (
    echo .
    echo =================================================
    echo Finished installation, this window is going to close now
    timeout /t 10
)

