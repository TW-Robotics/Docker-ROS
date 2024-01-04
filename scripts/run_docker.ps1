# Get the base directory of the repository
$BaseDir = git rev-parse --show-toplevel
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Could not find base directory of repository."
    exit 1
}

# Change to the base directory
Push-Location $BaseDir

$SharedDir = "/home/fhtw_user/ros2_ws/src"
$HostDir = (Get-Location).Path + "/ros2_ws/src"

Write-Host -ForegroundColor Green "Mounting folder:`n    $HostDir    to`n    $SharedDir"


# Run Docker command
docker run `
    -it --rm --shm-size=512m `
    --volume="$HostDir:$SharedDir:rw" `
    --privileged -v /dev/bus/usb:/dev/bus/usb `
    --publish 6080:80 `
    --publish 6900:5901 `
    --name "fhtw_ros" `
    fhtw:iron-desktop-ros_terminal-nvidia

# Return to the original directory
Pop-Location
