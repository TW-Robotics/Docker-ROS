# Get the base directory of the repository
$BASE_DIR = git rev-parse --show-toplevel
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Could not find the base directory of the repository."
    exit 1
}

# Set default values
$ROS_DISTRO = "iron"  # iron, humble
$BASE_PACKAGE = "desktop"  # desktop, base
$TARGET = "ros_terminal"  # ros_terminal, ros_vnc

# Check for NVIDIA or AMD graphics
if (lspci | Select-String -Pattern "VGA" | Select-String -Pattern "nvidia" -CaseSensitive) {
    $GRAPHICS_PLATFORM = "nvidia"
}
elseif (lspci | Select-String -Pattern "VGA" | Select-String -Pattern "amd" -CaseSensitive) {
    $GRAPHICS_PLATFORM = "amd"
}
else {
    $GRAPHICS_PLATFORM = "standard"  # standard, nvidia, amd
}

# Parse command-line arguments
$opts = Getopt $args "d:p:t:g:h"
foreach ($opt in $opts) {
    switch ($opt[0]) {
        "d" {  # ROS_DISTRO
            if ($opt[1] -eq "iron" -or $opt[1] -eq "humble") {
                $ROS_DISTRO = $opt[1]
            }
            else {
                Write-Host "Invalid ROS_DISTRO: $($opt[1])"
                exit 1
            }
        }
        "p" {  # BASE_PACKAGE
            if ($opt[1] -eq "desktop" -or $opt[1] -eq "base") {
                $BASE_PACKAGE = $opt[1]
            }
            else {
                Write-Host "Invalid BASE_PACKAGE: $($opt[1])"
                exit 1
            }
        }
        "t" {  # TARGET
            if ($opt[1] -eq "ros_terminal" -or $opt[1] -eq "ros_vnc") {
                $TARGET = $opt[1]
            }
            else {
                Write-Host "Invalid TARGET: $($opt[1])"
                exit 1
            }
        }
        "g" {  # GRAPHICS_PLATFORM
            if ($opt[1] -eq "standard" -or $opt[1] -eq "nvidia" -or $opt[1] -eq "amd") {
                $GRAPHICS_PLATFORM = $opt[1]
            }
            else {
                Write-Host "Invalid GRAPHICS_PLATFORM: $($opt[1])"
                exit 1
            }
        }
        "h" {  # Help
            Write-Host "Usage: ./build.ps1 -d [iron, humble] -p [desktop, base] -t [ros_terminal, ros_vnc]"
            exit 0
        }
    }
}

# Build the Docker image
Push-Location -Path "$BASE_DIR/.devcontainer"
Write-Host "Building ROS $ROS_DISTRO $BASE_PACKAGE image..."
docker build --build-arg ROS_DISTRO=$ROS_DISTRO `
    --build-arg BASE_PACKAGE=$BASE_PACKAGE `
    --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM `
    --target $TARGET `
    -t "fhtw:$ROS_DISTRO-$BASE_PACKAGE-$TARGET-$GRAPHICS_PLATFORM" `
    -f Dockerfile `
    .
Pop-Location
