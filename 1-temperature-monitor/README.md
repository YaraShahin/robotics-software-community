# Temperature Monitoring

## How to setup

1. `git clone `: 
    clone the monorepo. The repo already contains both workspaces for the publisher and subscriber containers.
2. `just setup-project`:
    a. symlinks the relevant packages from packages dir to each workspace src
    b. runs docker compose build to build both containers
    c. runs each container and installs all dependencies and builds the workspace.

## How to run

1. First Terminal:
    a. `just run-publisher`: will open the publisher container shell
    b. `cd `
    c. Optional: `colcon build --symlink-install && source install/setup.bash`: If you made changes in source code and want to rebuild
    d. `ros2 run `

