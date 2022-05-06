# Docker containers for ROS 2 development with RTI Connext DDS

This repository contains helper resources to generate Docker images for
developing and running ROS 2 application with RTI Connext DDS inside
Docker containers.

## Local Workspace Quickstart

The following workflow uses the included helper scripts to create a Docker container which mounts a local directory from the host, allowing you to easily build your project inside a preconfigured environment with ROS 2 and
RTI Connext DDS.

The process relies on creating a Docker image which contains the required
software components (ROS 2, RTI Connext DDS, and rmw_connextdds), and a non-root user which is mapped to the current user on the host.

1. [Install Docker](https://docs.docker.com/engine/install/) and
   [enable non-root access for your user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

2. Clone this repository:

   ```sh
   git clone https://github.com/rticommunity/rticonnextdds-ros2-docker
   ```

3. Build image `rmw_connextdds:latest`, a base Docker image with ROS 2, RTI
   Connext DDS, and `rmw_connextdds`. You can use script
   `build_image_rmw_connextdds.sh` for this purpose.

   If you prefer, you can control the name of the generated image using variable
   `DOCKER_IMAGE`.

   By default, the image will use ROS 2 Galactic, provided by image
   `osrf/ros:galactic-desktop`. If you prefer a different ROS 2 version,
   select an image from [those available on Docker Hub](https://github.com/osrf/docker_images) and specify it using variable `BASE_IMAGE`.

   You can choose between 3 different ways to generate this image, based on
   how RTI Connext DDS is provisioned to it:

   - Use an RTI Connext DDS installation from the host (default):

     ```sh
     CONNEXTDDS_DIR=/path/to/rti_connext_dds-6.1.0 \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - Specify the Connext installation with variables `CONNEXTDDS_DIR`, or
       `NDDSHOME`.

     - You can optionally use `CONNEXTDDS_ARCH` to specify the target architecture to use.

   - Install RTI Connext DDS using the official installers:

     ```sh
     CONNEXTDDS_FROM_RTIPKG=y \
     CONNEXTDDS_INSTALLER_HOST=/path/to/<HOST_INSTALLER> \
     CONNEXTDDS_INSTALLER_TARGET=/path/to/<TARGET_INSTALLER> \
     CONNEXTDDS_INSTALLER_LICENSE=/path/to/<LICENSE_FILE> \
     CONNEXTDDS_VERSION=6.1.0 \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - The following files are required:

       - A host installer, e.g. `rti_connext_dds-6.1.0-pro-host-x64Linux.run` (`CONNEXTDDS_INSTALLER_HOST`).
       - A target installer, e.g. `rti_connext_dds-6.1.0-pro-target-x64Linux4gcc7.3.0.rtipkg` (`CONNEXTDDS_INSTALLER_TARGET`).
       - A license file, e.g. `rti_license.dat` (`CONNEXTDDS_INSTALLER_LICENSE`).

     - Variable `CONNEXTDDS_VERSION` is required to let the Dockerfile detect
       the generated RTI Connext DDS installation directory.

   - Install RTI Connext DDS using a Debian package (x86_64 only):

     ```sh
     CONNEXTDDS_FROM_DEB=y \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - The community-licensed version included in the binary package distributed
       via the ROS 2 Debian repository can only be used for non-commercial and
       pre-production applications.

4. Build image `ros2_workspace:latest`.

   This image builds on top of the `rmw_connextdds` image by adding a non-root
   user to match your user, allowing you to mount and edit a directory from the
   host. You can generate this image using script `build_image_ros2_workspace.sh`.

   If you used a custom name for the base image, specify it using variable
   `BASE_IMAGE` (default: `rmw_connextdds:latest`).

   You can control the name of the generated image using variable
   `DOCKER_IMAGE` (default: `ros2_workspace:latest`).

   ```sh
   rticonnextdds-ros2-docker/scripts/build_image_ros2_workspace.sh
   ```

5. Start a Docker container with the generated image. You can use the
   `run_ros2_workspace.sh` script to do so, which will automatically start the 
   container or attach to it with a new terminal if it is already running.

   Use variable `WORKSPACE_DIR` to specify a directory to mount inside the
   container (under path `/workspace`, default: current directory).

   You can control the name of the container and the image to use with variables
   `DOCKER_CONTAINER` (default: `ros2_workspace-dev`) and `DOCKER_IMAGE` (default: `ros2_workspace:latest`) respectively.

   ```sh
   WORKSPACE_DIR=/path/to/my/workspace
   rticonnextdds-ros2-docker/scripts/run_ros2_workspace.sh
   ```

### Example Workspace

The repository includes a test ROS 2 package which you may use as a workspace
directory to test the Docker images.

After building the `ros2_workspace` image, use the `run_ros2_workspace.sh` script
to build and run the `example_workspace` directory inside a container:

```sh
cd rticonnextdds-ros2-docker/example_workspace
../scripts/run_ros2_workspace.sh
# Inside the container, build the included ROS 2 package
colcon build --symlink-install
source install/setup.bash
# Use the example launch file to start two nodes with a custom
# QoS configuration file.
ros2 launch example_workspace talker-listener.launch.py
```

### Docker Compose Example

The repository includes a `docker-compose` example which shows how to use the
`rmw_connextdds` images to run some example ROS 2 applications with RTI Connext
DDS.

After building the `rmw_connextdds` image, use the following command to run the
talker/listener example from package `demo_nodes_cpp` in two different
containers while forcing all communications to occur via the shared memory
transport:

```sh
cd rticonnextdds-ros2-docker/example_workspace
WORKSPACE_DIR=$(pwd) \
docker-compose -f ../docker_compose/docker-compose-talker_listener.yml up
```

Use `CTRL+C` to terminate the containers.

## Docker Images

All of the `rmw_connextdds` images should be built on top of one of the
[ROS 2 images provided by OpenRobotics](https://github.com/osrf/docker_images).

The `Dockerfile.rmw_connextdds.*` files all follow a similar workflow:

- Install RTI Connext DDS.
- Clone and build `rmw_connextdds`
- Set `rmw_connextdds` as the default RMW_IMPLEMENTATION
- Add a bashrc file to automatically load ROS 2 with RTI Connext DDS.

The `ros2_workspace` image uses the `rmw_connextdds` images to generate a
development environment suitable for mounting and editing a local directory
from the host machine.

### Dockerfile.rmw_connextdds.deb

This Dockerfile will generate a Docker image which includes a copy of
RTI Connext DDS installed using the community-licensed package distributed
via the ROS 2 Debian repository.

#### Build Arguments for Dockerfile.rmw_connextdds.deb

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|

#### Manual build of Dockerfile.rmw_connextdds.deb

```sh
docker build rticonnextdds-ros2-docker/docker \
             -t rmw_connextdds:latest \
             -f rticonnextdds-ros2-docker/docker/Dockerfile.rmw_connextdds.deb
```

### Dockerfile.rmw_connextdds.host

This Dockerfile will generate a Docker image which contains a copy of
RTI Connext DDS pre-installed on the host machine.

The Dockerfile will copy this installation specified by argument
`CONNEXTDDS_HOST_DIR` "as is".

This directory must be located under `archives/` in the build context, and it
must contain a valid license file and target libraries.

If multiple target libraries are installed, the desired target architecture should
be specified using argument `CONNEXTDDS_ARCH`.

#### Build Arguments for Dockerfile.rmw_connextdds.host

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|
|`CONNEXTDDS_ARCH`|Target architecture to use. ||
|`CONNEXTDDS_HOST_DIR`|Name of the subdirectory of `archives/`, containing the installation of RTI Connext DDS to use inside the container||

#### Manual build of Dockerfile.rmw_connextdds.host

This example assumes that the full path of the RTI Connext DDS host installation
is exported by variable `CONNEXTDDS_DIR` in the current shell environment.

```sh
# Create archives/ directory and copy Connext installation
mkdir rticonnextdds-ros2-docker/docker/archives
cp -r ${CONNEXTDDS_DIR} rticonnextdds-ros2-docker/docker/archives

docker build rticonnextdds-ros2-docker/docker \
             -t rmw_connextdds:latest \
             -f rticonnextdds-ros2-docker/docker/Dockerfile.rmw_connextdds.host \
             --build-arg CONNEXTDDS_HOST_DIR=$(basename ${CONNEXTDDS_DIR})
```

### Dockerfile.rmw_connextdds.rtipkg

This Dockerfile will generate a Docker image which contains a copy
of RTI Connext DDS installed from the official installers provided by RTI.

The Dockerfile expects to find the required installers in subdirectory
`archives/` of the docker build context.

Beside the target and host installers, you will also need to provide a valid
license file.

You can specify the name of the files (without paths)
using arguments `CONNEXTDDS_INSTALLER_HOST`, `CONNEXTDDS_INSTALLER_TARGET`,
and `CONNEXTDDS_INSTALLER_LICENSE`.

You must also make sure to specify argument `CONNEXTDDS_VERSION` with a 3-digit
version identifier for the selected version of RTI Connext DDS.

#### Build Arguments for Dockerfile.rmw_connextdds.rtipkg

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|
|`CONNEXTDDS_INSTALLER_HOST`|File name (without path) of the host bundle|`rti_connext_dds-6.1.0-pro-host-x64Linux.run`|
|`CONNEXTDDS_INSTALLER_LICENSE`|File name (without path) of the license file.|`rti_license.dat`|
|`CONNEXTDDS_INSTALLER_TARGET`|File name (without path) of the target bundle|`rti_connext_dds-6.1.0-pro-target-x64Linux4gcc7.3.0.rtipkg`|
|`CONNEXTDDS_VERSION`|Version identifier for Connext DDS. |`6.1.0`|
|`CONNEXTDDS_ARCH`|Target architecture to use. ||

#### Manual build of Dockerfile.rmw_connextdds.rtipkg

```sh
# Create archives/ directory and copy the Connext installer and license files
mkdir rticonnextdds-ros2-docker/docker/archives
cp /path/to/<HOST_INSTALLER> \
   /path/to/<TARGET_INSTALLER> \
   /path/to/<LICENSE_FILE> \
   rticonnextdds-ros2-docker/docker/archives

docker build rticonnextdds-ros2-docker/docker \
             -t rmw_connextdds:latest \
             -f rticonnextdds-ros2-docker/docker/Dockerfile.rmw_connextdds.rtipkg \
             --build-args CONNEXTDDS_INSTALLER_HOST=<HOST_INSTALLER> \
             --build-args CONNEXTDDS_INSTALLER_TARGET=<TARGET_INSTALLER> \
             --build-args CONNEXTDDS_INSTALLER_LICENSE=<LICENSE_FILE>
```

### Dockerfile.ros2_workspace

This Dockerfile will generate a Docker image that can be used for local
development of a ROS 2 application on the host machine.

The image requires one of the `rmw_connextdds` images to have been previously
built, and it will:

- Add a non-root user to match the user which owns the workspace directory on the
  host.
- Configure a custom entrypoint script which will automatically configure
  the development environment, and possibly start a custom command passed to
  `docker run` (see [entrypoint.sh](docker/scripts/ros2_workspace_entrypoint.sh))

#### Build Arguments for Dockerfile.ros2_workspace

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`rmw_connextdds:latest`|
|`DOCKER_GID`|Id of the main group for the container's non-root user|`1000`|
|`DOCKER_UID`|Id of the container's non-root user|`1000`|
|`DOCKER_USER`|Name of the non-root user created inside the container to map the host's user|`admin`|

#### Manual build of Dockerfile.ros2_workspace

```sh
docker build rticonnextdds-ros2-docker/docker \
             -t ros2_workspace:latest \
             -f rticonnextdds-ros2-docker/docker/Dockerfile.ros2_workspace \
             --build-arg BASE_IMAGE=rmw_connextdds:latest \
             --build-arg DOCKER_USER=$(whoami) \
             --build-arg DOCKER_UID=$(id -u) \
             --build-arg DOCKER_GID=$(id -g) \
```
