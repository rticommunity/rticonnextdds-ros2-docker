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

   You can choose between 4 different ways to generate this image, based on
   how RTI Connext DDS and `rmw_connextdds` are provisioned inside it:

   - Use an RTI Connext DDS installation from the host, and build `rmw_connextdds`
     from source (default):

     ```sh
     CONNEXTDDS_DIR=/path/to/rti_connext_dds-6.1.0 \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - Specify the Connext installation with variables `CONNEXTDDS_DIR`, or
       `NDDSHOME`.

     - You can optionally use `CONNEXTDDS_ARCH` to specify the target architecture to use.

   - Install RTI Connext DDS using the official installers, and build
     `rmw_connextdds` from source:

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

   - Install RTI Connext DDS using a Debian package and build
     `rmw_connextdds` from source (x86_64 only):

     ```sh
     CONNEXTDDS_FROM_DEB=y \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - The community-licensed version of Connext included in the binary package
       distributed via the ROS 2 Debian repository can only be used for
       non-commercial and pre-production applications.

   - Install RTI Connext DDS and `rmw_connextdds` using Debian packages
     (x86_64 only):

     ```sh
     RMW_CONNEXTDDS_FROM_DEB=y \
     rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
     ```

     - This installation method requires the base image to include a version of
       ROS 2 installed from Debian packages (e.g. "osrf/ros:galactic-desktop").

     - `rmw_connextdds` is available as a Debian package only for ROS 2 Galactic
       and newer.

     - The community-licensed version of Connext included in the binary package
        distributed via the ROS 2 Debian repository can only be used for
        non-commercial and pre-production applications.

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

### ARM Setup

Docker images to use ROS on ARM targets must be built manually using repository
[osrf/docker_images](https://github.com/osrf/docker_images).

In order to provision Connext, you must first install it on an x86_64 Linux host,
and then copy the installation to the ARM device where you will build the Docker
images. If you only plan on using Connext to support `rmw_connextdds`,
you may restrict this copy to only a subset of the installation.

It is also recommended to replace script `resource/cmake/FindRTIConnextDDS.cmake`
in the Connext installation with the
[most recent version made available by RTI](https://github.com/rticommunity/rticonnextdds-cmake-utils/blob/main/cmake/Modules/FindRTIConnextDDS.cmake).

#### ARM Setup Example

1. On a x86_64 Linux host, install the Connext "host" bundle, then use
   `rtipkginstall` to install a "target" bundle for the desired ARM target, e.g.:

   ```sh
   ./rti_connext_dds-6.1.1-lm-x64Linux4gcc7.3.0.run

   ~/rti_connext_dds-6.1.1/bin/rtipkginstall rti_connext_dds-6.1.1-lm-target-armv8Linux4gcc7.3.0.rtipkg
   ```

2. Update `resource/cmake/FindRTIConnextDDS.cmake`:

   ```sh
   wget -o ~/rti_connext_dds-6.1.1/resource/cmake/FindRTIConnextDDS.cmake \
     https://raw.githubusercontent.com/rticommunity/rticonnextdds-cmake-utils/main/cmake/Modules/FindRTIConnextDDS.cmake
   ```

3. Generate an archive with the files required to support `rmw_connextdds`, e.g.:

   ```sh
   tar czf rti_connext_dds-6.1.1-rmw-runtime.tar.gz \
       rti_connext_dds-6.1.1/include \
       rti_connext_dds-6.1.1/lib/armv8Linux4gcc7.3.0 \
       rti_connext_dds-6.1.1/resource/cmake \
       rti_connext_dds-6.1.1/resource/scripts \
       rti_connext_dds-6.1.1/rti_versions.xml
   ```

   You can also generate the archive using script [generate_connext_rmw_runtime.sh](scripts/generate_connext_rmw_runtime.sh):

   ```sh
   git clone https://github.com/rticommunity/rticonnextdds-ros2-docker

   rticonnextdds-ros2-docker/scripts/generate_connext_rmw_runtime.sh \
     ~/rti_connext_dds-6.1.1 \
     armv8Linux4gcc7.3.0
   ```

4. Copy and extract archive on ARM target, e.g. (replace `arm-target` with the
   host name/IP address of your device):

   ```sh
   scp rti_connext_dds-6.1.1-rmw-runtime.tar.gz arm-target:~/

   ssh arm-target tar xzf rti_connext_dds-6.1.1-rmw-runtime.tar.gz
   ```

   or, without first copying the archive:

   ```sh
   cat rti_connext_dds-6.1.1-rmw-runtime.tar.gz | ssh arm-target tar xzf -
   ```

   or, without even creating an intermediate archive:

   ```sh
   tar cz \
       rti_connext_dds-6.1.1/include \
       rti_connext_dds-6.1.1/lib/armv8Linux4gcc7.3.0 \
       rti_connext_dds-6.1.1/resource/cmake \
       rti_connext_dds-6.1.1/resource/scripts \
       rti_connext_dds-6.1.1/rti_versions.xml | ssh arm-target tar xzf -
   ```

5. On the ARM target, clone `osrf/docker_images` and build the base ROS image, e.g.:

   ```sh
   git clone https://github.com/osrf/docker_images

   docker build -t ros:humble-desktop docker_images/ros/humble/ubuntu/jammy/desktop
   ```

6. Clone this repository and build the `rmw_connextdds` image:

   ```sh
   git clone https://github.com/rticommunity/rticonnextdds-ros2-docker

   CONNEXTDDS_DIR=~/rti_connext_dds-6.1.1 \
   BASE_IMAGE=ros:humble-desktop \
   DOCKER_IMAGE=rmw_connextdds:humble-desktop-6.1.1 \
   rticonnextdds-ros2-docker/scripts/build_image_rmw_connextdds.sh
   ```

7. (Optional) Build the `ros2_workspace` image:

   ```sh
   BASE_IMAGE=rmw_connextdds:humble-desktop-6.1.1 \
   DOCKER_IMAGE=ros2_workspace:humble-desktop-6.1.1 \
   rticonnextdds-ros2-docker/scripts/build_image_ros2_workspace.sh
   ```

8. (Optional) Use the `ros2_workspace` image:

   ```sh
   DOCKER_IMAGE=ros2_workspace:humble-desktop-6.1.1 \
   rticonnextdds-ros2-docker/scripts/run_ros2_workspace.sh
   ```

## Docker Images

All of the `rmw_connextdds` images should be built on top of one of the
[ROS 2 images provided by OpenRobotics](https://github.com/osrf/docker_images).

The `Dockerfile.rmw_connextdds.*` files all follow a similar workflow:

- Install RTI Connext DDS, then clone and build `rmw_connextdds`, or...
- Install both RTI Connext DDS and `rmw_connextdds` from a Debian binary
  packages.
- Set `rmw_connextdds` as the default RMW_IMPLEMENTATION
- Add a bashrc file to automatically load ROS 2 with RTI Connext DDS.

The `ros2_workspace` image uses the `rmw_connextdds` images to generate a
development environment suitable for mounting and editing a local directory
from the host machine.

### Dockerfile.rmw_connextdds.bin

This Dockerfile will generate a Docker image which includes a copy of
RTI Connext DDS installed using the community-licensed package distributed
via the ROS 2 Debian repository.

`rmw_connextdds` will also be installed using the binary Debian package provided
in the ROS 2 Debian repository.

#### Build Arguments for Dockerfile.rmw_connextdds.bin

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|

#### Manual build of Dockerfile.rmw_connextdds.bin

```sh
docker build rticonnextdds-ros2-docker/docker \
             -t rmw_connextdds:latest \
             -f rticonnextdds-ros2-docker/docker/Dockerfile.rmw_connextdds.bin
```

### Dockerfile.rmw_connextdds.deb

This Dockerfile will generate a Docker image which includes a copy of
RTI Connext DDS installed using the community-licensed package distributed
via the ROS 2 Debian repository.

`rmw_connextdds` will be built from source.

#### Build Arguments for Dockerfile.rmw_connextdds.deb

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|
|`RMW_CONNEXTDDS_BRANCH`|Branch to build in `rmw_connextdds`'s Git repository|Automatically detected based on `ROS_DISTRO`|
|`RMW_CONNEXTDDS_DIR`|Container directory where to clone `rmw_connextdds`.|`/opt/rmw_connextdds`|
|`RMW_CONNEXTDDS_URL`|Clone URL of `rmw_connextdds`'s Git repository|`https://github.com/ros2/rmw_connextdds`|

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

`rmw_connextdds` will be built from source.

#### Build Arguments for Dockerfile.rmw_connextdds.host

| Variable | Description | Default Value |
|----------|-------------|---------------|
|`BASE_IMAGE`|Base Docker image.|`osrf/ros:galactic-desktop`|
|`CONNEXTDDS_ARCH`|Target architecture to use. ||
|`CONNEXTDDS_HOST_DIR`|Name of the subdirectory of `archives/`, containing the installation of RTI Connext DDS to use inside the container||
|`RMW_CONNEXTDDS_BRANCH`|Branch to build in `rmw_connextdds`'s Git repository|Automatically detected based on `ROS_DISTRO`|
|`RMW_CONNEXTDDS_DIR`|Container directory where to clone `rmw_connextdds`.|`/opt/rmw_connextdds`|
|`RMW_CONNEXTDDS_URL`|Clone URL of `rmw_connextdds`'s Git repository|`https://github.com/ros2/rmw_connextdds`|

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
|`RMW_CONNEXTDDS_BRANCH`|Branch to build in `rmw_connextdds`'s Git repository|Automatically detected based on `ROS_DISTRO`|
|`RMW_CONNEXTDDS_DIR`|Container directory where to clone `rmw_connextdds`.|`/opt/rmw_connextdds`|
|`RMW_CONNEXTDDS_URL`|Clone URL of `rmw_connextdds`'s Git repository|`https://github.com/ros2/rmw_connextdds`|

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
