#!/bin/sh
################################################################################
# (c) 2022 Copyright, Real-Time Innovations, Inc. (RTI) All rights reserved.
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software solely in combination with RTI Connext DDS. Licensee
# may redistribute copies of the Software provided that all such copies are
# subject to this License. The Software is provided "as is", with no warranty
# of any type, including any warranty for fitness for any purpose. RTI is
# under no obligation to maintain or support the Software. RTI shall not be
# liable for any incidental or consequential damages arising out of the use or
# inability to use the Software. For purposes of clarity, nothing in this
# License prevents Licensee from using alternate versions of DDS, provided
# that Licensee may not combine or link such alternate versions of DDS with
# the Software.
################################################################################
# Run a docker container to build and run local applications with rmw_connextdds.
################################################################################
# The generated container will mount a local directory under `/workspace`, and
# it will be configured to share the X11 server with the host.
#
# The directory to mount can be specified using variable `WORKSPACE_DIR` (
# default: current directory).
#
# You can control the name of the container using variable `DOCKER_CONTAINER`.
# (default: `ros2_workspace-dev`).
#
# The scripts expects the container's image to have already been built.
# You can use the included helper scripts (`build_image_rmw_connextdds.sh` and
# `build_image_ros2_workspace.sh`) to generate a suitable image.
#
# You can specify a custom image to use with variable `DOCKER_IMAGE` (default: 
# `ros2_workspace:latest`).
#
# The script expects the image to have a non-root user that matches the
# one. You can customize the name of the user with variable `DOCKER_USER`.
#
# The script will first check if a container with the specified name is already
# running and attach to it if it is.
#
# Remember to keep the first terminal that starts a container open, because
# the container will be automatically terminated and removed once this shell
# terminates.
################################################################################

################################################################################
# Query Docker to check if a container is in a certain status. Returns a
# non-empty string if so.
################################################################################
check_container_status()
{
  docker ps -a --quiet --filter status=${2} --filter name=${1}
}

################################################################################
# Begin script
################################################################################
set -e
SH_CWD=$(pwd)
SH_DIR=$(cd "$(dirname "$0")" > /dev/null 2>&1 && pwd) 
REPO_DIR=$(cd "${SH_DIR}/.." && pwd)

# Load common helper functions
. "${SH_DIR}/common.sh"

: "${WORKSPACE_DIR:=${SH_CWD}}"
: "${DOCKER_CONTAINER:=ros2_workspace-dev}"
: "${DOCKER_IMAGE:="ros2_workspace:latest"}"
: "${DOCKER_USER:=$(whoami)}"

# Sanity-check: remove the container if it exists and it's in "exited" status
if [ "$(check_container_status ${DOCKER_CONTAINER} exited)" ]; then
    log_info "removing docker container: ${DOCKER_CONTAINER}"
    docker rm ${DOCKER_CONTAINER} > /dev/null
fi

# Check if a container with the selected name is already running.
# If so, attach to it.
if [ "$(check_container_status ${DOCKER_CONTAINER} running)" ]; then
    log_info "attaching to docker container: ${DOCKER_CONTAINER}"
    [ -z "$@" ] || log_info "with command: $@"
    docker exec -it -u ${DOCKER_USER} \
      --workdir /workspace \
      ${DOCKER_CONTAINER} \
      /bin/bash \
      $@
    exit 0
fi

# Docker requires volume paths to be absolute
WORKSPACE_DIR=$(cd "${WORKSPACE_DIR}" && pwd)

log_info "running docker container: ${DOCKER_CONTAINER}"
[ -z "$@" ] || log_info "with command: $@"
(
  set -x
  docker run -it --rm \
    --network host \
    -v "${WORKSPACE_DIR}:/workspace" \
    --name "${DOCKER_CONTAINER}" \
    --user "${DOCKER_USER}" \
    --workdir /workspace \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${DOCKER_IMAGE}" \
    $@
)