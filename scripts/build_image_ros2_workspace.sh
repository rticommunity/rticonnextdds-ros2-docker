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

################################################################################
# Build a Docker image for local ROS 2 development with RTI Connext DDS
################################################################################
# This script generates a Docker image meant to mount a local host directory
# and to allow building and running a ROS 2 application in a preconfigured
# environment.
#
# The docker image is generated using file Dockerfile.workspace, and it
# requires the use of a base image that already provides ROS 2, RTI Connext DDS,
# and rmw_connextdds. This image can be generated with script
# build_rmw_connextdds_docker.sh and files "Dockerfile.rmw_connextdds.*".
#
# The base image to use can be specified with variable BASE_IMAGE.
# 
# The script will configure the generated image to have a user which matches
# the current user running the script. You can customize information about this
# user using variables DOCKER_USER, DOCKER_UID, and DOCKER_GID.
# 
################################################################################
set -e
SH_DIR=$(cd "$(dirname "$0")" > /dev/null 2>&1 && pwd) 
REPO_DIR=$(cd "${SH_DIR}/.." && pwd)

# Load common helper functions
. "${SH_DIR}/common.sh"

: "${BASE_IMAGE:="rmw_connextdds:latest"}"
: "${DOCKER_IMAGE:="ros2_workspace:latest"}"
: "${DOCKER_DIR:=${REPO_DIR}/docker}"
: "${DOCKER_FILE:=${DOCKER_DIR}/Dockerfile.ros2_workspace}"
: "${DOCKER_BUILD_TAG:=$(date +%s)}"
: "${DOCKER_BUILD_CONTEXT:=/tmp/ros2_worspace-docker-ctx-${DOCKER_BUILD_TAG}}"
: "${DOCKER_USER:=$(whoami)}"
: "${DOCKER_UID:=$(id -u)}"
: "${DOCKER_GID:=$(id -g)}"

export_docker_build_arg DOCKER_BUILD_ARGS BASE_IMAGE
export_docker_build_arg DOCKER_BUILD_ARGS DOCKER_USER
export_docker_build_arg DOCKER_BUILD_ARGS DOCKER_UID
export_docker_build_arg DOCKER_BUILD_ARGS DOCKER_GID

log_info "building ros2_workspace docker image: ${DOCKER_IMAGE}"
(
  set -x
  docker build "${DOCKER_DIR}" \
    -f "${DOCKER_FILE}" -t "${DOCKER_IMAGE}" ${DOCKER_BUILD_ARGS}
)
