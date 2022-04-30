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
# Build a docker image and start a docker container to build and run
# applications with rmw_connextdds.
################################################################################
# You can control the name of the container using variable DOCKER_CONTAINER.
# If unspecified, the container will be named `rmw_connextdds-dev`.
#
# By default, the location of repository `rticonnextdds-ros2-docker` will be
# used (determined by "introspecting" the location of this script).
# A custom location can be specified by setting variable WORKSPACE_DIR.
# Any argument passed to this script will be passed directlry to `docker run`.
#
# Connext DDS can be provisioned inside the contained image in multiple ways.
#
# By default, RTI Connext DDS will be installed using the official installers
# provided by RTI. The script expects to find the required installer files, and
# a valid license inside directory connext_docker_workspace/resource/docker/archives.
#
# Before running this script, make sure to create this directory and to copy
# (or symlink) the required files inside it. For example:
#
#  mkdir connext_docker_workspace/resource/docker/archives
#  cp rti_connext_dds-6.1.0-pro-host-x64Linux.run \
#     rti_connext_dds-6.1.0-pro-target-x64Linux4gcc7.3.0.rtipkg \
#     rti_license.dat \
#     connext_docker_workspace/resource/docker/archives
#
# You can use variables CONNEXTDDS_VERSION, CONNEXTDDS_HOST_ARCH, and
# CONNEXTDDS_ARCH to allow the Dockerfile to guess the correct name of the
# installers to use. Alternatively, you may also explicitly provide the name
# of the host and target bundles using variables CONNEXTDDS_INSTALLER_HOST and
# CONNEXTDDS_INSTALLER_TARGET. You can specify a custom name for the license
# file with variable CONNEXTDDS_INSTALLER_LICENSE.
#
# Alternatively, you can have the script copy a preinstalled copy of Connext
# from the host machine. You must copy (or symlink) this directory inside
# `connext_docker_workspace/resource/docker/archives` and specify its name using
# variable CONNEXTDDS_HOST_DIR. You must also specify the target architecture to
# use with variable CONNEXTDDS_ARCH.
#
# You can also build a container image using the community-licensed version of
# Connext DDS provided by Debian package rti-connext-dds-6.0.1. This image can
# only be used for pre-production and non-commercial purposes.
# In order to use it, set variable CONNEXTDDS_COMMUNITY to a non-empty value,
# e.g.: `export CONNEXTDDS_COMMUNITY=y`.
#
# In all cases, the generated image will be tagged as `rmw_connextdds:latest`.
# You can specify a custom tag using variable DOCKER_IMAGE.
#
# The images are created with a non-root user which should be mapped to the user
# which owns the workspace directory that will be mounted on the container.
# You can customize the username, and the UID and GID using
# variables DOCKER_USER, DOCKER_GID, and DOCKER_GID respectively. If these are
# unspecified, the script will use information from the user running the script.
################################################################################

set -e

################################################################################
# Helper functions to build a list of arguments to pass to `docker build`.
# Argument names and values must not contain any spaces.
################################################################################
eval_var()
{
  eval "printf -- \"\${${1}}\""
}
export_docker_arg()
{
  [ -n "$(eval_var ${1})" ] || return 0
  local build_arg="--build-arg ${1}=$(eval_var ${1})"
  if [ -z "${DOCKER_BUILD_ARGS}" ]; then
    DOCKER_BUILD_ARGS="${build_arg}"
  else
    DOCKER_BUILD_ARGS="${DOCKER_BUILD_ARGS} ${build_arg}"
  fi
}

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
SH_DIR=$(cd "$(dirname "$0")" > /dev/null 2>&1 && pwd) 
REPO_DIR=$(cd "${SH_DIR}/../.." && pwd)

: "${WORKSPACE_DIR:=${REPO_DIR}}"
: "${DOCKER_CONTAINER:=rmw_connextdds-dev}"
: "${DOCKER_USER:=${LOGNAME}}"
: "${DOCKER_UID:=$(id -u)}"
: "${DOCKER_GID:=$(id -g)}"
: "${DOCKER_IMAGE:="rmw_connextdds:latest"}"
: "${DOCKER_DIR:=${REPO_DIR}/connext_docker_workspace/resource/docker}"
: "${DOCKER_FILE_DEB:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.community}"
: "${DOCKER_FILE_RTIPKG:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.rtipkg}"
: "${DOCKER_FILE_HOST:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.host}"

# Remove the container if it exists and it's in "exited" status
if [ "$(check_container_status ${DOCKER_CONTAINER} exited)" ]; then
    docker rm ${DOCKER_CONTAINER} > /dev/null
fi

# Check if a container with the selected name is already running.
if [ "$(check_container_status ${DOCKER_CONTAINER} running)" ]; then
    docker exec -it -u ${DOCKER_USER} \
      --workdir "${WORKSPACE_DIR}" \
      ${DOCKER_CONTAINER} \
      /bin/bash \
      $@
    exit 0
fi

export_docker_arg BASE_IMAGE
export_docker_arg DOCKER_USER
export_docker_arg DOCKER_UID
export_docker_arg DOCKER_GID

if [ -n "${CONNEXTDDS_COMMUNITY}" ]; then
  DOCKER_FILE="${DOCKER_FILE_DEB}"
elif [ -n "${CONNEXTDDS_HOST_DIR}" ]; then
  DOCKER_FILE="${DOCKER_FILE_HOST}"
  export_docker_arg CONNEXTDDS_HOST_DIR
  export_docker_arg CONNEXTDDS_ARCH
else
  DOCKER_FILE="${DOCKER_FILE_RTIPKG}"
  export_docker_arg CONNEXTDDS_VERSION
  export_docker_arg CONNEXTDDS_HOST_ARCH
  export_docker_arg CONNEXTDDS_ARCH
  export_docker_arg CONNEXTDDS_INSTALLER_HOST
  export_docker_arg CONNEXTDDS_INSTALLER_TARGET
  export_docker_arg CONNEXTDDS_INSTALLER_LICENSE
fi

set -x

docker build "${DOCKER_DIR}" \
  -f "${DOCKER_FILE}" \
  -t "${DOCKER_IMAGE}" \
  ${DOCKER_BUILD_ARGS}

docker run -it --rm \
  --network host \
  -v "${WORKSPACE_DIR}:/workspace" \
  --name "${DOCKER_CONTAINER}" \
  --user "${DOCKER_USER}" \
  --workdir /workspace \
  "${DOCKER_IMAGE}" \
  $@
