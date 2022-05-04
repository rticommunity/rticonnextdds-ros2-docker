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
# By default, the script will try to detect and use a host installation, by
# inspecting variables CONNEXTDDS_DIR, and NDDSHOME (in this order). If any of
# these variables is set to a non-empty value, the script will assume it points
# to a Connext installation and it will copy it into the container image. If
# multiple target libraries are installed, you must use CONNEXTDDS_ARCH to
# specify the architecture to use.
#
# If Connext is not installed on the host (or if you want to use a different
# version than the one used by the host), RTI Connext DDS can also be installed
# using the official installers provided by RTI. Set variable
# CONNEXTDDS_FROM_RTIPKG to a non-empty value to use this mode.
#
# The location of the required files must be specified using variables
# CONNEXTDDS_INSTALLER_HOST, CONNEXTDDS_INSTALLER_TARGET, and
# CONNEXTDDS_INSTALLER_LICENSE.
#
# Finally, you can also build a container image using the community-licensed
# version of Connext DDS which is distributed by Debian package
# rti-connext-dds-6.0.1.
# This image can only be used for pre-production and non-commercial purposes.
# In order to use it, set variable CONNEXTDDS_FROM_DEB to a non-empty value,
# e.g.: `export CONNEXTDDS_FROM_DEB=y`.
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
# Helper function to log messages to the console
################################################################################
log_info()
{
  printf -- "\e[0;32m-- %s\e[0;0m\n" "$@"
}

################################################################################
# Begin script
################################################################################
SH_CWD=$(pwd)
SH_DIR=$(cd "$(dirname "$0")" > /dev/null 2>&1 && pwd) 
REPO_DIR=$(cd "${SH_DIR}/../.." && pwd)

: "${WORKSPACE_DIR:=${REPO_DIR}}"
: "${DOCKER_CONTAINER:=rmw_connextdds-dev}"
: "${DOCKER_USER:=${LOGNAME}}"
: "${DOCKER_UID:=$(id -u)}"
: "${DOCKER_GID:=$(id -g)}"
: "${DOCKER_IMAGE:="rmw_connextdds:latest"}"
: "${DOCKER_DIR:=${REPO_DIR}/connext_docker_workspace/resource/docker}"
: "${DOCKER_FILE_DEB:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.deb}"
: "${DOCKER_FILE_RTIPKG:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.rtipkg}"
: "${DOCKER_FILE_HOST:=${DOCKER_DIR}/Dockerfile.rmw_connextdds.host}"
: "${CONNEXTDDS_VERSION:=6.1.0}"
: "${CONNEXTDDS_HOST_ARCH:=x64Linux}"
: "${CONNEXTDDS_ARCH:=x64Linux4gcc7.3.0}"

# Remove the container if it exists and it's in "exited" status
if [ "$(check_container_status ${DOCKER_CONTAINER} exited)" ]; then
    log_info "removing docker container: ${DOCKER_CONTAINER}"
    docker rm ${DOCKER_CONTAINER} > /dev/null
fi

# Check if a container with the selected name is already running.
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

# Determine the method of installation for RTI Connext DDS:
# - If CONNEXTDDS_FROM_DEB, use Debian package-based installation
# - If CONNEXTDDS_FROM_RTIPKG, use official RTI installers.
# - Otherwise default to use host installation
DOCKER_FILE="${DOCKER_FILE_HOST}"
if [ -n "${CONNEXTDDS_FROM_DEB}" ]; then
  DOCKER_FILE="${DOCKER_FILE_DEB}"
elif [ -n "${CONNEXTDDS_FROM_RTIPKG}" ]; then
  DOCKER_FILE="${DOCKER_FILE_RTIPKG}"
fi


# The Dockerfiles expect all external files to be provided in the
# build context's "archives/" subdirectory. We achieve this by collecting
# all files into a temporary tar.gz archive.
: "${DOCKER_BUILD_TAG:=$(date +%s)}"
: "${DOCKER_BUILD_CONTEXT:=/tmp/rmw_connextdds-docker-ctx-${DOCKER_BUILD_TAG}}"

(
  # Create initial context with files shared by all images
  rm -rf "${DOCKER_BUILD_CONTEXT}"
  mkdir -p "${DOCKER_BUILD_CONTEXT}/archives"
  cd "${DOCKER_BUILD_CONTEXT}"
  cp -r "${REPO_DIR}/connext_docker_workspace/resource/docker/scripts" .
  cp "${DOCKER_FILE}" Dockerfile
  log_info "generating context archive: ${DOCKER_BUILD_CONTEXT}.tar"
  tar cf "${DOCKER_BUILD_CONTEXT}.tar" *
  rm -rf "${DOCKER_BUILD_CONTEXT}"
)
# Create a trap to delete the context archive on exit
trap 'rm -rf "${DOCKER_BUILD_CONTEXT}.tar"; trap - EXIT; exit' EXIT INT HUP TERM

# Helper function to add files to the context tar archive
add_to_context()
{
  local file_path="${1}" \
    ctx_prefix="${2}"
  
  # make sure ctx_prefix doesn't end with a /
  ctx_prefix=${ctx_prefix%%/}

  (
    cd $(dirname "${file_path}")
    # This operation requires GNU tar and its "--transform" option to
    # prepend a prefix while adding the file
    file_name="$(basename "${file_path}")"
    log_info "adding to context: ${file_path} -> ${ctx_prefix}/${file_name}"
    tar rf "${DOCKER_BUILD_CONTEXT}.tar" \
      --transform "s+^+${ctx_prefix}/+" \
      "${file_name}"
  )
}

export_docker_arg BASE_IMAGE
export_docker_arg DOCKER_USER
export_docker_arg DOCKER_UID
export_docker_arg DOCKER_GID
export_docker_arg RMW_CONNEXTDDS_URL
export_docker_arg RMW_CONNEXTDDS_BRANCH

case "${DOCKER_FILE}" in
${DOCKER_FILE_DEB})
  # Nothing else to do for this installation method
  ;;
${DOCKER_FILE_RTIPKG})
  # Detect paths of required files and add them to the context archive
  : "${CONNEXTDDS_INSTALLER_HOST:=${SH_CWD}/rti_connext_dds-6.1.0-pro-host-x64Linux.run}"
  : "${CONNEXTDDS_INSTALLER_TARGET:=${SH_CWD}/rti_connext_dds-6.1.0-pro-target-x64Linux4gcc7.3.0.rtipkg}"
  : "${CONNEXTDDS_INSTALLER_LICENSE:=${SH_CWD}/rti_license.dat}"

  add_to_context "${CONNEXTDDS_INSTALLER_HOST}" archives/
  add_to_context "${CONNEXTDDS_INSTALLER_TARGET}" archives/
  add_to_context "${CONNEXTDDS_INSTALLER_LICENSE}" archives/

  # The Dockerfiles expect the installer variables to only contain the file
  # names, so update the variables and export them as build arguments.
  CONNEXTDDS_INSTALLER_HOST=$(basename "${CONNEXTDDS_INSTALLER_HOST}")
  CONNEXTDDS_INSTALLER_TARGET=$(basename "${CONNEXTDDS_INSTALLER_TARGET}")
  CONNEXTDDS_INSTALLER_LICENSE=$(basename "${CONNEXTDDS_INSTALLER_LICENSE}")

  export_docker_arg CONNEXTDDS_INSTALLER_HOST
  export_docker_arg CONNEXTDDS_INSTALLER_TARGET
  export_docker_arg CONNEXTDDS_INSTALLER_LICENSE
  export_docker_arg CONNEXTDDS_VERSION
  export_docker_arg CONNEXTDDS_ARCH
  ;;
${DOCKER_FILE_HOST})
  if [ -z "${CONNEXTDDS_DIR}" -a -n "${NDDSHOME}" ]; then
    CONNEXTDDS_DIR="${NDDSHOME}"
  else
    printf "ERROR: no Connext DDS installation detected.\n" >&2
    exit 1
  fi
  add_to_context "${CONNEXTDDS_DIR}" archives/

  CONNEXTDDS_HOST_DIR=$(basename "${CONNEXTDDS_DIR}")
  export_docker_arg CONNEXTDDS_HOST_DIR
  export_docker_arg CONNEXTDDS_ARCH
  ;;
esac

log_info "building docker image: ${DOCKER_IMAGE}"
(
  set -x
  cat "${DOCKER_BUILD_CONTEXT}.tar" | docker build - \
    -t "${DOCKER_IMAGE}" \
    ${DOCKER_BUILD_ARGS}
)

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
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${DOCKER_IMAGE}" \
    $@
)