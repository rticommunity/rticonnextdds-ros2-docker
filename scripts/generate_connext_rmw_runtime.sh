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
set -e
SH_DIR=$(cd "$(dirname "$0")" > /dev/null 2>&1 && pwd) 
REPO_DIR=$(cd "${SH_DIR}/.." && pwd)

# Load common helper functions
. "${SH_DIR}/common.sh"

if [ $# -ne 2 ]; then
  log_warning "invalid arguments. Usage:"
  log_warning "${0} CONNEXTDDS_DIR CONNEXTDDS_ARCH"
  exit 1
fi

CONNEXTDDS_DIR="${1}"
CONNEXTDDS_ARCH="${2}"
CONNEXTDDS_HOST_DIR=$(basename "${CONNEXTDDS_DIR}")
CONNEXTDDS_ARCHIVE="${CONNEXTDDS_HOST_DIR}-${CONNEXTDDS_ARCH}-rmw-runtime.tar.gz"

log_info "packaging installation: ${CONNEXTDDS_DIR}"
log_info "target architecture: ${CONNEXTDDS_ARCH}"

OUTPUT_DIR=$(pwd)
cd "$(dirname "${CONNEXTDDS_DIR}")"

if [ -f "${CONNEXTDDS_ARCHIVE}" ]; then
  log_warning "deleting existing file: ${CONNEXTDDS_ARCHIVE}"
  rm "${CONNEXTDDS_ARCHIVE}"
fi

log_info "generating archive: ${CONNEXTDDS_ARCHIVE}"
(
  set -x
  tar czf ${CONNEXTDDS_ARCHIVE} \
          ${CONNEXTDDS_HOST_DIR}/include \
          ${CONNEXTDDS_HOST_DIR}/lib/${CONNEXTDDS_ARCH} \
          ${CONNEXTDDS_HOST_DIR}/resource/cmake \
          ${CONNEXTDDS_HOST_DIR}/resource/scripts \
          ${CONNEXTDDS_HOST_DIR}/rti_versions.xml
)

log_info "generated: $(pwd)/${CONNEXTDDS_ARCHIVE}"
