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

# Some helper functions to print log messages
log_msg()
{
  local lvl="${1}"
        color="${2}" \
        msg="${3}"

  printf -- "${color}[%s] -- %s\e[0;0m\n" "${lvl}" "${msg}"
}

log_warning()
{
  log_msg W "\e[0;33m" "${1}"
}

log_info()
{
  log_msg I "\e[0;32m" "${1}"
}

log_error()
{
  log_msg E "\e[0;31m" "${1}"
}

# Dynamically get the value of variable passed by name
get_var()
{
  eval "printf -- '%s' \"\${${1}}\""
}

# Dynamically set the value of variable passed by name
set_var()
{
  eval "${1}=\"${2}\""
}

# Build a list of arguments to pass to `docker build`. Add specified variable
# to arguments list only if it has a non-empty value.
# Argument names and values must not contain any spaces.
export_docker_build_arg()
{
  local args_list="${1}" \
        var_name="${2}"
  local args_val="$(get_var ${args_list})" \
        var_val="$(get_var ${var_name})"

  # Do nothing if the variable is empty
  [ -n "${var_val}" ] || return 0

  local build_arg="--build-arg ${var_name}=${var_val}"
  if [ -n "${args_val}" ]; then
    set_var "${args_list}" "${args_val} ${build_arg}"
  else
    set_var "${args_list}" "${build_arg}"
  fi
}

# Take a directory and create a temporary tar archive to use it as a 
# docker build context. Set a shell trap to delete the archive on exit.
create_docker_build_context()
{
  local build_context="${1}" \
        context_archive="${2}"
  
  [ -n "${context_archive}" ] || context_archive="${build_context}.tar"

  log_info "generating context archive: ${build_context} -> ${context_archive}"
  (
    # Create initial context with files shared by all images  
    cd "${build_context}"
    tar cf "${context_archive}" *
  )
  # Create a trap to delete the context archive on exit
  trap 'rm -rf "${context_archive}"; trap - EXIT; exit' EXIT INT HUP TERM
}

# Add a file to the tar archive used as docker build context
# Helper function to add files to the context tar archive
add_to_docker_build_context()
{
  local context_archive="${1}" \
        file_path="${2}" \
        ctx_prefix="${3}"
  
  # make sure ctx_prefix doesn't end with a /
  ctx_prefix=${ctx_prefix%%/}

  local file_name="$(basename "${file_path}")"

  log_info "adding to context: ${file_path} -> ${context_archive}:${ctx_prefix}/${file_name}"
  (
    cd $(dirname "${file_path}")
    # This operation requires GNU tar and its "--transform" option to
    # prepend a prefix while adding the file
    tar rf "${context_archive}" \
      --transform "s+^+${ctx_prefix}/+" \
      "${file_name}"
  )
}
