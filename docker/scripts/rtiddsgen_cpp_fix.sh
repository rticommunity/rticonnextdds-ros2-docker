#!/bin/sh

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

detect_cpp_major_version()
{
  cpp_version=$(
    cpp --version | head -1 | rev | awk '{print $1;}' | rev) 2>/dev/null
  
  if [ -z "${cpp_version}" ]; then
    log_error "failed to detect C++ preprocessor (cpp)"
    log_error "rtiddsgen will not work properly unless you disable the C preprocessor feature."
    return 1
  fi
  cpp_major_version="$(printf -- "%s" "${cpp_version}" | cut -d. -f1)"
  if ! printf -- "%s" "${cpp_major_version}" |
        grep -qE "[1-9][0-9]*"; then
    log_error "failed to detect C++ preprocessor's version"
    log_error "Please make sure that you are using gcc 10 or earlier."
    return 2
  fi
  printf -- "%s" "${cpp_major_version}"
}

rtiddsgen_check_cpp()
{
  cpp_major_version=$(detect_cpp_major_version)

  if [ $? -ne 0 ]; then
    return -1
  fi

  if [ "${cpp_major_version}" -le 10 ] 2>/dev/null; then
    return 0
  fi

  log_warning "current cpp version (${cpp_major_version}) is incompatible with rtiddsgen (11 or newer)" >&2

  # Try to install gcc 10 if not already installed
  if which cpp-10 > /dev/null 2>&1; then
    log_warning "cpp-10 is already installed."
    log_warning "You should run rtiddsgen with arguments:\`-ppPath cpp-10 -ppOption -C\`"
    return 0
  fi

  return 1
}

rtiddsgen_install_cpp()
{
  log_warning "Trying to install cpp-10..."
  if ! sudo apt-get update ||
        ! sudo apt-get install -y cpp-10; then
    log_error "failed to install cpp-10"
    log_error "rtiddsgen will not be available unless you install and select an older version.\n" >&2
    log_error "If you prefer not to change your global cpp version, you may also:\n" >&2
    log_error "- select a different C preprocessor with option \`%s\`\n" "-ppPath" >&2
    log_error "- disable rtiddsgen's C preprocessor feature with \`%s\`\n" "-ppDisable" >&2
    return 1
  fi
  if ! which cpp-10 >/dev/null 2>&1; then
    log_error "failed to detect cpp-10 after installation"
    return 2
  fi
  log_info "cpp-10 is now installed."
  log_info "You can run rtiddsgen with arguments:\`-ppPath cpp-10 -ppOption -C\`"
}
