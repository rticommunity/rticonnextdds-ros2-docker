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

# rtiddsgen does not currently support gcc 11. Check if the gcc version
# is compatible, otherwise print a warning for the user
(
  gcc_version=$(
    gcc --version | head -1 | rev | awk '{print $1;}' | rev) 2>/dev/null
  
  if [ "$(printf -- "%s" "${gcc_version}" | cut -d. -f1)" = 11 ]; then
    log_warning "current gcc version (${gcc_version}) is incompatible with rtiddsgen (11 or newer)" >&2

    # Try to install gcc 10
    (
      log_warning "Trying to install gcc 10..."
      sudo apt-get update
      sudo apt-get install -y gcc-10 g++-10
      log_warning "gcc 10 is now installed."
      log_warning "You should run rtiddsgen with arguments:\`-ppPath cpp-10 -ppOption -C\`"
    ) || (
      log_error "failed to install gcc 10"
      log_error "rtiddsgen will not be available unless you install and select an older version.\n" >&2
      log_error "If you prefer not to change your global gcc version, you may also:\n" >&2
      log_error "- select a different C preprocessor with option \`%s\`\n" "-ppPath" >&2
      log_error "- disable rtiddsgen's C preprocessor feature with \`%s\`\n" "-ppDisable" >&2
    )
  fi
)