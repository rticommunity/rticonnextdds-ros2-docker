#!/bin/bash

source ${RMW_CONNEXTDDS_INSTALL_DIR}/setup.bash

if [ "$@" = "__default__" ]; then
  exec /bin/bash
else
  exec "$@"
fi
