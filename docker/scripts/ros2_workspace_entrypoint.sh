#!/bin/bash

if [ -f /opt/rti.com/rti_connext_dds.arch ]; then
  export CONNEXTDDS_ARCH=$(cat /opt/rti.com/rti_connext_dds.arch)
fi

/fix_gcc.sh

if [ "$@" = "__default__" ]; then
  exec /bin/bash
else
  exec "$@"
fi
