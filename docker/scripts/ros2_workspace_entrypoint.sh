#!/bin/bash

/fix_gcc.sh

if [ "$@" = "__default__" ]; then
  exec /bin/bash
else
  exec "$@"
fi
