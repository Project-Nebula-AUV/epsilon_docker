#!/bin/bash
# Water session 1 checkout: hold test. See watertest.sh for knobs/abort.
TEST=hold exec "$(dirname "$0")/watertest.sh" "$@"
