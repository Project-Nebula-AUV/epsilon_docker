#!/bin/bash
# Water session 1 checkout: roll test. See watertest.sh for knobs/abort.
TEST=roll exec "$(dirname "$0")/watertest.sh" "$@"
