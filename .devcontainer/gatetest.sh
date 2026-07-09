#!/bin/bash
# Water session 1 checkout: gate test. See watertest.sh for knobs/abort.
TEST=gate exec "$(dirname "$0")/watertest.sh" "$@"
