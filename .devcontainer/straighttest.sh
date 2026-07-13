#!/bin/bash
# Closed-loop straight-line test: hold heading+depth while driving forward.
# Knobs: ROBOSUB_SURGE (0.3), ROBOSUB_STRAIGHT_S (20), ROBOSUB_TEST_DEPTH (1.2)
TEST=straight exec "$(dirname "$0")/watertest.sh" "$@"
