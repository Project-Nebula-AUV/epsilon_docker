#!/usr/bin/env bash
set -e

if grep -Fxvf ~/.bashrc /sitl.bashrc >/dev/null; then
    cat /sitl.bashrc >> ~/.bashrc
fi

touch ~/.helper.txt

if grep -Fxvf ~/.helper.txt /sitl.helper.txt >/dev/null; then
    cat /sitl.helper.txt > ~/.helper.txt
fi

exec "$@"