#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OS_TYPE="unknown"

# -------------------------
# Detect OS / platform
# -------------------------
if [ -n "${WSL_DISTRO_NAME:-}" ]; then
    OS_TYPE="wsl"
elif [ "$(uname -s)" = "Darwin" ]; then
    OS_TYPE="mac"
elif [ "$(uname -s)" = "Linux" ]; then
    ARCH="$(uname -m)"
    if [ "$ARCH" = "aarch64" ]; then
        # Check for Jetson by looking for Tegra device tree
        if [ -f /proc/device-tree/model ] && grep -qi "jetson" /proc/device-tree/model; then
            OS_TYPE="jetson"
        elif [ -f /etc/rpi-issue ]; then
            OS_TYPE="rpi"
        else
            OS_TYPE="linux"
        fi
    else
        OS_TYPE="linux"
    fi
else
    echo "[ERROR] Unsupported OS detected, exiting..."
    exit 1
fi

# -------------------------
# Detect NVIDIA GPU for Linux desktop
# -------------------------
if [ "$OS_TYPE" = "linux" ]; then
    if command -v nvidia-smi &>/dev/null; then
        echo "[INFO] NVIDIA GPU detected, using NVIDIA override"
        OS_TYPE="nvidia"
    else
        echo "[INFO] No NVIDIA GPU detected, using standard Linux override"
    fi
fi

# -------------------------
# Apply override
# -------------------------
OVERRIDE_FILE="${SCRIPT_DIR}/docker-compose.override.${OS_TYPE}.yml"

if [ ! -f "$OVERRIDE_FILE" ]; then
    echo "[ERROR] Override file not found: $OVERRIDE_FILE"
    exit 1
fi

cp "$OVERRIDE_FILE" "${SCRIPT_DIR}/docker-compose.override.yml"
echo "[INFO] Applied override: $OVERRIDE_FILE"
