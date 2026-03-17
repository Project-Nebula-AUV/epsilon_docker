#!/usr/bin/env bash
set -euo pipefail

echo "[INFO] Installing dependencies for ZED SDK on $TARGET_PLATFORM..."

if [[ "$TARGET_PLATFORM" == "nvidia" || "$TARGET_PLATFORM" == "jetson" ]]; then

    apt-get update -y
    apt-get install -y --no-install-recommends \
        lsb-release wget less udev zstd sudo build-essential cmake \
        python3 python3-pip libpng-dev libgomp1 file

    python3 -m pip install --upgrade pip
    python3 -m pip install numpy opencv-python pyopengl

    if [ "$TARGET_PLATFORM" = "nvidia" ]; then
        ZED_RUN_FILE="ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run"
        ZED_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_VERSION}/cu${CUDA_VERSION%%.*}/ubuntu${UBUNTU_RELEASE_YEAR}"
        echo "[INFO] Downloading ZED SDK for desktop from $ZED_URL ..."
    elif [ "$TARGET_PLATFORM" = "jetson" ]; then
        ZED_RUN_FILE="ZED_SDK_Linux.run"
        ZED_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_VERSION}/l4t${L4T_VERSION}/jetsons"
        echo "[INFO] Downloading ZED SDK for Jetson L4T $L4T_VERSION from $ZED_URL ..."
        apt-get install -y --no-install-recommends nvidia-cuda-toolkit 
    fi

    wget -q -O "$ZED_RUN_FILE" "$ZED_URL"

    if ! file "$ZED_RUN_FILE" | grep -q 'executable'; then
        echo "[ERROR] Downloaded file is not a valid .run executable. Check the ZED SDK URL for your version."
        exit 1
    fi

    chmod +x "$ZED_RUN_FILE"
    echo "[INFO] Installing ZED SDK..."
    ./"$ZED_RUN_FILE" silent

    ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so || true

    echo "[INFO] Fixing permissions..."
    chown -R $USERNAME:$USERNAME /usr/local/zed || true

    echo "[INFO] Setting environment variables..."

    BASHRC="$HOME/.bashrc"
    if ! grep -q 'ZED_DIR=/usr/local/zed' "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# ZED SDK environment variables" >> "$BASHRC"
        echo "export ZED_DIR=/usr/local/zed" >> "$BASHRC"
        echo "export CMAKE_PREFIX_PATH=/usr/local/zed:\$CMAKE_PREFIX_PATH" >> "$BASHRC"
        echo "export PATH=/workspace/venv/bin:\$PATH" >> "$BASHRC"
        echo "export LD_LIBRARY_PATH=/usr/local/zed/lib:\$LD_LIBRARY_PATH" >> "$BASHRC"
        echo "export PYTHONPATH=/workspace/venv/lib/python3.10/site-packages:\$PYTHONPATH" >> "$BASHRC"
        echo "[INFO] Environment variables appended to $BASHRC"
    fi

    rm -rf "$ZED_RUN_FILE" /var/lib/apt/lists/*
    mkdir -p ~/Documents/ZED/

    echo "[INFO] ZED SDK installation complete on $TARGET_PLATFORM"
else
    echo "[INFO] $TARGET_PLATFORM not supported, skipping installation" 
fi