#!/usr/bin/env bash

echo "[postcreate] Appending custom bashrc..."
cat .devcontainer/dev.bashrc >> ~/.bashrc

echo "[postcreate] Creating helper text file..."
touch ~/.helper.txt
cat .devcontainer/dev.helper.txt >> ~/.helper.txt

echo "[postcreate] Updating apt packages..."
sudo apt update -y

echo "[postcreate] Updating rosdep..."
rosdep update

echo "[postcreate] Install geoid dataset for mavros..."
#sudo geographiclib-get-geoids egm96-5
sudo wget -O /tmp/egm96-5.tar.bz2 'https://psychz.dl.sourceforge.net/project/geographiclib/geoids-distrib/egm96-5.tar.bz2?viasf=1' #this link came to me in a dream
sudo tar -xf /tmp/egm96-5.tar.bz2 -C /usr/share/GeographicLib/
sudo rm -rf /tmp/egm96-5.tar.bz2

echo "[postcreate] Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -y \
  --skip-keys="$(tr '\n' ' ' < .devcontainer/package-ignore.txt)"

cd /tmp && \
    git clone https://github.com/WiringPi/WiringPi && \
    cd WiringPi && \
    ./build && \
    rm -rf /tmp/WiringPi

# echo "[postcreate] GPU permissions..."
# sudo chown -R $USER:$USER /dev 2> /dev/null
# sudo chown -R $USER:$USER /workspace/venv || true 2> /dev/null
# sudo chgrp video /dev/nvhost-gpu /tmp/argus_socket || true 2> /dev/null
# sudo chmod 660 /dev/nvhost-gpu /tmp/argus_socket || true 2> /dev/null
sudo ldconfig

echo "[postcreate] Fixing NumPy 2 / cv_bridge ABI mismatch..."
# The ROS-distro cv_bridge is compiled against NumPy 1.x, but this container's venv uses NumPy 2,
# which makes cv_bridge's imgmsg_to_cv2 SEGFAULT (breaks the entire camera/vision path).
# Root cause: /usr/include/python3.10/numpy is a symlink to the apt NumPy 1.x headers and sits on
# the default -I include path (via python3-config), so it shadows the venv's NumPy 2 headers at
# compile time. Removing it lets C-extension builds resolve <numpy/...> to the venv NumPy 2.
# Runtime numpy is unaffected (this only touches compile-time headers).
sudo rm -f /usr/include/python3.10/numpy
# Rebuild cv_bridge (vendored in src/cv_bridge from ros-perception/vision_opencv, humble) against
# NumPy 2 so the overlay shadows the broken distro build.
if [ -d ~/robosub_ws/src/cv_bridge ]; then
    cd ~/robosub_ws
    source /opt/ros/humble/setup.bash
    # -j1 / sequential: cv_bridge pulls in heavy OpenCV headers; on the 2GB Pi a parallel
    # compile OOM-thrashes the host. Single-threaded is slower but safe.
    MAKEFLAGS="-j1" colcon build --packages-select cv_bridge --executor sequential \
        --cmake-args -DPython3_EXECUTABLE=/workspace/venv/bin/python3 -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
        || echo "[postcreate] WARN: cv_bridge rebuild failed; rebuild manually with the same --cmake-args"
fi


# Sensor driver runtime deps. The epsilon_sensors imu/depth nodes run under SYSTEM python3
# (install shebang is #!/usr/bin/python3), so these must live in system python, not just the venv.
# imu: adafruit-blinka (board) + adafruit-circuitpython-bno055 + lgpio; depth: smbus2 (ms5837 is vendored).
echo "[postcreate] Installing sensor driver deps (system python3)..."
sudo pip3 install adafruit-blinka adafruit-circuitpython-bno055 lgpio smbus2 \
    || echo "[postcreate] WARN: sensor dep install failed; install manually with: sudo pip3 install ..."

# Belt-and-suspenders device access for the IMU (BNO055) / depth (MS5837) drivers.
# Primary mechanism is group_add (986 gpio / 988 i2c) in docker-compose.override.rpi.yml;
# this chmod covers the case where that group membership is not applied for any reason.
echo "[postcreate] Ensuring gpio/i2c device access..."
sudo chmod a+rw /dev/gpiochip0 /dev/i2c-1 /dev/i2c-2 2>/dev/null || true

echo "[postcreate] Done!"
