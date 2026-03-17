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

echo "[postcreate] GPU permissions..."
sudo chown -R $USER:$USER /dev 2> /dev/null
sudo chown -R $USER:$USER /workspace/venv || true 2> /dev/null
sudo chgrp video /dev/nvhost-gpu /tmp/argus_socket || true 2> /dev/null
sudo chmod 660 /dev/nvhost-gpu /tmp/argus_socket || true 2> /dev/null

echo "[postcreate] Done!"
