#!/usr/bin/env bash
set -euo pipefail

sync_time() {
    echo "Syncing time to resolve Docker sync conflicts..."

    sudo timedatectl set-local-rtc 1 --adjust-system-clock || true
    TZ=$(curl -s https://ipapi.co/timezone || true)
    if [[ -n "$TZ" ]]; then
        sudo timedatectl set-timezone "$TZ"
    fi
    sudo timedatectl set-ntp true || true
}

detect_os() {
    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        OS_ID="$ID"
        OS_LIKE="${ID_LIKE:-}"
        OS_VERSION_CODENAME="${VERSION_CODENAME:-}"
        echo "Detected OS: $OS_ID ($OS_LIKE), version codename: $OS_VERSION_CODENAME"
    else
        echo "Cannot detect OS. Exiting."
        exit 1
    fi
    ARCH=$(uname -m)
    echo "Detected architecture: $ARCH"
}

detect_gpus() {
    if ! command -v lspci &> /dev/null; then
        echo "lspci not found, skipping GPU detection"
        return
    fi
    GPU_INFO=$(lspci -nn | grep -i 'vga\|3d\|display' || true)
    echo "Detected GPUs:"
    echo "$GPU_INFO"
    echo

    HAS_INTEL=false
    HAS_INTEL_ARC=false
    HAS_NVIDIA=false
    HAS_AMD=false

    if echo "$GPU_INFO" | grep -iq 'Intel'; then
        HAS_INTEL=true
        if echo "$GPU_INFO" | grep -iq 'arc'; then
            HAS_INTEL_ARC=true
        fi
    fi

    if echo "$GPU_INFO" | grep -iq 'NVIDIA'; then
        HAS_NVIDIA=true
    fi

    if echo "$GPU_INFO" | grep -iq 'AMD\|ATI'; then
        HAS_AMD=true
    fi
}

setup_gpu() {
    echo "Installing GPU dependencies..."

    if [[ "$OS_ID" == "arch" || "$OS_LIKE" == *"arch"* ]]; then
        sudo pacman -Syu --needed --noconfirm vulkan-icd-loader vulkan-tools mesa

        if $HAS_INTEL; then
            sudo pacman -Syu --needed --noconfirm vulkan-intel
        fi

        if $HAS_NVIDIA; then
            sudo pacman -Syu --needed --noconfirm nvidia-utils nvidia-container-toolkit
        fi
    elif [[ "$OS_ID" == "ubuntu" || "$OS_ID" == "debian" || "$OS_LIKE" == *"debian"* ]]; then
        sudo apt-get update
        sudo apt-get install -y vulkan-icd-loader vulkan-tools mesa-vulkan-drivers || true

        if $HAS_INTEL; then
            sudo apt-get install -y vulkan-intel || true
        fi

        if $HAS_NVIDIA; then
            # Install NVIDIA driver
            if command -v ubuntu-drivers &> /dev/null; then
                NVIDIA_DRIVER=$(ubuntu-drivers devices | grep recommended | awk '{print $3}')
                if [ -n "$NVIDIA_DRIVER" ]; then
                    sudo apt-get install -y "$NVIDIA_DRIVER"
                else
                    sudo apt-get install -y nvidia-driver || true
                fi
            else
                sudo apt-get install -y nvidia-driver || true
            fi

            # NVIDIA container toolkit
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
            curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
            sudo apt-get update
            sudo apt-get install -y nvidia-container-toolkit
        fi
    elif [[ "$OS_ID" == "fedora" || "$OS_LIKE" == *"rhel"* || "$OS_LIKE" == *"fedora"* ]]; then
        sudo dnf install -y \
            mesa-vulkan-drivers \
            vulkan-tools \
            mesa-dri-drivers \
            mesa-libGL
    
        if $HAS_INTEL; then
            sudo dnf install -y mesa-vulkan-drivers
        fi

        if $HAS_NVIDIA; then
            if [[ "$OS_ID" == "fedora" ]]; then
                sudo dnf install -y \
                    https://download1.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm \
                    https://download1.rpmfusion.org/nonfree/fedora/rpmfusion-nonfree-release-$(rpm -E %fedora).noarch.rpm
        
                sudo dnf install -y akmod-nvidia xorg-x11-drv-nvidia-cuda
            else
                sudo dnf install -y epel-release
                sudo dnf config-manager --set-enabled crb || true
        
                sudo dnf install -y \
                    https://developer.download.nvidia.com/compute/cuda/repos/rhel$(rpm -E %rhel)/x86_64/cuda-rhel$(rpm -E %rhel).repo
        
                sudo dnf install -y nvidia-driver
            fi
        
            sudo dnf install -y nvidia-container-toolkit
        fi
    fi
    # Fix zed permissions
    sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2b03\", MODE=\"0666\"" > /etc/udev/rules.d/99-zed.rules'
}

setup_os() {
    echo "Setting up OS-specific packages..."

    if [[ "$OS_ID" == "arch" || "$OS_LIKE" == *"arch"* ]]; then
        echo "Arch-based distro detected."

        # Install/update yay
        CONFLICTS=$(sudo pacman -Qq 2>/dev/null | grep '^yay' || true)
        if [[ -n "$CONFLICTS" ]]; then
            echo "Removing conflicting packages: $CONFLICTS"
            sudo pacman -Rns --noconfirm $CONFLICTS > /dev/null
        fi
        echo "Installing/updating AUR helper (yay)..."
        sudo pacman -S --needed --noconfirm base-devel git
        rm -rf yay
        git clone https://aur.archlinux.org/yay.git
        cd yay
        makepkg -si --noconfirm
        cd ..
        rm -rf yay

        # Remove conflicting packages
        for pkg in docker.io docker-doc podman-docker containerd runc; do
            yay -Rns --noconfirm $pkg || true  > /dev/null
        done

        yay -S --noconfirm docker docker-buildx xorg-xwayland visual-studio-code-bin python-hjson jq
    elif [[ "$OS_ID" == "ubuntu" || "$OS_ID" == "debian" || "$OS_LIKE" == *"debian"* ]]; then
        echo "Debian/Ubuntu-based distro detected."

        # Remove conflicting packages
        for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
            sudo apt-get remove -y $pkg || true > /dev/null
        done

        # Docker repo setup
        sudo mkdir -p /etc/apt/keyrings
        curl -fsSL "https://download.docker.com/linux/$OS_ID/gpg" | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/$OS_ID $OS_VERSION_CODENAME stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

        sudo apt-get update

        # Install main packages
        sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin xwayland \
            ca-certificates curl gnupg lsb-release python3-pip software-properties-common apt-transport-https wget jq iptables iptables-persistent nftables

        # Install hjson
        if [[ "$ARCH" = "aarch64" ]]; then
            GET=https://github.com/hjson/hjson-go/releases/download/v4.5.0/hjson_v4.5.0_linux_arm64.tar.gz
        else
            GET=https://github.com/hjson/hjson-go/releases/download/v4.5.0/hjson_v4.5.0_linux_amd64.tar.gz
        fi
        curl -sSL $GET | sudo tar -xz -C /usr/local/bin

        # Install VSCode
        wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
        sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
        sudo apt update
        sudo apt install -y code
    elif [[ "$OS_ID" == "fedora" || "$OS_LIKE" == *"rhel"* || "$OS_LIKE" == *"fedora"* ]]; then
        echo "RPM-based distro detected."

        sudo dnf install -y dnf-plugins-core curl wget jq git
        sudo dnf remove -y podman podman-docker containerd runc || true > /dev/null

        sudo dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo || \
        sudo dnf config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo
        
        sudo dnf install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin xorg-x11-server-Xwayland

        sudo rpm --import https://packages.microsoft.com/keys/microsoft.asc
        
		cat <<-EOF | sudo tee /etc/yum.repos.d/vscode.repo
		[code]
		name=Visual Studio Code
		baseurl=https://packages.microsoft.com/yumrepos/vscode
		enabled=1
		gpgcheck=1
		gpgkey=https://packages.microsoft.com/keys/microsoft.asc
		EOF
        sudo dnf install -y code

        if [[ "$ARCH" = "aarch64" ]]; then
            GET=https://github.com/hjson/hjson-go/releases/download/v4.5.0/hjson_v4.5.0_linux_arm64.tar.gz
        else
            GET=https://github.com/hjson/hjson-go/releases/download/v4.5.0/hjson_v4.5.0_linux_amd64.tar.gz
        fi
        curl -sSL $GET | sudo tar -xz -C /usr/local/bin
    else
        echo "Unsupported OS: $OS_ID"
        exit 1
    fi

    # Fix Cube Orange Permissions
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", MODE="0666"' | sudo tee /etc/udev/rules.d/99-cubepilot.rules > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

setup_docker() {
    echo "Configuring Docker..."
    sudo groupadd -f docker
    sudo usermod -aG docker $USER
    sudo systemctl enable --now docker
}

setup_vscode_extensions() {
    if [[ -f .vscode/extensions.json ]]; then
        echo "Installing VSCode extensions..."
        extensions=$(jq -r '.recommendations[]' .vscode/extensions.json)
        for extension in $extensions; do
            if ! code --list-extensions | grep -q "$extension"; then
                echo "Installing $extension..."
                code --install-extension "$extension" || echo "Failed: $extension"
            fi
        done
    else
        echo "No .vscode/extensions.json found. Skipping extensions."
    fi
}

setup_vscode_settings() {
    echo "Configuring VSCode port forwarding..."
    VSC_DIR="$HOME/.config/Code/User/"
    VSC_CONFIG="$VSC_DIR/settings.json"
    mkdir -p $VSC_DIR
    touch $VSC_CONFIG
    hjson -j $VSC_CONFIG > $VSC_CONFIG.tmp && mv $VSC_CONFIG.tmp $VSC_CONFIG
    jq '.["remote.autoForwardPorts"] = false' $VSC_CONFIG > $VSC_CONFIG.tmp && mv $VSC_CONFIG.tmp $VSC_CONFIG
}

setup_headless_devcontainer() {
    if ! type nvm &> /dev/null; then
        echo "Installing NVM..."
        export NVM_DIR="$HOME/.nvm"
        mkdir -p "$NVM_DIR"
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    else
        echo "NVM installation found."
    fi
    nvm install --lts
    nvm use --lts
    
    echo "Installing devcontainers CLI..."
    npm install -g @devcontainers/cli
}

run_prebuild() {
    echo "Running prebuild script..."
    bash .devcontainer/prebuild.sh || true
}

main() {
    sync_time
    detect_os
    detect_gpus

    setup_os
    setup_gpu
    setup_docker
    setup_vscode_extensions
    setup_vscode_settings
    setup_headless_devcontainer
    run_prebuild
    bash mdns.sh || true
    bash clone_optional.sh || true

    echo "Setup completed successfully!"
}

main