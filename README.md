## Introduction

Welcome to the MHSeals Docker container, serving as an ultra-portable development tool, whether you are working with an NVIDIA Jetson, using a Raspberry Pi, running Windows, using an Arch-based distro, or using a Red Hat distro, you can run this container on anything.

## Installation

Clone the repo with submodules:

```bash
git clone --recurse-submodules https://github.com/1unarzDev/epsilon.git
```

If you didn't include the recurse-submodules flag, then run the following command to pull each needed submodule:

```
git submodule foreach '
  default_branch=$(git remote show origin | sed -n "/HEAD branch/s/.*: //p")
  echo "Pulling latest changes from $default_branch in $name"
  git fetch origin "$default_branch"
  git checkout "$default_branch"
  git pull origin "$default_branch"
'
```

For your convenience, an environment setup script for each OS has been provided. Simply run the following (`<OS>` corresponds to either `linux`, `mac`, or `windows`):

```bash
./setup.<OS>.sh
```

Alternatively, you may manually install the following necessary dependencies as you see fit:
- Docker
- NVIDIA Container Toolkit
- Docker Compose
- VSCode (or devcontainer CLI)
- Unity + Vulkan (if you're running the sim, and depending on the available graphics driver)

For manual installation, reference the OS-specific guides below.

### Linux

Not much to do here, but for manual Docker installation instructions, visit the [Docker Engine installation guide](https://docs.docker.com/engine/install/). Be sure to follow all instructions in the [Linux post-install guide](https://docs.docker.com/engine/install/linux-postinstall/). Also, be sure to get a text editor to work with the code and set up an X11 host if necessary (e.g., a Wayland-based WM).

> [!TIP]
> For those using editors other than VSCode, devcontainers offers a CLI tool. Start by installing NVM:
>
> ```bash
> curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
> ```
>
> Alternatively:
>
> ```
> wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
> ```
>
> Then, make the `nvm` command available by sourcing your shell configuration file. I would highly advise ZSH users to lazy load NVM by replacing the generated commands with the [zsh-nvm](https://github.com/lukechilds/zsh-nvm) plugin. Alternatively, replace it with a [lazy loading function](https://github.com/nvm-sh/nvm/issues/730).
>
> ```bash
> source ~/.bashrc # .zshrc or config.fish depending on your shell
> ```
>
> Then, install and use the latest LTS version of npm and Node.
>
> ```bash
> nvm install --lts
> nvm use --lts
> ```
>
> Finally, install the [devcontainers cli tool (more usage information here)](https://github.com/devcontainers/cli):
>
> ```bash
> npm install -g @devcontainers/cli
> ```

Anything else you would like to install manually, reference the installation scripts for help on figuring out how to install them.

### Windows

For manual installation, get the following programs:

- [VcXsrv (X server for display)](https://sourceforge.net/projects/vcxsrv/)
- [Git](https://git-scm.com/downloads)
- [VSCode](https://code.visualstudio.com/)
- [Docker Desktop](https://docs.docker.com/desktop/release-notes/)

**Start the Docker Daemon each time you want to work on the project by opening the Docker Desktop application.** The first time you install it, you will be prompted to restart your system.

### Mac

> [!IMPORTANT]
> If you are willing to troubleshoot installing a newer version of OpenGL on an X11 Server (XQuartz), follow the steps below and document what you do as much as possible. Otherwise, simply install Linux on your Mac and follow the Linux instructions as normal.

Identify your chip architecture (Intel or Apple Silicon) by running `uname -m`. If your system is an Intel-based Mac, it should output `x86_64`, and if it is Apple Silicon, it will show `arm64`.

Install the following programs through your preferred method:

- [XQuartz (X server for display)](https://www.xquartz.org/)
- [Git](https://git-scm.com/downloads)
- [VSCode](https://code.visualstudio.com/)
- [Docker Desktop](https://docs.docker.com/desktop/release-notes/)

Brew provides an easy way to install all of them at once. Start by installing Brew:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Now, install all of the needed packages:

```bash
brew install git --cask visual-studio-code docker xquartz
```

You will need to restart your system to use both Docker and XQuartz. If, for some reason, you are running a Hackintosh or a macOS VM, it is likely that Docker will complain about Hyper-V for virtualization. Depending on your setup, you will need to add these options: `+vmx,+smep,+smap,+hypervisor` to your VM/boot configuration. You will likely have to troubleshoot issues, but feel free to ask questions here.

After restarting, open XQuartz and enable `File > Preferences > Security > Allow connections from network clients`. **Each time you need to run a GUI application in the Docker container, be sure to run `xhost +` to give XQuartz access to X11 forwarding ports.** For more information, see [X11 Forwarding on macOS and Docker](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088). It may be beneficial to add a configuration to your system that runs this command automatically.

## Usage

### Simulation

There are three primary components to the simulation stack:

- Unity physics sim
- Ardupilot SITL control
- ROS navigation logic

For the Unity physics sim, visit [this page](https://github.com/1unarzDev/unity_asv_sim) and follow the instructions for the setup.

The ROS packages/nodes you run for navigation are all completely up to you depending on what needs to be tested; however, be sure to always use the `ros_tcp_endpoint` package by running `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p <arg>:=<value>` (`ROS_IP` and `ROS_TCP_PORT` are useful args for matching the connection with Unity).

Finally, in order to start the Ardupilot SITL, you must start the devcontainer. After it's running, in VSCode, open the activity bar. From there, select the "Remote Explorer" option. In the "Other Containers" section, should should be able to attach a VSCode window to `epsilon_docker_devcontainer (ardupilot_sitl)` (you may also just open it through a local terminal by running `docker run -it ardupilot_sitl bash`). After you have access to the terminal, run the following command below (please note it will fail initially unless the Unity connection is already up):

```bash
Tools/autotest/sim_vehicle.py -v "$VEHICLE" $SITL_EXTRA_ARGS
```

and connect it to ROS by starting the MAVROS node

```
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5763
```

All of these commands can also be found by running the `help` command in their respective containers.

To run the ROS navigation code, a launch file has been provided for your convenience. Run `ros2 launch mhseals_nav robot.launch.py` with the optional arguments `use_sime_time` (should equal false if running on the actual boat by using `use_sime_time:=false`), `ros_ip` for the Unity simulation IP address, and finally `ros_port` for the simulation port. There are other arguments available for configuring a Zed 2i camera among other things. More details can be found be looking in the `launch` folder.