# ROS 2 on the Pi

How the robot's Raspberry Pi 5 runs ROS 2 and the Pico bridge, and how the laptop talks to it. The Pi's first boot and power tests are in [bench-tests.md](bench-tests.md) (test 1).

Set up and checked on 10 November 2026: Ubuntu 24.04.5 on the Pi, ROS 2 Jazzy on both machines, and the laptop in WSL2.

## 1. ROS 2 Jazzy

Over SSH (`ssh msevestre@robot-pi.local` from PowerShell), following the [official install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-ros-base ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

`ros-base` leaves out RViz and Gazebo, which run on the laptop.

**Gotcha: missing update sources.** The Pi's image listed only `noble` in `/etc/apt/sources.list.d/ubuntu.sources`, without `noble-updates`. The install then failed on `liblz4-dev` and `libzstd-dev`, whose versions must match libraries the security updates had already upgraded. Fixed with:

```bash
sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' /etc/apt/sources.list.d/ubuntu.sources
sudo apt update
```

These are still 24.04 packages. Don't run `do-release-upgrade`, which the login banner offers: Jazzy doesn't support 26.04.

**Check:** `ros2 doctor --report | grep "distribution name"` shows `jazzy`.

## 2. The bridge

```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/Mathieu-py/autonomous-robot.git
cd ~/robot_ws
sudo rosdep init; rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select pico_bridge
colcon test --packages-select pico_bridge && colcon test-result --verbose | tail -1
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

The firmware folder is skipped; it's built on the laptop with the Pico SDK and flashed by copying `firmware/build/robot_firmware.uf2` onto the Pico in BOOTSEL mode.

**Check:** 7 tests, 0 failures.

## 3. The Pico's USB link

The Pico plugs into one of the Pi's USB-A ports. The Pi's USB-C port is power only.

```bash
sudo usermod -aG dialout $USER                # serial port access; log in again after
sudo systemctl disable --now ModemManager     # it probes new serial devices with modem commands
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0009", SYMLINK+="pico", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-pico.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The udev rule names the Pico `/dev/pico` whatever `ttyACM` number it gets: on a replug it came up as `ttyACM1`. `2e8a:0009` is the Pico SDK's USB serial on the RP2350. `/dev/pico` is the bridge's default port.

Run the bridge:

```bash
ros2 run pico_bridge pico_bridge_node
```

**Check:** it logs `Connected to the Pico on /dev/pico`; `ros2 topic hz /joint_states` shows 50 Hz; `/battery` reads the supply. Measured: 50.00 Hz with 0.02 ms jitter on the Pi, 11.93 V on a 12 V supply, and turning the first wheel by hand moved the left joint.

A low-battery stop when the bridge starts is expected if the motor supply is off: the Pico reads the battery through the divider, so it reads about 0 V until the supply comes on, and the stop clears once it's above 11.0 V.

## 4. Laptop networking

WSL2's default NAT mode hides it behind a virtual adapter, so ROS discovery can't reach the Pi. In Windows:

1. Put this in `%USERPROFILE%\.wslconfig`:
   ```
   [wsl2]
   networkingMode=mirrored
   ```
2. In PowerShell as Administrator, allow inbound traffic to WSL ([Microsoft's WSL networking docs](https://learn.microsoft.com/en-us/windows/wsl/networking)):
   ```
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```
3. Run `wsl --shutdown` (this also restarts Docker Desktop). WSL then has an address on the home network (`10.0.0.55`, with the Pi at `10.0.0.225`).

On both machines, the same ROS domain keeps the robot's topics apart from any other ROS on the network:

```bash
echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc
```

A node started before this keeps the old domain, so restart the bridge in a terminal where `echo $ROS_DOMAIN_ID` prints 17. ROS commands on the laptop run in Ubuntu (WSL), not PowerShell.

**Check:** a `ros2 topic pub` on either machine reaches a `ros2 topic echo /chatter std_msgs/msg/String` on the other; the laptop sees `/joint_states` at 50 Hz. Measured: 50.0 Hz, with a few milliseconds of WiFi jitter.

## 5. Driving from the laptop

With the bridge running on the Pi:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Checked 10 November 2026** with the wheels lifted on the 12 V bench supply: both wheels followed every key, and stopping the bridge mid-drive stopped them through the Pico's 250 ms command timeout.
