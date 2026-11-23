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

**Check:** 8 tests, 0 failures.

## 3. The Pico's USB link

The Pico plugs into one of the Pi's USB-A ports. The Pi's USB-C port is power only.

```bash
sudo usermod -aG dialout $USER                # serial port access; log in again after
sudo systemctl disable --now ModemManager     # it probes new serial devices with modem commands
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0009", SYMLINK+="pico", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-pico.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The udev rule names the Pico `/dev/pico` whatever `ttyACM` number it gets: on a replug it came up as `ttyACM1`. `2e8a:0009` is the Pico SDK's USB serial on the RP2350. `/dev/pico` is the bridge's default port.

Run the bridge (once section 6 is done, it starts at boot instead):

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
3. Run `wsl --shutdown` (this also restarts Docker Desktop). WSL then has an address on the home network (`10.0.0.55`, with the Pi at `10.0.0.226`; the router can change both).

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

## 6. Low-battery power-off and starting at boot

At 10.2 V (3.4 V per cell) the Pico repeats `SHUTDOWN: battery critically low` until the bridge powers the Pi off. Otherwise the converter keeps the Pi running down to about 8 V, which damages the LiPo. The Pico doesn't ask when it reads no battery at all, so the Pi can still run on its own supply.

Let the bridge power off the Pi, and nothing else, without a password:

```bash
echo 'msevestre ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff' | sudo tee /etc/sudoers.d/robot-poweroff
sudo chmod 440 /etc/sudoers.d/robot-poweroff
sudo visudo -c
```

After a power-off the Pi still draws about 1.3 W, so unplug the battery.

**Never run `sudo rpi-eeprom-config --edit` on this Pi.** Ubuntu's tool rewrites the bootloader with the older image it ships (December 2025), and on 23 November that left this board unable to start its memory: 8 short blinks, repeating. The fix was Raspberry Pi's `rpi-boot-eeprom-recovery-<date>-2712-sd.zip` from the [rpi-eeprom releases](https://github.com/raspberrypi/rpi-eeprom/releases): copy `recovery.bin`, `pieeprom.bin` and `pieeprom.sig` from its image onto the card's boot partition, power on until the LED blinks fast, then take the three files off again. `sudo rpi-eeprom-update` with no options only ever installs a newer bootloader, so it's safe.

Start the bridge at boot, and again if it exits, as it does when the Pico is replugged or reboots. It waits up to 30 s for WiFi first: ROS only uses the networks that are up when a node starts.

```bash
sudo cp ~/robot_ws/src/autonomous-robot/ros/pico_bridge/systemd/pico-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pico-bridge
journalctl -u pico-bridge -f          # its log
```

To run the bridge by hand instead, `sudo systemctl stop pico-bridge` first: only one bridge can open the Pico's port.

**Check:**

1. `sudo -n /usr/bin/systemctl poweroff` powers the Pi off without asking for a password.
2. After a reboot, the laptop sees `/joint_states` at 50 Hz without anything started on the Pi. A Pi that has powered off shows a steady red LED: press its power button, or unplug the battery until the LED goes out.
3. End to end: with the battery unplugged, feed the XT60 from the bench supply and turn it down to 10.0 V. Within a few seconds the Pi powers off, and the log's last line reads `Pico: SHUTDOWN: battery critically low. Powering off the Pi`.

## 7. Reflashing the Pico from the Pi

The Pico's BOOTSEL button is under the upper deck, but the firmware reboots into BOOTSEL when its serial port is set to 1200 baud. Build on the laptop, then copy the firmware over from the repo folder in WSL (use the Pi's address, `10.0.0.226`, if `robot-pi.local` doesn't resolve):

```bash
scp firmware/build/robot_firmware.uf2 msevestre@robot-pi.local:~
```

On the Pi:

```bash
sudo systemctl stop pico-bridge
stty -F /dev/pico 1200                # the Pico reboots as a USB drive named RP2350
sleep 3
sudo mount -L RP2350 /mnt
sudo cp ~/robot_firmware.uf2 /mnt && sync
sudo umount -l /mnt                   # the Pico has already rebooted into the new firmware
sudo systemctl start pico-bridge
```
