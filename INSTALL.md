# Franka Research 3 (FR3) Low-Level ROS 2 Control Setup
## Ubuntu 24.04 + PREEMPT_RT + ROS 2 Jazzy + libfranka + franka_ros2

This guide documents a complete, working setup for low-level control of a Franka Research 3 (FR3), starting from a fresh Ubuntu 24.04 installation and ending with successful `libfranka` communication, ROS 2 hardware bringup, and the Franka gravity-compensation example controller.

It also includes the issues encountered during setup and the fixes that worked on the tested workstation, including:

- Ubuntu Pro requirement for the Canonical real-time kernel
- NVIDIA display failure after switching to PREEMPT_RT
- NVIDIA DKMS rebuild for the real-time kernel
- FCI/Control IP vs Arm IP confusion
- two-Ethernet-port configuration issues
- FCI activation in Desk
- real-time scheduling permission failure
- `libfranka` connection timeout troubleshooting
- missing `franka_fr3_moveit_config` package after an incomplete build

---

# 1. Tested setup

The tested workstation used:

```text
Ubuntu:                 24.04 LTS
Real-time kernel:       6.8.1-1061-realtime
ROS 2:                  Jazzy
Robot:                  Franka Research 3 (FR3)
Control / FCI IP:       192.16.0.1
Control-PC FCI IP:      192.16.0.2/24
Arm-side network:       192.168.0.1
NVIDIA driver used:     595-open
NVIDIA DKMS version:    595.91.07
```

Important: package and driver versions change. Use the current Franka Jazzy dependency manifest and the NVIDIA branch recommended by `ubuntu-drivers devices` unless reproducing this exact tested system.

---

# 2. Recommended topology

For FCI/libfranka/ROS 2 control, the PC communicates with **Franka Control**, not directly with the Arm LAN interface.

Recommended minimal topology:

```text
Control workstation
   Ethernet NIC
   192.16.0.2/24
        |
        | direct Ethernet
        |
Franka Control
   192.16.0.1
        |
        | internal robot connection
        |
FR3 arm + gripper
```

For low-level control, one Ethernet cable to the **Control / Shop Floor / LAN port on Franka Control** is sufficient.

A second cable to the Arm network is optional and is not required for FCI.

If two PC Ethernet cables are used, keep the two subnets on separate physical NICs:

```text
NIC A -> Franka Control
PC:       192.16.0.2/24
Control:  192.16.0.1

NIC B -> Arm-side network
PC:       192.168.0.133/24   # example from the tested machine
Arm:      192.168.0.x
```

Do not assign both `192.16.0.x` and `192.168.0.x` to the same NIC unless there is a specific reason to do so.

---

# 3. Install Ubuntu 24.04 LTS

Install Ubuntu 24.04 LTS Desktop, 64-bit.

After first boot:

```bash
sudo apt update
sudo apt full-upgrade -y

sudo apt install -y \
    build-essential \
    git \
    cmake \
    ninja-build \
    curl \
    wget \
    gnupg \
    locales \
    software-properties-common \
    ethtool \
    cpufrequtils \
    rt-tests

sudo reboot
```

Verify:

```bash
lsb_release -a
uname -m
```

Expected architecture:

```text
x86_64
```

---

# 4. Ubuntu Pro account and real-time kernel

Canonical's Ubuntu real-time kernel is provided through Ubuntu Pro.

You need:

1. an Ubuntu/Canonical account;
2. an Ubuntu Pro subscription/entitlement (personnal use is free);
3. Internet access while attaching the machine.

Ubuntu Pro has a free personal tier for eligible personal use.

Install/update the Pro client:

```bash
sudo apt update
sudo apt install -y ubuntu-pro-client
```

Check status:

```bash
pro status
```

Attach the workstation:

```bash
sudo pro attach
```

Follow the displayed browser/device-code instructions.

If you already have a token:

```bash
sudo pro attach YOUR_TOKEN
```

Then enable the real-time kernel:

```bash
sudo pro enable realtime-kernel
```

Reboot:

```bash
sudo reboot
```

Verify:

```bash
uname -r
cat /sys/kernel/realtime
```

Expected:

```text
<kernel-name>-realtime
1
```

On the tested workstation:

```text
6.8.1-1061-realtime
```

Keep at least one generic Ubuntu kernel installed as a recovery option in GRUB.

---

# 5. Potential failure after enabling PREEMPT_RT

## 5.1 Symptom observed

After booting the new real-time kernel, the workstation can loose its second display and the remaining display can fall back to 1024x768.

Diagnostics showed:

```bash
nvidia-smi
```

returning:

```text
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver.
```

and:

```bash
sudo modprobe nvidia
```

returning:

```text
modprobe: FATAL: Module nvidia not found in directory /lib/modules/6.8.1-1061-realtime
```

The problem is usually that the NVIDIA kernel module has not been built for the PREEMPT_RT kernel. In that case, check the following options for the GPU.

Run:

```bash
uname -r
lspci -k | grep -EA4 'VGA|3D|Display'
lsmod | grep -E 'nvidia|nouveau|amdgpu|i915'
sudo lshw -c display
nvidia-smi
ubuntu-drivers devices
```

Check Secure Boot:

```bash
mokutil --sb-state
```

If `mokutil` is missing:

```bash
sudo apt install -y mokutil
```

On the tested machine, the driver was not build, installing the open module fixed it:

```text
nvidia-driver-595-open - recommended
```

Use the driver branch recommended by your machine; do not assume `595-open` is always the correct branch.

To install the driver, first install the dkms. While booted into the real-time kernel:

```bash
sudo apt update

sudo apt install -y \
    dkms \
    build-essential \
    linux-headers-$(uname -r)
```

Verify:

```bash
ls -ld /lib/modules/$(uname -r)/build
```

Then rebuild/install the NVIDIA driver for PREEMPT_RT. For the tested system:

```bash
sudo env IGNORE_PREEMPT_RT_PRESENCE=1 \
    apt install -y nvidia-dkms-595-open nvidia-driver-595-open
```

If Ubuntu recommends a different branch, substitute that branch.

Check DKMS:

```bash
dkms status
```

The tested system reported:

```text
nvidia/595.91.07, 6.8.1-1061-realtime, x86_64: installed
```

A warning for another generic kernel may be harmless if the real-time kernel line itself says `installed`.

Then:

```bash
sudo depmod -a

find /lib/modules/$(uname -r) \
    -type f \
    -iname 'nvidia*.ko*'

modinfo nvidia | head -20
sudo modprobe nvidia
nvidia-smi
```

Reboot:

```bash
sudo reboot
```

Verify:

```bash
uname -r
nvidia-smi
lsmod | grep nvidia
lspci -k | grep -EA4 'VGA|3D|Display'
```

The NVIDIA card should show:

```text
Kernel driver in use: nvidia
```

The dual-screen configuration and normal resolution returned after this fix on the tested workstation.

If the DKMS module exists but `modprobe nvidia` reports:

```text
Key was rejected by service
```

check:

```bash
mokutil --sb-state
sudo dmesg | grep -iE 'nvidia|NVRM|secure|module|verification' | tail -100
```

The module may need MOK enrollment/signing.

## 5.6 Alternative: use the AMD/iGPU for displays

The tested workstation also had an AMD integrated GPU using `amdgpu`, which worked normally under PREEMPT_RT.

For a dedicated robot-control workstation, using the integrated GPU for displays can be simpler than maintaining NVIDIA + PREEMPT_RT.

---

# 6. Real-time user permissions

The real-time kernel alone is not enough. The ROS 2 controller manager and libfranka must also be allowed to use FIFO real-time scheduling.

Create the group and add the current user:

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
```

Create:

```bash
sudo nano /etc/security/limits.d/99-realtime.conf
```

with:

```text
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

Reboot or fully log out and log back in:

```bash
sudo reboot
```

Verify:

```bash
id
ulimit -r
ulimit -l
```

Expected:

```text
realtime   # appears in the user's groups
99         # ulimit -r
unlimited  # ulimit -l, ideally
```

Direct test:

```bash
chrt -f 80 sleep 0.1
```

A successful test exits silently.

A failed setup produces errors such as:

```text
Could not enable FIFO RT scheduling policy: Operation not permitted
```

or:

```text
libfranka: unable to set realtime scheduling: Operation not permitted
```

The tested system was considered fixed when ROS printed:

```text
Successful set up FIFO RT scheduling policy with priority 97.
```

---

# 7. CPU performance mode

Install:

```bash
sudo apt install -y cpufrequtils
```

Configure:

```bash
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl enable cpufrequtils
sudo systemctl restart cpufrequtils
```

Check:

```bash
grep . /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

Prefer `performance` on CPUs used for control.

Modern `intel_pstate`/`amd_pstate` systems may expose slightly different behavior, so verify the effective governor.

---

# 8. Optional real-time latency test

Run:

```bash
sudo cyclictest \
    --mlockall \
    --smp \
    --priority=80 \
    --interval=200 \
    --distance=0 \
    --loops=100000
```

Investigate unexplained millisecond-scale maximum latency spikes before running aggressive low-level controllers.

---

# 9. Configure the Franka Control network

On the tested robot:

```text
Franka Control / FCI IP:  192.16.0.1
PC FCI interface:         192.16.0.2/24
```

The ROS parameter named `robot_ip` must be set to the **Control / FCI IP**, not to the Arm-side IP.

For this setup:

```bash
export FRANKA_IP=192.16.0.1
```

## 9.1 Identify PC Ethernet interfaces

```bash
ip -br link
ip -br addr
```

To check physical link state:

```bash
sudo ethtool enp12s0 | grep -i 'Link detected'
sudo ethtool enp11s0 | grep -i 'Link detected'
```

If both ports have cables, the easiest way to identify the Control-facing NIC is to unplug only the cable at Franka Control and run:

```bash
watch -n 0.5 'ip -br link'
```

The interface that loses `LOWER_UP` is the Control-facing NIC.

## 9.2 Example static FCI configuration

Assume `enp12s0` is the interface connected to Franka Control.

A NetworkManager profile can be created with:

```bash
sudo nmcli connection add \
    type ethernet \
    ifname enp12s0 \
    con-name franka-fci \
    ipv4.method manual \
    ipv4.addresses 192.16.0.2/24 \
    ipv4.never-default yes \
    ipv6.method disabled
```

Activate:

```bash
sudo nmcli connection up franka-fci
```

Check:

```bash
ip -br addr show enp12s0
ip route get 192.16.0.1
```

Expected route:

```text
192.16.0.1 dev enp12s0 src 192.16.0.2
```



# 10. Basic Control network tests

First:

```bash
ping -c 100 192.16.0.1
```

A healthy link should have zero packet loss.

Check routing:

```bash
ip route get 192.16.0.1
```

Check ARP if ping fails:

```bash
sudo apt install -y arping
sudo arping -I enp12s0 192.16.0.1
ip neigh show dev enp12s0
```

If `arping` gets no reply, the problem is below ROS/libfranka. Check:

- wrong NIC;
- wrong cable;
- wrong Control IP;
- cable connected to Arm instead of Control;
- bad cable/link;
- incorrect static address.

Check link quality:

```bash
sudo ethtool enp12s0 | grep -E 'Speed|Duplex|Link detected'
```

Prefer:

```text
Speed: 1000Mb/s
Duplex: Full
Link detected: yes
```

High-rate test:

```bash
sudo ping 192.16.0.1 \
    -i 0.001 \
    -D \
    -c 10000 \
    -s 1200
```

The tested link produced sub-millisecond RTT and ultimately achieved reliable libfranka communication.

---

# 11. FCI feature and FCI activation in Desk

The robot must have the Franka Control Interface feature installed.

In Desk, verify the installed features and make sure FCI is available.

On recent robot software versions, FCI must also be explicitly activated in Desk before a libfranka/ROS connection is accepted.

If it is not active, errors look like:

```text
libfranka: Connection to FCI refused.
Please install FCI feature or enable FCI mode in Desk.
```

Activate FCI in Desk, then retry.

Once FCI is active, Desk/Apps should not simultaneously control the robot.

---

# 12. Install ROS 2 Jazzy

Configure locale:

```bash
sudo apt update
sudo apt install -y locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

Enable Universe:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Install the ROS repository configuration:

```bash
sudo apt update
sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(
    curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F 'tag_name' \
    | awk -F\" '{print $4}'
)

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
```

Install ROS:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Source it:

```bash
source /opt/ros/jazzy/setup.bash
```

Persist:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

Verify:

```bash
ros2 --help
```

---

# 13. Create the Franka ROS 2 workspace

Create the workspace:

```bash
mkdir -p ~/franka_ros2_ws
cd ~/franka_ros2_ws
```

Clone the Jazzy branch directly into `src`:

```bash
git clone \
    -b jazzy \
    https://github.com/frankarobotics/franka_ros2.git \
    src
```

Import Franka's pinned dependencies:

```bash
vcs import src \
    < src/dependency.repos \
    --recursive \
    --skip-existing
```

This is preferable to independently installing arbitrary versions of `libfranka`, `franka_description`, or `ros2_control`.

---

# 14. Install Franka/ROS dependencies

Initialize rosdep if necessary:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Install dependencies:

```bash
cd ~/franka_ros2_ws

rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -y \
    --skip-keys=zed_wrapper
```

Install MoveIt explicitly if needed:

```bash
sudo apt install -y ros-jazzy-moveit
```

---

# 15. Build the Franka workspace

Always use a Release build for the real robot:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/franka_ros2_ws

colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF
```

Source it:

```bash
source ~/franka_ros2_ws/install/setup.bash
```

Persist:

```bash
echo 'source ~/franka_ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

# 16. Missing moveit package issue

A build that only partially succeeds can lead to:

```text
Package 'franka_fr3_moveit_config' not found
```

Check whether the source package exists:

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash

colcon list | grep -E 'franka_(fr3_moveit_config|bringup|example_controllers|robot_state_broadcaster)'
```

If the source exists, reinstall dependencies and build up to the MoveIt package:

```bash
sudo apt install -y ros-jazzy-moveit

rosdep update
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -y \
    --skip-keys=zed_wrapper

colcon build \
    --symlink-install \
    --packages-up-to franka_fr3_moveit_config \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    --event-handlers console_direct+
```

Then:

```bash
source ~/franka_ros2_ws/install/setup.bash
ros2 pkg prefix franka_fr3_moveit_config
```

---

# 17. Test the stack with fake hardware

Once the package exists:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
    robot_ip:=dont-care \
    use_fake_hardware:=true
```

RViz/MoveIt should start with the FR3 model.

This verifies the ROS software side before involving the real robot.

---

# 18. Build libfranka examples separately

Find the imported libfranka source:

```bash
find ~/franka_ros2_ws/src \
    -type f \
    -path '*/examples/communication_test.cpp' \
    -print
```

Assuming the source is:

```text
~/franka_ros2_ws/src/libfranka
```

build examples:

```bash
cmake \
    -S ~/franka_ros2_ws/src/libfranka \
    -B ~/libfranka-fci-build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=ON

cmake --build ~/libfranka-fci-build -j"$(nproc)"
```

Check:

```bash
ls ~/libfranka-fci-build/examples/
```

Useful binaries include:

```text
communication_test
echo_robot_state
```

---

# 19. Direct libfranka communication test

Set:

```bash
export FRANKA_IP=192.16.0.1
```

Make sure:

- FCI is activated in Desk;
- the robot workspace is clear;
- the user stop is immediately accessible;
- the robot is in an operational state.

Run:

```bash
~/libfranka-fci-build/examples/communication_test "$FRANKA_IP"
```

Important: `communication_test` **moves the robot** to an initial joint configuration before testing communication.

The tested system produced:

```text
Finished moving to initial joint configuration.

Starting communication test.
#100 Current success rate: 1.00
#200 Current success rate: 1.00
#300 Current success rate: 1.00
...
#900 Current success rate: 1.00
```

This validates the basic FCI/control path.

If the user stop is pressed during the test, an expected result is:

```text
libfranka: Move command aborted: User Stop pressed!
```

That is not a communication failure.

A passive state-read test can also be run with:

```bash
~/libfranka-fci-build/examples/echo_robot_state "$FRANKA_IP"
```

---

# 20. Meaning of common libfranka network errors

## 20.1 `Connection timeout`

Example:

```text
libfranka: Connection timeout. Please check your network connection or settings.
```

Possible causes:

- wrong Control/FCI IP;
- using the Arm IP instead of Control IP;
- wrong NIC;
- cable connected to the Arm LAN port instead of Control;
- FCI not active;
- firewall/filtering;
- Control not reachable at layer 2/3;
- unstable network.

Check:

```bash
ping -c 100 192.16.0.1
ip route get 192.16.0.1
sudo arping -I <control-interface> 192.16.0.1
sudo ethtool <control-interface> | grep -E 'Speed|Duplex|Link detected'
sudo ufw status
```

## 20.2 `Connection to FCI refused`

Example:

```text
libfranka: Connection to FCI refused.
Please install FCI feature or enable FCI mode in Desk.
```

This means the network path is working well enough to reach Control, but FCI itself is not enabled/available.

Enable FCI in Desk and verify the feature is installed.

## 20.3 `unable to set realtime scheduling`

Example:

```text
libfranka: unable to set realtime scheduling: Operation not permitted
```

This is a Linux permission issue, not a network issue.

Fix the `realtime` group and limits in Section 6.

---

# 21. Bring up the real FR3 with ROS 2

After the direct libfranka test succeeds:

```bash
export FRANKA_IP=192.16.0.1

source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

ros2 launch franka_bringup franka.launch.py \
    robot_type:=fr3 \
    robot_ip:="$FRANKA_IP" \
    load_gripper:=true
```

If no Franka gripper is installed:

```text
load_gripper:=false
```

A successful launch should include:

```text
update rate is 1000 Hz
Spawning controller_manager RT thread with scheduler priority: 97
Successful set up FIFO RT scheduling policy with priority 97.
Connecting to robot at "192.16.0.1" ...
```

and should not contain:

```text
Connection timeout
Connection to FCI refused
unable to set realtime scheduling
```

---

# 22. Verify ROS 2 hardware state

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Check state topics:

```bash
ros2 topic list | grep -E 'joint_states|franka'
```

Then, for example:

```bash
ros2 topic echo /joint_states --once
```

At this point the chain is:

```text
ROS 2
  -> ros2_control
  -> franka_hardware
  -> libfranka
  -> FCI
  -> Franka Control
  -> FR3
```

---

# 23. Gravity-compensation example

This was the next tested step after successful ROS bringup.

Keep the normal Franka bringup running in terminal 1.

Make sure:

- the workspace is clear;
- the robot is in a safe configuration;
- the user stop is immediately accessible;
- FCI remains active.

In terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

ros2 control load_controller \
    --set-state active \
    gravity_compensation_example_controller
```

Check:

```bash
ros2 control list_controllers
```

Expected controller state includes approximately:

```text
gravity_compensation_example_controller   active
joint_state_broadcaster                    active
franka_robot_state_broadcaster             active
```

The arm should become compliant under gravity compensation.

To deactivate:

```bash
ros2 control set_controller_state \
    gravity_compensation_example_controller inactive
```

To reactivate:

```bash
ros2 control set_controller_state \
    gravity_compensation_example_controller active
```

An alternative one-command example launcher is:

```bash
ros2 launch franka_bringup example.launch.py \
    controller_names:=gravity_compensation_example_controller
```

For debugging, the two-step approach is preferable:

```text
1. start franka.launch.py
2. verify state/hardware
3. load gravity_compensation_example_controller manually
```

---

# 24. Recommended architecture for custom low-level controllers

Do not use a normal ROS publisher as the timing source of a 1 kHz torque loop.

Use a `ros2_control` controller plugin:

```text
FCI state
   |
   v
franka_hardware::read()
   |
   v
controller_manager @ 1000 Hz
   |
   v
custom ControllerInterface::update()
   |
   v
joint command interfaces
   |
   v
franka_hardware::write()
   |
   v
libfranka / FCI
   |
   v
FR3
```

Use ROS topics/services/actions for slower supervisory commands and desired references.

Pass references into the real-time loop using real-time-safe mechanisms such as:

```text
realtime_tools::RealtimeBuffer
```

---

# 25. Final known-good state from the tested setup

The workstation was considered ready when all of the following were true:

```text
Ubuntu 24.04                     OK
PREEMPT_RT kernel                OK
/sys/kernel/realtime = 1         OK
NVIDIA driver under RT kernel    OK
Dual displays restored           OK
realtime group/rtprio            OK
SCHED_FIFO priority 97           OK
Control IP reachable             OK
FCI active                       OK
libfranka communication_test     success rate 1.00
ROS 2 hardware bringup           OK
gravity compensation controller  OK
```

Key tested network configuration:

```text
Control / FCI IP:  192.16.0.1
PC FCI IP:         192.16.0.2/24
```

Key tested NVIDIA DKMS state:

```text
nvidia/595.91.07, 6.8.1-1061-realtime, x86_64: installed
```

Key successful RT message:

```text
Successful set up FIFO RT scheduling policy with priority 97.
```

Key successful FCI test:

```text
#100 Current success rate: 1.00
#200 Current success rate: 1.00
...
```

---

# 26. Official references

Ubuntu Pro:

```text
https://ubuntu.com/pro
https://ubuntu.com/pro/docs/attach-tutorial/
```

Ubuntu real-time kernel:

```text
https://ubuntu.com/real-time
https://ubuntu.com/real-time/docs/
```

Ubuntu NVIDIA driver documentation:

```text
https://ubuntu.com/server/docs/nvidia-drivers-installation/
```

ROS 2 Jazzy:

```text
https://docs.ros.org/en/jazzy/
```

Franka documentation:

```text
https://frankarobotics.github.io/docs/
```

Franka ROS 2:

```text
https://github.com/frankarobotics/franka_ros2
```

libfranka:

```text
https://github.com/frankarobotics/libfranka
```

---

# 30. Important operational notes

1. Keep a generic Ubuntu kernel installed as a recovery option.
2. After future kernel upgrades, recheck `dkms status` if using NVIDIA.
3. Do not independently upgrade `libfranka` without checking robot-system compatibility.
4. For FCI, use the **Control IP**, not the Arm IP.
5. FCI must be activated in Desk on current robot software.
6. Use Release builds for the real robot.
7. Keep the hard 1 kHz control loop inside `ros2_control`.
8. `communication_test` moves the robot.
9. Pressing User Stop during a motion example will intentionally abort the motion command.
10. Always clear the robot workspace and verify safety equipment before enabling low-level torque controllers.
