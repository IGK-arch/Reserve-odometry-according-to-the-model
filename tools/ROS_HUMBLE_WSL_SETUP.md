# ROS 2 Humble on this Windows host

The project needs Ubuntu 22.04 (Jammy) for the official Humble deb packages.
**Status, 26 September 2026:** WSL 2 Ubuntu 22.04.5 is installed at
`E:\WSL\Ubuntu-22.04` with Linux user `admin123`. ROS 2 Humble, colcon and
rosbag2 with SQLite support are installed. A real
`colcon build --packages-up-to tram_odometry` completed successfully, and
short anchored and no-GNSS rosbag replays both passed the live topic checks.
The Linux build workspace is `/home/admin123/tram_hack_ros2_ws`.
Moving anchored replays passed for both vehicle IDs (`30618_01f73500` and
`30639_50956d6e`); their reports, plus the no-GNSS report, are saved under
`evaluation/ros_smoke/`. WSL may print `Failed to translate D:\...` for stale
Windows PATH entries; Linux commands, ROS and colcon still work.
The steps below reproduce this environment on another host; skip the Windows
installation steps on this machine.

## 1. WSL setup on a fresh Windows host

Open **PowerShell as Administrator** and run:

```powershell
wsl --install --no-distribution
```

**Reboot Windows manually** when this command completes. It does not reboot
Windows by itself. Then, in Administrator PowerShell:

```powershell
wsl --update --web-download
wsl --install -d Ubuntu-22.04 --location E:\WSL\Ubuntu-22.04 --web-download
```

If either command fails, keep its full output. Check `wsl --list --verbose`
before retrying so an already registered Ubuntu is not installed twice.

## 2. Finish Ubuntu setup after Windows has rebooted

Launch the new distribution if installation did not already launch it:

```powershell
wsl -d Ubuntu-22.04
```

Complete the first-launch Linux username/password prompt. On this host the
account is already `admin123`. From the Ubuntu shell run the installer when
rebuilding the environment:

```bash
bash /mnt/e/projects/mos_transportr_hack/tools/install_ros_humble_wsl.sh
```

The Ubuntu script checks for Ubuntu 22.04, installs the official `ros2-apt-source`,
updates Ubuntu, and installs `ros-humble-ros-base`, `ros-dev-tools`, colcon,
rosbag2, and the SQLite bag plugin. It then copies only `ros2_ws/src` into the
Ubuntu filesystem, runs `colcon build --packages-up-to tram_odometry`, checks
the executable and custom message type, and reads one bag's metadata. The
original data remains in the Windows project directory. Re-running the script
is safe; `apt` and `colcon` update what is already present.

## 3. Replay a bag

Open three Ubuntu shells. In each one:

```bash
source ~/tram_hack_ros2_ws/install/setup.bash
```

First shell:

```bash
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:=30618
```

Second shell:

```bash
ros2 bag play /mnt/e/projects/mos_transportr_hack/dataset/data/30618_2050d396 --clock
```

Third shell:

```bash
ros2 topic hz /result/velocity
ros2 topic echo /result/diagnostics
```

The node and bag player must both be running while `ros2 topic hz` measures
the output. For automated short replays on this machine:

```bash
bash /mnt/e/projects/mos_transportr_hack/tools/ros_smoke_wsl.sh anchored
bash /mnt/e/projects/mos_transportr_hack/tools/ros_smoke_wsl.sh no-gnss
```

Both modes passed on 26 September 2026 at approximately 29 Hz wall-clock
publication rate for velocity and position, with the expected output types,
frames and timestamp matches. The detailed topic and parameter contract is in
`ros2_ws/src/tram_odometry/README.md`.

## Sources

- [Microsoft WSL installation](https://learn.microsoft.com/en-us/windows/wsl/install)
- [Microsoft WSL command options, including `--location`](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
- [ROS 2 Humble Ubuntu installation (official humble branch)](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)
- [ROS 2 apt source setup (official humble branch)](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst)
