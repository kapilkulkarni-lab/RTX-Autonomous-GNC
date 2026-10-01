# AVC Mission Bird Dog – GNC Simulation

Simulation and offboard-control stack for the GNC team (path planning and drone control) for the 2026–27 Raytheon AVC. Everything runs on Ubuntu 24.04, either natively or in WSL2 on Windows. Stack: PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy, bridged by the Micro XRCE-DDS agent.

## What's in here

| File | What it does |
|---|---|
| `setup_gnc_env.sh` | One-time setup. Builds the XRCE agent, the Python venv, and the ROS 2 workspace (px4_msgs + gnc_offboard), and adds the `gnc` shortcut. Safe to re-run. |
| `start_sim.sh` | Starts Gazebo, PX4, the agent and a dev shell in one tmux window. `-H` runs without the Gazebo window. |
| `doctor.sh` | Health check. Run it while the sim is up and it reports what's working and what isn't: builds, processes, agent count, and whether ROS 2 can see PX4's topics. |
| `gnc_env.sh` | Loads ROS 2, the workspace and the venv in the right order. You normally just type `gnc`. |
| `generate_avc_world.py` | Builds a random AVC field (15×15 yd, start zone, dice, obstacles 5 ft apart) plus a JSON manifest of ground truth. |
| `gnc_offboard/` | ROS 2 package: `frames.py` (field ENU ↔ PX4 NED, tested) and `mission_demo` (take off → target die → start zone → land). |
| `requirements-gnc.txt` | Python packages for the venv (numpy pinned below 2 for ROS 2 Jazzy). |

## Install (once per computer)

All commands go in an **Ubuntu (WSL) terminal**, not PowerShell. The prompt should look like `you@PC:~$`.

**Step 0, Windows only:** install WSL with Ubuntu 24.04 by running `wsl --install -d Ubuntu-24.04` in PowerShell, then reboot and open "Ubuntu" from the Start menu.

**Step 1: PX4 and Gazebo** (about 30–60 min, mostly downloading)
```bash
mkdir -p ~/dev && cd ~/dev
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Close and reopen the terminal afterwards. Then run `cd ~/dev/PX4-Autopilot && make px4_sitl gz_x500` once to check it works, and type `shutdown` at the `pxh>` prompt to exit.

**Step 2: ROS 2 Jazzy**
```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

**Step 3: this repo**
```bash
cd ~/dev
git clone <THIS_REPO_URL> avc-gnc
cd avc-gnc
./setup_gnc_env.sh
```
The first run takes 10–20 min, most of it building px4_msgs. It ends by printing `OK: rclpy + px4_msgs import`. Open a new terminal when it's done.

## Daily use

```bash
~/dev/avc-gnc/start_sim.sh              # sim with the current world
~/dev/avc-gnc/start_sim.sh -H           # same, no Gazebo window (much lighter)
~/dev/avc-gnc/start_sim.sh -c 3 -s 12 -k ne   # new world: challenge 3, seed 12, NE start
~/dev/avc-gnc/start_sim.sh stop         # shut everything down
gz sim -g                               # open the Gazebo window on a headless sim
```
Once PX4's pane settles, fly the demo from the dev pane (bottom right):
```bash
ros2 run gnc_offboard mission_demo
ros2 run gnc_offboard mission_demo --ros-args -p altitude:=2.0 -p waypoints:="[0.0, 0.0, 3.0, -2.0]"
```
Quick manual test in the PX4 pane: `commander takeoff`, then `commander land`.

**tmux keys:** Ctrl+b then an arrow key switches panes, Ctrl+b z zooms a pane, Ctrl+b d detaches (`tmux attach -t avc` to return). For mouse support: `echo 'set -g mouse on' >> ~/.tmux.conf`.

**After a `git pull`:** if `gnc_offboard` changed, rebuild it (takes seconds):
```bash
deactivate 2>/dev/null; cd ~/dev/ros2_ws && colcon build --packages-select gnc_offboard; gnc
```

## Conventions (read before writing planner code)

- **Frames.** The field frame is ENU with the origin at the field center (+x east, +y north). PX4 local is NED with the origin at the spawn point. Convert only through `gnc_offboard/frames.py`, never by hand.
- **State input.** Use `/fmu/out/vehicle_odometry` or `vehicle_local_position` in the local frame. Never use lat/lon: GPS is banned at competition, and when perception's VIO replaces sim GPS, local-frame code keeps working unchanged.
- **Topic names.** Some PX4 topics carry version suffixes (`_v1`). `mission_demo` detects this automatically; copy its `resolve()` helper.
- **QoS.** PX4 topics need BEST_EFFORT + TRANSIENT_LOCAL, or subscribers silently get nothing. Use `PX4_QOS` from `mission_demo.py`.
- **Venv.** The venv is for your Python scripts and nodes. Deactivate it before `colcon build`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Anything not working | Run `./doctor.sh` while the sim is up, and start with the first `[FAIL]` line. |
| `The term '...' is not recognized` (PowerShell-style error) | You're in PowerShell. Type `wsl` or open the Ubuntu terminal. |
| `Unable to find or download file` from `gz sim` | Wrong path to the world. Use `start_sim.sh`, which handles it. |
| `mission_demo`: "No PX4 topics after 20 s" | `./start_sim.sh stop`, then start again. If it persists, run `wsl --shutdown` in PowerShell and relaunch. |
| PX4 hangs at "waiting for Gazebo world" | World name mismatch or Gazebo not up yet. Run `start_sim.sh stop`, then start again. |
| `Arming denied: manual control lost` | In the PX4 pane: `param set COM_RC_IN_MODE 4` |
| apt: `Network is unreachable` mid-install | Network dropped. Rerun with `sudo`. If it repeats: `echo 'Acquire::ForceIPv4 "true";' \| sudo tee /etc/apt/apt.conf.d/99force-ipv4` |
| colcon build `Killed` / `signal 9` | WSL ran out of RAM: `MAKEFLAGS="-j2" colcon build --executor sequential` |
| `ModuleNotFoundError: rclpy` in the venv | Rerun `./setup_gnc_env.sh`, which fixes the venv in place. |
| Everything sluggish | Check `glxinfo -B \| grep renderer`. If it shows `llvmpipe`, the GPU isn't used, so run with `-H`. |

## Open questions for our Raytheon mentor

1. May a UAV fly *over* obstacles, or must it avoid them in 2-D? The rules say obstacles are things "the UGV can not drive over" but also that all other objects "shall be avoided."
2. Real dimensions of the boxes, cones and buckets. `OBSTACLE_TYPES` in `generate_avc_world.py` uses estimates.
