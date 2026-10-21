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
| `gnc_offboard/` | ROS 2 package: `frames.py` (field ENU ↔ PX4 NED, tested), `trajectory.py` (time-defined smoothstep cubic guidance, GNC Notes 5.5, tested) and `mission_demo` (take off → target die → start zone → land). Guidance defaults to `cubic`; `-p guidance:=carrot` gives the old constant-speed setpoint. See `docs/cubic_vs_carrot.png`. |
| `tools/plot_field.py` | Draws the field from the manifest; with `--waypoints` or `--path`, checks a path against inflated obstacles and marks violations. |
| `tools/plot_flight.py` | Plots a PX4 flight log (newest by default) on the field: path, altitude, speed, tracking error. Compare up to 3 logs. |
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

## Projected camera view and searched ground

### Quick start: run a mission with camera coverage

Run these commands in **Ubuntu/WSL terminals** after completing the installation
above. `gnc` loads ROS 2, the built workspace, the Python venv, and project paths
for the current terminal. RViz is not required.

1. **Build once after adding or updating the camera feature:**

   ```bash
   gnc
   deactivate 2>/dev/null
   cd ~/dev/ros2_ws
   colcon build --packages-select gnc_offboard --symlink-install
   gnc
   ```

2. **Start the simulator:**

   ```bash
   cd ~/dev/avc-gnc
   ./start_sim.sh
   ```

   Wait for PX4 to report `Ready for takeoff!`.

3. **Start the camera observer in a separate Ubuntu terminal:**

   ```bash
   gnc
   ros2 run gnc_offboard camera_fov
   ```

   Leave this running before and during the mission.

4. **Run the mission in the simulator's bottom-right dev pane:**

   ```bash
   ros2 run gnc_offboard mission_demo
   ```

   Gazebo displays **cyan = current camera footprint** and **orange = searched
   ground**. For a headless sim (`./start_sim.sh -H`), open its viewer from
   another terminal with `gz sim -g`.

5. **After landing, press Ctrl+C in the camera terminal** to save the coverage
   JSON and full-map PNG. See the plot-access commands below. When finished,
   stop the simulator from another terminal:

   ```bash
   ~/dev/avc-gnc/start_sim.sh stop
   ```

### Added camera files

Paths below are relative to the repository root.

| File | Purpose |
|---|---|
| `gnc_offboard/gnc_offboard/camera_model.py` | Camera geometry and **default camera configuration** in the `CameraModel` dataclass. Projects the view onto the ground and accumulates searched cells. Independent of ROS. |
| `gnc_offboard/gnc_offboard/camera_fov.py` | ROS 2 observer node. Reads PX4 odometry, applies the camera model, handles live parameters, publishes coverage, draws Gazebo markers, and saves results. Does not send flight commands. |
| `gnc_offboard/gnc_offboard/coverage_plot.py` | Creates the north-up, bird's-eye PNG showing the full field and searched ground. |
| `tools/plot_camera_coverage.py` | Command-line tool to regenerate a PNG from saved coverage JSON. |
| `gnc_offboard/test/test_camera_model.py` | Tests projection geometry, attitude and mount transforms, clipping, coverage accumulation, and parameter validation. |

`gnc_offboard/setup.py` registers the `camera_fov` command, and
`gnc_offboard/package.xml` declares its dependencies.

### Camera defaults and configuration before a run

There is no dedicated camera YAML config file yet. The default geometry settings
are in `camera_model.py`; startup/output defaults are in `camera_fov.py`.

| Parameter | Default | Meaning |
|---|---|---|
| `hfov_deg` | `70.0` | Horizontal field of view, degrees |
| `vfov_deg` | `50.0` | Vertical field of view, degrees |
| `pitch_down_deg` | `45.0` | Camera pitch downward relative to the vehicle |
| `forward_m` | `0.2` | Mount translation forward from vehicle origin, metres |
| `right_m` | `0.0` | Mount translation right, metres |
| `down_m` | `0.0` | Mount translation down, metres |
| `yaw_right_deg` | `0.0` | Camera yaw right relative to the vehicle |
| `roll_deg` | `0.0` | Camera roll, clockwise looking forward |
| `max_range_m` | `30.0` | Maximum horizontal projection range, metres |

Translations use vehicle body axes (forward, right, down). The model also applies
the vehicle's actual roll, pitch and yaw from PX4 odometry. Override defaults at
startup by replacing the camera command in quick-start step 3:

```bash
gnc
ros2 run gnc_offboard camera_fov --ros-args \
  -p hfov_deg:=80.0 -p vfov_deg:=60.0 \
  -p pitch_down_deg:=45.0 \
  -p forward_m:=0.2 -p right_m:=0.0 -p down_m:=0.0
```

Other parameters must be set at startup: `resolution_m` (0.25 m grid),
`ground_z_m` (0 m field ENU), `manifest` (field manifest path),
`output` (JSON output path; PNG uses the same stem),
`odometry_topic` (`/fmu/out/vehicle_odometry`), and `gazebo_markers` (true).
For example, choose a finer grid and preserve this run under a distinct name:

```bash
ros2 run gnc_offboard camera_fov --ros-args \
  -p resolution_m:=0.1 \
  -p output:="$HOME/dev/avc-gnc/plots/camera_run_01.json"
```

### Inspect and edit settings during a run

With `camera_fov` running, open another Ubuntu terminal. All geometry parameters
in the table above can be changed live:

```bash
gnc
ros2 param list /camera_fov
ros2 param get /camera_fov hfov_deg
ros2 param set /camera_fov hfov_deg 90.0
ros2 param set /camera_fov vfov_deg 60.0
ros2 param set /camera_fov pitch_down_deg 55.0
ros2 param set /camera_fov forward_m 0.3
ros2 param set /camera_fov right_m 0.1
ros2 param set /camera_fov down_m 0.0
ros2 param set /camera_fov yaw_right_deg 10.0
ros2 param set /camera_fov roll_deg 0.0
ros2 param set /camera_fov max_range_m 20.0
```

These changes affect subsequent observations; existing searched coverage is
retained. Live changes do not modify the source defaults or persist after a
restart. FOV values must be greater than 0 and less than 179 degrees. Restart the
node to change grid/output settings or to begin with an empty coverage grid.

### Save and open the camera plot

After landing, **Ctrl+C in the camera terminal** exports:

- `~/dev/avc-gnc/plots/camera_coverage.png`: full-field, north-up bird's-eye map
  with searched ground highlighted, a blue dotted drone trajectory, obstacles,
  targets, and the start zone. The header reports covered area and percentage
  of the full arena, and whether the target centre entered an observed sensor
  footprint. This is geometric coverage, not image-based target recognition.
- `~/dev/avc-gnc/plots/camera_coverage.json`: searched cell coordinates, grid
  resolution, approximate searched area, field manifest, recorded vehicle
  trajectory (`trajectory_enu`), and exact footprint-based `target_found` status.

Older JSON files do not contain a trajectory or exact target observation. Their
plots state that the path was not recorded and label target coverage as a grid
estimate. New runs record the actual observed vehicle path; data gaps appear as
breaks in the dotted line. Coverage uses the full rectangular arena area as the
denominator, including obstacle footprints; edge cells are clipped to the arena.

The PNG is generated from coverage data, independently of the Gazebo window and
its viewing angle. In WSL, open the plots folder in Windows Explorer and
double-click `camera_coverage.png`:

```bash
cd ~/dev/avc-gnc/plots
explorer.exe .
```

Each camera-node session starts with empty coverage. Multiple missions during
one session accumulate together. Files are overwritten when that session stops;
while the node is running, existing files still contain the previous saved
session. Use a distinct `output` filename to retain separate runs.

Regenerate the PNG from an existing JSON without rerunning the simulation:

```bash
gnc
cd ~/dev/avc-gnc
python tools/plot_camera_coverage.py

# Or select a saved run and an output PNG:
python tools/plot_camera_coverage.py plots/camera_run_01.json \
  -o plots/camera_run_01.png
```

### ROS outputs and model limits

`/camera/footprint` is a `geometry_msgs/PolygonStamped` in field ENU (`map`).
`/camera/searched` is a `nav_msgs/OccupancyGrid`: 100 means searched, 0 means
not yet searched, and -1 means outside the field. To inspect their rates:

```bash
gnc
ros2 topic hz /camera/footprint
# Ctrl+C, then:
ros2 topic hz /camera/searched
```

Coverage and area are approximate, based on grid-cell centres and received
odometry samples. The footprint intersects a flat ground plane, clips to field
bounds and range, and handles views crossing the horizon. It does not model
lens distortion, terrain or obstacle occlusion, or add an image sensor to Gazebo.
Stale odometry clears the current footprint after one second; historical coverage
remains visible.

## Plotting

Run these in the dev pane (`gnc` active). PNGs go to `plots/` (git-ignored); add `--show` to open a window.

```bash
cd ~/dev/avc-gnc
tools/plot_field.py                                    # the current world
tools/plot_field.py --waypoints "-6.1,6.1, -3,2.5"     # check a path (field x,y pairs)
tools/plot_field.py --path my_path.json --inflate 0.6  # JSON [[x,y],...] or CSV; exit code 1 if it hits anything
tools/plot_flight.py                                   # newest SITL flight log
tools/plot_flight.py cubic.ulg carrot.ulg --labels cubic,carrot   # side by side
```

The same pieces are importable for planner work: `gnc_offboard.field_map.FieldMap` (obstacles,
`clearance(x, y)`, `path_violations(path, inflate)`), `gnc_offboard.flight_log.load_ulog`, and
`gnc_offboard.plotting.draw_field / draw_path`. `inflate` defaults to 0.5 m (x500 radius ~0.35 m
plus 0.15 m margin). Flight logs live in `~/dev/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`;
copy ones worth keeping somewhere safe, since `make distclean` deletes them.

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
