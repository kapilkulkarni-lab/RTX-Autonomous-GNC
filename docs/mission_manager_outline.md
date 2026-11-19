# mission\_manager SMACH Build Outline (for Claude Code)

Nov 19, 2026 · @Kapil

## Purpose

Build `mission_manager`, the GNC-hosted mission state machine for the RTX AVC drone, as one rclpy node using SMACH. This outline is the build brief for Claude Code. It condenses the "Mission State Machine v2 (GNC-hosted)" spec (Oct 23), the CDR architecture slides, and earlier design reviews.

Rules for the builder:

- Treat the States, Transitions and Interfaces sections as the spec. Where this doc and v2 disagree, this doc wins; where both are silent, stop and ask.
- Every TBD number goes in `config/mission.yaml` as a parameter, never as a literal in code.
- Work in the build order at the end, one milestone per PR, with tests passing before moving on.
- Never edit files in `avc_msgs`. Comms owns them; write any field change you need as a note for Abhi.
- Use American spelling in code, comments and docs (color, behavior, center, meter).

## Context and constraints

| Item | Value |
| --- | --- |
| Repo | `kapilkulkarni-lab/RTX-Autonomous-GNC` (packages `avc_interfaces`, `gnc_offboard`) |
| Comms messages | `avc_msgs` from `avc-comms-ws`, branch `parser-bridge-quadrant-masks`, pinned to commit `375598d`. Cloned into the `gnc` account and linked into `~/dev/ros2_ws/src`; built before `gnc_offboard` |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy, colcon, Python venv from `requirements-gnc.txt` |
| SMACH | `ros-jazzy-smach` 3.0.3 (the ROS 2 port of `executive_smach`); `smach_ros` optional, for the introspection viewer |
| PX4 link | uXRCE-DDS, `/fmu/in/*` and `/fmu/out/*`, `px4_msgs` on `release/1.16` |
| Target computer | Jetson Orin Nano, `gnc` account, `ROS_DOMAIN_ID=10` |
| Position source | Qualisys mocap into PX4 EKF2 in the lab; Perception VIO later. GNC reads the fused estimate from PX4 either way |
| Dice positions | Perception owns the dice map (xyz centroid and color of every die seen). GNC reads it directly and keeps no die memory of its own |
| Downward camera | One camera pointing straight down. BrainChip confirms die color from it, and it drives the final centering during descent |
| Main entry point | `mission_manager` is the one script that flies the mission. `mission_demo.py` is retired: lift its arming, offboard handshake and setpoint code into `guidance/px4_interface.py`, then remove `mission_demo` from `setup.py` and delete the file (git history keeps it). Nothing new may import or launch it |

House conventions:

- Mission logic and planning stay ROS-free and unit-testable; ROS nodes are thin wrappers.
- Subscription callbacks only store the latest message with a receive timestamp. No decisions in callbacks.
- Heartbeats and inputs are judged by age (timestamp staleness), never by boolean "received" flags.
- Store whole message objects, not copied fields.
- All QoS profiles live in one `qos.py` module (see Interfaces).
- Safety defaults: the hardware kill switch is outside the state machine. Nothing in this node may block a pilot takeover.

## Architecture

One node, `mission_manager`, owns the mission state. A SMACH `StateMachine` runs in a worker thread; the ROS node spins in the main thread and handles all I/O. They share one `MissionContext` object.

1. **Node (ROS side).** Subscriptions store the latest message and receive time into `MissionContext`. A 20 Hz timer streams the current setpoint to PX4 so offboard never drops, whatever SMACH is doing, including while landed on the die. A 5 Hz timer publishes `/mission/state`; a state change or a handled intent publishes immediately too.
2. **MissionContext (shared, lock-protected).** Latest inputs (including Perception's dice map), active target color, active regions with their until rules, mission clock, retry counters (descent, NFC per motor stage), motor stage on the die, current setpoint, where to go after LIFTOFF, `paused_from`, the stored intent received while paused, last handled `intent_id` and its result, `read_id` counter, and NFC results awaiting a comms verdict.
3. **SMACH states (ROS-free).** Each state subclasses a common `MissionStateBase(smach.State)`. Its `execute()` loops at 20 Hz: take a snapshot of the context, run the global checks, run the state's own checks, write any actions, and return an outcome when a transition fires. States never call rclpy directly; they write to the context or call an injected `MissionIO` interface.
4. **MissionIO interface.** Methods such as `set_setpoint()`, `arm()`, `disarm()`, `set_offboard()`, `land()`, `nfc_command()`, `gripper_command()`, `publish_nfc_result()`. The node implements it with real publishers; tests use a fake that records calls.
5. **State names.** Each state is added to SMACH under comms' constant (`MissionState.STATE_SEARCHING` and so on), so SMACH's active state label is exactly what goes on `/mission/state`. A SMACH transition callback (`register_transition_cb`) records `previous_state` and `reason` and triggers an immediate publish.

Global transitions live in `MissionStateBase`, checked first on every loop, so no state can forget them. "On the die" means NFC\_READY, READING or RETRIEVING.

| Check (in priority order) | Outcome | Applies in |
| --- | --- | --- |
| PX4 failsafe, offboard lost, or pilot takeover | `aborted` (stop sending commands) | Any state |
| Intent `abort` (acts on the action alone; other fields may be empty) | `aborted`. On the die: disarm and stay | Any state, including PAUSED |
| Position estimate invalid for TBD s, or Perception heartbeat stale for TBD s | `aborted` (hold, then land in place) | Airborne states |
| Battery below TBD % | `return` (on the die: LIFTOFF first) | Airborne and on-die states |
| Mission clock past the force-return threshold | `return` (on the die: LIFTOFF first) | All states before RETURNING |
| Intent `return_to_start` | `return` (on the die: LIFTOFF first) | Airborne and on-die states, including PAUSED |
| Intent `pause` (acts on the action alone) | `pause` | Every state except RETURN\_CONFIRMED and ABORTED |
| Intent `retrieve` with a new color | `retarget_seen` or `retarget_unseen`; drop the die first if holding one; on the die, LIFTOFF first. In PAUSED: stored, stays paused | TAKEOFF through RETURNING |

Why a worker thread: SMACH's `execute()` blocks until it returns an outcome. Running it inside a timer callback would freeze the executor, and the PX4 stream with it. Use a `MultiThreadedExecutor` or run `sm.execute()` in a `threading.Thread`, and stop it cleanly on shutdown with `request_preempt()`.

## States

Fifteen states: the twelve from v2 plus DESCENDING, LIFTOFF and PAUSED. The drone lands on the die before the NFC read and the grab, to cut rotor disturbance and put the reader as close to the tag as possible. Each state publishes under comms' `STATE_*` string with `previous_state` and `reason` on `/mission/state`.

| # | State | On entry | Each loop | Outcomes |
| --- | --- | --- | --- | --- |
| 1 | STANDBY | Disarmed, no setpoints | Wait for the first `retrieve` intent with a target color | `start` |
| 2 | PREFLIGHT | Start the setpoint stream (hold current pose) | Wait for EKF converged and stable for TBD s; record start position in the field frame; arm; switch to offboard | `ready`, `failed` |
| 3 | TAKEOFF | Setpoint = start position at search altitude | Check altitude within tolerance | `at_altitude` |
| 4 | SEARCHING | **Target check first:** if Perception's dice map already holds the active color at a position that fits the current intent (inside any target quadrant, outside avoid regions), leave at once. Otherwise fly the search pattern, skipping ground already covered, and any target quadrant first | Follow pattern; mark covered cells; leave when the dice map reports the active color | `target_known`, `target_found` |
| 5 | APPROACHING | Plan an obstacle- and region-aware path to the die's centroid from the dice map | Fly to hover height (TBD) over the centroid; run the color check from the downward camera | `over_target` (centered, color right), `wrong_color_known` → APPROACHING to the target, `wrong_color_unknown` / `nothing` / `lost` → SEARCHING |
| 6 | DESCENDING | Lock the touchdown point to the centroid's x/y | Descend in stages at TBD m/s; at each stage re-center on Perception's target-relative position and only continue while the sideways error is under that stage's tolerance; re-check color while the die is in view; below the camera blind height finish on the last good centering plus rangefinder and EKF; confirm touchdown from the rangefinder, not only PX4's land detector | `touchdown`, `retry` (error too large or die lost: climb to hover height and try again), `failed` (retries used) → APPROACHING |
| 7 | NFC\_READY | On the die. Motor stage 1: stay armed at idle (minimum throttle). Stage 2: disarm. Send `/hw/nfc/command` start | Wait for NFC status `reading` | `read_started`, `timeout` |
| 8 | READING | Stay landed in the current motor stage | Wait for `/hw/nfc/result`; publish it on `/mission/nfc_result` with a new `read_id`; then wait for comms' verdict on that `read_id`. No timeout: wait until comms says success or failed (global checks still apply) | `success_retrieve`, `success_return`, `failed_retry` (same stage), `failed_motors_off` (stage 1 used up → disarm, NFC\_READY), `failed_redescend` (stage 2 used up, first time → LIFTOFF then DESCENDING), `failed_hold` |
| 9 | RETRIEVING | Re-run the color check if the camera still sees the die; send gripper close | Wait for gripper `holding` | `holding`, `failed` (both → LIFTOFF, then RETURNING) |
| 10 | LIFTOFF | Re-arm and re-enter offboard if disarmed | Climb straight off the die to hover height | `to_descending`, `to_searching`, `to_approaching`, `to_returning` (chosen from the context) |
| 11 | RETURNING | Plan path to start zone center | Follow path | `over_start` |
| 12 | LANDING | Command land inside the start zone | Wait for landed and disarmed | `landed` |
| 13 | RETURN\_CONFIRMED | Stop commands | Terminal | `mission_complete` |
| 14 | ABORTED | Depends on cause: stop commanding if PX4 or the pilot has control; on the die, disarm and stay; otherwise hold, then land in place | Terminal | `mission_aborted` |
| 15 | PAUSED | Save `paused_from`; set `previous_state` to it. Ground states: stop arming, disarm if armed in PREFLIGHT. Airborne: hover in place (DESCENDING and LANDING stop descending and hover). On the die: keep the current motor stage; cancel a running read | Hold; store any new intent; mission clock keeps running; abort, return, battery and pose-loss checks still apply | `resume_new_intent`, or one `resume_<state>` per pausable state |

Notes:

- **Two-stage NFC read.** Stage 1, motors at idle: up to TBD reads. Stage 2, motors fully off: up to TBD reads. If both fail, LIFTOFF, re-center and DESCEND once more, then repeat both stages. If that fails too, hold landed and report on `/mission/state` until the leader sends a command. The motor stage goes in `reason`, for example "read failed at idle, retrying motors off".
- **Comms decides every read.** Every result, success or failure, goes to comms on `/mission/nfc_result` and is republished each heartbeat until comms answers. GNC moves on only on comms' verdict for that `read_id`. A late verdict for an earlier `read_id` still counts.
- **PX4 auto-disarm.** PX4 disarms on its own a few seconds after landing (`COM_DISARM_LAND`), which would end motor stage 1 early. GNC must own disarming on the die; flag the parameter for the GNC lead to set.
- `success_retrieve` requires retrieval enabled, enough time left (TBD) and the color check still right; otherwise `success_return` (LIFTOFF, then RETURNING).
- Time thresholds: past the first, RETRIEVING is skipped; past the second, the global check forces a return.
- **Resume rule.** If a new intent (any action other than `pause` or `resume`) arrived while paused, apply it and resume to SEARCHING, through LIFTOFF first if on the die. Otherwise resume to `paused_from`. Extra color or region fields on a `resume` are ignored. SMACH transitions are fixed at build time, so PAUSED returns a separate outcome for each state it can go back to.

## Transitions

Wire these as the SMACH `transitions={...}` map for each state. Global outcomes (`aborted`, `return`, `pause`, `retarget_seen`, `retarget_unseen`) are added to every state they apply to.

| From | Outcome | Trigger and guard | To |
| --- | --- | --- | --- |
| STANDBY | `start` | Comms `retrieve` with a target color | PREFLIGHT |
| PREFLIGHT | `ready` | EKF stable TBD s, start recorded, armed in offboard | TAKEOFF |
| PREFLIGHT | `failed` | Arming or offboard refused after TBD s | ABORTED |
| TAKEOFF | `at_altitude` | Altitude within tolerance | SEARCHING |
| SEARCHING | `target_known` | Entry check: active color already in the dice map, inside any target quadrant, outside avoid regions | APPROACHING |
| SEARCHING | `target_found` | Dice map reports the active color | APPROACHING |
| APPROACHING | `over_target` | At hover height over the centroid, centered, color check right | DESCENDING |
| APPROACHING | `wrong_color_known` | Color check wrong; the target color is elsewhere in the dice map | APPROACHING (new goal) |
| APPROACHING | `wrong_color_unknown`, `nothing`, `lost` | Color check wrong with target not in the map, low confidence, or no detection for TBD s | SEARCHING |
| DESCENDING | `touchdown` | Touchdown confirmed over the centroid | NFC\_READY (motor stage 1) |
| DESCENDING | `retry` | Sideways error over tolerance or die lost; retries left | DESCENDING (after climbing to hover height) |
| DESCENDING | `failed` | Descent retries used | APPROACHING |
| NFC\_READY | `read_started` | NFC status `reading` | READING |
| NFC\_READY | `timeout` | No `reading` status in TBD s | Counts as a failed read |
| READING | `success_retrieve` | Comms verdict success; retrieval enabled, time left, color still right | RETRIEVING |
| READING | `success_return` | Comms verdict success, otherwise | LIFTOFF → RETURNING |
| READING | `failed_retry` | Comms verdict failed; tries left in this motor stage | NFC\_READY (same stage) |
| READING | `failed_motors_off` | Comms verdict failed; stage 1 used up | NFC\_READY (disarm, stage 2) |
| READING | `failed_redescend` | Comms verdict failed; stage 2 used up, first time | LIFTOFF → DESCENDING |
| READING | `failed_hold` | Comms verdict failed; second full cycle used up | Hold landed in READING, wait for intent |
| RETRIEVING | `holding`, `failed` | Gripper holding; or fault, failure, time short | LIFTOFF → RETURNING |
| LIFTOFF | `to_descending`, `to_searching`, `to_approaching`, `to_returning` | At hover height; target state taken from the context | DESCENDING / SEARCHING / APPROACHING / RETURNING |
| RETURNING | `over_start` | Over start zone center | LANDING |
| LANDING | `landed` | PX4 landed and disarmed | RETURN\_CONFIRMED |
| Any except end states | `pause` | Comms `pause` | PAUSED |
| PAUSED | `resume_new_intent` | Comms `resume` after a new intent arrived while paused (release a die of the old color first) | SEARCHING, or LIFTOFF → SEARCHING if on the die |
| PAUSED | `resume_<state>` | Comms `resume`, no new intent | `paused_from` (a read cancelled by the pause restarts in NFC\_READY and does not count as a try) |
| Any | `aborted` | See global checks | ABORTED |
| Airborne or on the die | `return` | `return_to_start`, low battery, or force-return time | RETURNING, or LIFTOFF → RETURNING if on the die |
| TAKEOFF through RETURNING | `retarget_seen` / `retarget_unseen` | New `retrieve` color; release the gripper first if holding | APPROACHING / SEARCHING, through LIFTOFF if on the die |

### Color check

BrainChip confirms die color from the downward camera, published on `/perception/color_check`. GNC decides on a window of the last TBD messages, not one frame.

| Result | Rule | Action |
| --- | --- | --- |
| Right color | Confidence ≥ TBD and guess = target | Continue |
| Wrong color | Confidence ≥ TBD and guess ≠ target | Go to the target if the dice map has it elsewhere, else SEARCHING. Perception corrects the die's color in its own map |
| Nothing | Confidence < TBD | SEARCHING |

Run it at hover height before DESCENDING starts, again during the descent while the die is in view, and on entry to RETRIEVING.

### Intent handling

Every intent GNC handles sets `last_intent_id` and `last_intent_result` (`applied` or `rejected`) on `/mission/state`, published at once. Every message on `/mission/intent` is a new intent, whatever its ID: `intent_id` restarts at 1 when the comms node restarts, so never order or drop intents by ID (log a warning when it goes down). Empty fields mean "no change", and an empty action means `update_only`.

| Action | Effect |
| --- | --- |
| `retrieve`, first | Set target color; STANDBY → PREFLIGHT |
| `retrieve`, same color | No state change; apply region fields |
| `retrieve`, new color | Retarget (global check). While PAUSED: stored, counts as a new intent |
| `abort` | Any → ABORTED, including from PAUSED. Acts on the action alone |
| `return_to_start` | Airborne or on the die → RETURNING, including from PAUSED |
| `read_chip` | Go to NFC\_READY only if landed on the target die. **Rejected** otherwise: "read\_chip rejected: not on the die" |
| `pause` | Any state except the end states → PAUSED. Acts on the action alone |
| `resume` | Leave PAUSED by the resume rule. **Rejected** if not paused: "resume ignored: not paused" |
| `update_only` | No state change; apply regions and target quadrant. While PAUSED: also counts as a new intent |
| Any intent except `abort` | **Rejected** after RETURN\_CONFIRMED or ABORTED: "mission over: intent ignored" |

Those three are the only rejections. `reason` is plain English, under 80 characters, with no `|` character.

### Regions

Regions are halves (N, S, E, W) or quadrants (NE, NW, SE, SW) of the field, split at the field center, +y = north. The field encoding in `avc_msgs/Intent` (quadrant masks) is deferred; see open questions.

- **Avoid:** a no-fly area for the planner. Re-plan now and drop it from the search. Without an until, it lasts the whole mission.
- **Stay:** a route constraint; keep inside it.
- **Until:** when the drone reaches the until region, lift whichever rule came with it: the avoid, the stay, or both. A lifted rule is gone for good, even if the drone leaves the until region. Re-plan when a rule lifts, and publish "avoid lifted: reached SW" or similar.
- **Target quadrant:** search that quadrant first.
- If constraints make the target or the start zone unreachable, say so in `reason` instead of breaking them.

## Interfaces

Comms owns `Intent`, `MissionState`, `NfcResult` and `LlmHeartbeat` in `avc_msgs` (Abhi's page, Nov 12). GNC mirrors them exactly and never redefines them. Perception's messages stay in `avc_interfaces`.

**Inputs to mission\_manager**

| From | Topic | Type / contents | QoS |
| --- | --- | --- | --- |
| Comms | `/mission/intent` | `avc_msgs/Intent` | Reliable, volatile, keep last 10 |
| Comms | `/llm/heartbeat` | `avc_msgs/LlmHeartbeat`: alive, echoed `read_id`, comms' verdict on each NFC read (success or failed) | Reliable, volatile, keep last 10 (Abhi's proposal) |
| Perception | `/perception/dice_map` (placeholder) | Every die seen: color and xyz centroid in the field frame | TBD |
| Perception | `/perception/target_relative` | Target position relative to the drone from the downward camera | During APPROACHING and DESCENDING |
| Perception | `/perception/color_check` | `ColorCheck.msg` from BrainChip on the downward camera | APPROACHING to RETRIEVING |
| Perception | `/perception/obstacle_detections` | Obstacle position, size | Continuous |
| Perception | `/perception/heartbeat` | Alive, VIO and detector health | 5 Hz |
| Hardware | `/hw/nfc/status` | idle, reading, fault | On change |
| Hardware | `/hw/nfc/result` | success or failed, message text | After each read |
| Hardware | `/hw/gripper/status` | open, closed, holding, fault | On change |
| PX4 | `/fmu/out/vehicle_status`, `battery_status`, `vehicle_local_position`, `vehicle_land_detected`, `distance_sensor` | Arming, nav state, failsafe, battery, fused position, landed, rangefinder | Best effort |

**Outputs from mission\_manager**

| To | Topic | Type / contents | QoS |
| --- | --- | --- | --- |
| Everyone | `/mission/state` | `avc_msgs/MissionState`, 5 Hz and on every change or handled intent | Reliable, transient local, keep last 1 |
| Comms | `/mission/nfc_result` | `avc_msgs/NfcResult`, every read attempt; republished each heartbeat until comms gives its verdict | Reliable, transient local, keep last 10 |
| Everyone | `/mission/target` | Active color and centroid once known | Reliable, transient local |
| Hardware | `/hw/nfc/command` | start, cancel | Reliable |
| Hardware | `/hw/gripper/command` | open, close, release | Reliable |
| PX4 | `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command` | Offboard heartbeat, setpoints, arm / disarm / mode / land | Best effort |

**`avc_msgs/MissionState`** (comms' file; read constants from it, never retype them):

```
std_msgs/Header header
string state                # one of the STATE_* constants
string previous_state       # empty if there has been none
string reason               # plain English, under 80 characters, no pipe character
string active_target        # blue, red, yellow, black or empty
float32 mission_time_s
uint32 seq                  # +1 on every publish
uint32 last_intent_id       # newest intent handled, applied or rejected; 0 = none
string last_intent_result   # applied, rejected, or empty if none yet
```

One helper builds every MissionState: it bumps `seq`, sets `previous_state` (the paused-from state while PAUSED) and cuts `reason` to under 80 characters with any `|` removed. Comms' constants cover the 12 v2 states plus PAUSED; DESCENDING and LIFTOFF must be added by Abhi.

**`avc_msgs/Intent`** actions: `update_only` (0, also the default when empty), `retrieve` (1), `read_chip` (2), `abort` (3), `pause` (4), `return_to_start` (5), `resume` (6). Other fields: `target_color`, `avoid_mask`, `stay_mask`, `until_mask`, `target_quadrant`, `header`, `intent_id`. `pause` and `abort` from a stop word arrive with every other field empty.

**`ColorCheck.msg`** (Perception, in `avc_interfaces`):

```
std_msgs/Header header
string color_guess      # blue | red | yellow | black
float32 confidence      # 0 to 1
```

## File layout

All fifteen SMACH states live in one file, `mission_manager_smach.py`, with no rclpy imports. The node is a separate thin wrapper and the only executable that flies the mission.

```
~/dev/ros2_ws/src/
    avc_msgs -> avc-comms-ws/src/avc_msgs   (comms-owned, pinned at 375598d, never edited)
    avc_interfaces/msg/  ColorCheck.msg  TargetDetection.msg  (+ Perception's dice map message)
    gnc_offboard/

gnc_offboard/gnc_offboard/
    mission/
        context.py               # MissionContext, Inputs snapshot, state name constants from avc_msgs
        io.py                    # MissionIO abstract interface
        checks.py                # pure functions: color window, staleness, centering error, touchdown, regions and until rules
        qos.py                   # every QoS profile in one place
        state_msg.py             # the one MissionState builder
        mission_manager_smach.py # MissionStateBase + all 15 states + build_state_machine(ctx, io, cfg)
    planning/                    # search pattern, path planner (existing or stubs)
    guidance/px4_interface.py    # arming, disarming, offboard handshake, setpoints (lifted from mission_demo.py)
    ros/mission_manager_node.py  # subscriptions, timers, RosMissionIO, thread running sm.execute()
    sim/
        fake_intent.py           # CLI: any intent, including stop-word style with empty fields
        fake_comms.py            # LLM heartbeat; echoes read_id and gives a success or failed verdict (scriptable)
        fake_hardware.py         # NFC reader + gripper stand-in with configurable failures per motor stage
        fake_dice_map.py         # Perception dice map + target-relative + color check from the world manifest
        fake_heartbeats.py       # Perception heartbeat

gnc_offboard/config/mission.yaml
gnc_offboard/launch/mission.launch.py   # mission_manager, plus sim stand-ins when sim:=true
gnc_offboard/test/
    test_checks.py  test_states.py  test_global_checks.py  test_intents.py  test_state_msg.py  test_qos.py
```

Register `mission_manager = gnc_offboard.ros.mission_manager_node:main` in `setup.py`; add `smach` and `avc_msgs` to `package.xml`; install `config/` and `launch/` in `data_files`.

## Config and thresholds

Every value below is a ROS parameter loaded from `config/mission.yaml`. Use clearly marked placeholder defaults and a `# TBD` comment until test data sets them.

| Parameter | Used by | Placeholder |
| --- | --- | --- |
| `loop_hz` / `setpoint_hz` / `state_pub_hz` | Node, states | 20 / 20 / 5 |
| `ekf_stable_s`, `arm_timeout_s` | PREFLIGHT, LIFTOFF | 2.0, 10.0 |
| `search_altitude_m`, `altitude_tol_m` | TAKEOFF, SEARCHING | 1.5, 0.15 |
| `target_lost_s` | APPROACHING, DESCENDING | 2.0 |
| `hover_height_m` | APPROACHING, DESCENDING, LIFTOFF | 1.0 |
| `descent_stages_m` | DESCENDING | \[1.0, 0.5, 0.25\] |
| `descent_rate_mps` | DESCENDING | 0.15 |
| `stage_lateral_tol_m` | DESCENDING | \[0.10, 0.05, 0.03\] |
| `reader_lateral_tol_m` | DESCENDING (final) | 0.02 |
| `camera_blind_height_m` | DESCENDING | 0.20 |
| `touchdown_range_m`, `touchdown_hold_s` | DESCENDING | 0.05, 0.5 |
| `descent_max_retries` | DESCENDING | 2 |
| `color_conf_threshold`, `color_window` | Color check | 0.7, 5 |
| `nfc_start_timeout_s` | NFC\_READY | 2.0 |
| `nfc_tries_idle`, `nfc_tries_motors_off` | READING | 2, 2 |
| `retrieval_enabled`, `gripper_timeout_s` | RETRIEVING | false, 10.0 |
| `mission_time_limit_s`, `skip_retrieve_at_s`, `force_return_at_s` | Global, READING | 420, TBD, TBD |
| `battery_return_pct` | Global | 30 |
| `pose_invalid_s`, `perception_stale_s`, `llm_stale_s` | Global | 1.0, 1.0, 2.0 |
| `land_after_hold_s` | ABORTED | 3.0 |
| `start_zone_size_m` | RETURNING, LANDING | 1.524 (5 ft) |
| `field_north_axis` | Regions | +y |

Placeholders are starting guesses for sim only, not agreed numbers. There is deliberately no timeout on comms' NFC verdict.

## Sim stand-ins and testing

The state machine must be fully testable without Gazebo or PX4, then flown in PX4 SITL with stand-ins for the other teams.

**Unit tests (pytest, no ROS)**

- One test per row of the Transitions table: build a context, feed inputs through `FakeIO` and a fake clock, run the state, assert the outcome and the recorded actions.
- Global checks from every state they apply to, including return and retarget from the die (LIFTOFF first) and abort on the die (disarm and stay).

| Area | Test |
| --- | --- |
| MissionState | Each state publishes its comms `STATE_*` string; `previous_state` holds the paused-from state while PAUSED; `reason` is cut to under 80 characters with no `\|`; `seq` goes up on every publish |
| Intents | `last_intent_id` and `last_intent_result` are set for every intent; the three rejections publish their reasons |
| Intent IDs | IDs 5, 6, 1, 2 are all applied and echoed |
| Empty fields | `pause` and `abort` with every other field empty work; an empty action is applied as `update_only` |
| Until rules | Avoid + until lifts on arrival; avoid alone never lifts; avoid + stay + until lift together |
| QoS | Profiles in `qos.py` match the Interfaces tables |
| Pause | Pause and resume from STANDBY, PREFLIGHT (disarms), DESCENDING, LANDING and on the die; resume with no new intent returns to `paused_from`; new intent while paused resumes to SEARCHING (through LIFTOFF on the die); extra fields on `resume` are ignored |
| Dice map | SEARCHING's entry check goes straight to APPROACHING when the map already has the target |
| Descent | Stage-by-stage tolerance gating; retry on a large error or lost die; touchdown from the rangefinder |
| NFC read | Stage 1 idle, stage 2 motors off, then LIFTOFF and re-descend, then hold; nothing moves on until comms' verdict; a late verdict for an earlier `read_id` still counts |
| Staleness | A heartbeat that stops updating triggers its failure path after the configured time |
| Color check | One noisy frame does not change the result |

**Sim stand-ins (ROS nodes, sim only):** `fake_intent.py`, `fake_comms.py`, `fake_hardware.py`, `fake_dice_map.py` and `fake_heartbeats.py` (see File layout).

**SITL acceptance runs**

1. Happy path: STANDBY to RETURN\_CONFIRMED, landing on the die, one successful read at idle, no retrieval.
2. Wrong color first, target already in the dice map: goes straight to the right die.
3. Read fails twice at idle, then succeeds with motors off.
4. Both motor stages fail: LIFTOFF, re-descend, success.
5. Retarget mid-search and while on the die.
6. `abort` and `return_to_start` from each airborne state, from the die, and from PAUSED.
7. Perception heartbeat drops: hold, then land in place.
8. `pause` then `resume` with no new intent from SEARCHING, DESCENDING and READING: returns to the same state.
9. `pause`, new `retrieve` for a die already in the map, `resume`: SEARCHING then straight to APPROACHING.
10. Landing accuracy: 20 descents onto a die; report the touchdown offset from the centroid (mean and worst). Repeat under mocap in the lab.

Record each run with `ros2 bag record` so `/mission/state` gives a replayable timeline.

## Build order

One milestone per PR. Each ends with `colcon build` clean and all tests green.

1. **Comms messages and interfaces.** Clone `avc-comms-ws` into the `gnc` account at commit `375598d`, link `avc_msgs` into the workspace, add `ColorCheck.msg` and the dice map placeholder to `avc_interfaces`. Run `colcon test` and send the results to Abhi (he has no `rclpy` to test with).
2. **Core, no ROS.** `context.py`, `io.py`, `checks.py`, `qos.py`, `state_msg.py` and their tests.
3. **Retire mission\_demo; flight phases.** Lift the PX4 code from `mission_demo.py` into `px4_interface.py`, then remove `mission_demo` from `setup.py` and delete it. Build STANDBY → PREFLIGHT → TAKEOFF → hover → LANDING → RETURN\_CONFIRMED, plus ABORTED and the global checks. Fly it in SITL with `ros2 run gnc_offboard mission_manager`. Check every comms topic with `ros2 topic info -v` against comms' node.
4. **Intents and pause.** Intent handling, the three rejections, PAUSED and the resume rule, on the flight phases first.
5. **Search and approach.** SEARCHING, APPROACHING, the dice map lookup, color check, retarget, with `fake_dice_map.py`.
6. **Descent and liftoff.** DESCENDING and LIFTOFF, including re-arming after motors off. Run the landing-accuracy test.
7. **NFC.** NFC\_READY and READING, both motor stages, the comms verdict wait, with `fake_hardware.py` and `fake_comms.py`.
8. **Retrieve and return.** RETRIEVING, RETURNING, time thresholds, battery.
9. **Regions.** Avoid, stay, until rules and target quadrant (after the encoding question is settled).
10. **Launch and acceptance.** `mission.launch.py` with `sim:=true`, then the ten SITL runs.
11. **Jetson.** Run on the Jetson `gnc` account against mocap in the lab, props off first. Set `COM_DISARM_LAND` before any landing on a die.

## Updates from other teams

Each change here is already folded into the sections above.

| Team | Change | Affects |
| --- | --- | --- |
| Comms | Abhi's page (Nov 12): comms owns `avc_msgs`; new `resume` action; until ends an avoid as well as a stay; real `/mission/state` with `last_intent_id` and `last_intent_result`; QoS for all four comms topics; `intent_id` restarts at 1 with comms | Interfaces, Intent handling, PAUSED, Regions |
| Comms | GNC moves on from a read only on comms' success or failed verdict | READING |
| Perception | Perception owns the dice map (xyz centroid of each die); GNC keeps no die memory | SEARCHING, APPROACHING, DESCENDING |
| Perception | BrainChip confirms color from the downward camera, which also drives the descent | Color check, DESCENDING |
| Hardware |  |  |
| GNC | `mission_demo.py` retired; `mission_manager` is the main script | Context, Build order |
| GNC | Land on the die before the read and the grab; two-stage read (motors idle, then off) | States, Transitions, Config |

## Open questions

The builder should code these behind a parameter or a clearly marked default, not guess.

**For Comms (Abhi)**

- [ ] **DESCENDING and LIFTOFF:** add both to the `STATE_*` constants in `MissionState`.
- [ ] **NFC verdict in `LlmHeartbeat`:** which field carries comms' success or failed verdict for a `read_id`? The outline assumes an echoed `read_id` plus a verdict.
- [ ] **`/llm/heartbeat` QoS and `link_ok`:** his proposal is accepted on our side; confirm with him.
- [ ] **Region encoding:** `avc_msgs/Intent` sends quadrant masks (NE 1, SE 2, SW 4, NW 8). Decoding is deferred.
- [ ] **ABORTED behavior** (his question 5): he says abort was meant to return to start; the outline defaults to land in place. Must stay distinct from `return_to_start`.
- [ ] **`stay` without `until`:** for the rest of the mission?

**For Perception**

- [ ] **Dice map:** topic name, message type and rate.
- [ ] **Final approach:** rate and accuracy of `/perception/target_relative` below 1 m, and the camera blind height.
- [ ] **Color threshold and window:** test data under venue lighting.

**For Hardware**

- [ ] **Reader tolerance:** does the frame straddle the die, and how close does the reader need to be (placeholder 2 cm)?
- [ ] **NFC tries per motor stage** and time per read.
- [ ] **Gripper:** planned for competition, and how long a grab takes on the die.

**For GNC**

- [ ] **PX4 auto-disarm:** set `COM_DISARM_LAND` so motor stage 1 holds idle on the die.
- [ ] **Field north:** which way, and who sets it on the day.
