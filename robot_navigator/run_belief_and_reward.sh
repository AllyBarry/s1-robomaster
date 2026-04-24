#!/bin/bash
# Bundled launch: global_feedback + one belief node per robot_id, in a
# single container. Couples their lifecycles — restarting this container
# re-seeds every belief posterior against fresh targets (a belief's
# learned gradient is only valid for the current reward landscape, and
# `forgetting=0.999` is too slow to wash out a stale posterior).
#
# The navigator is intentionally kept separate (./run_navigator.sh) so
# robots can coast during a belief re-seed instead of being killed
# mid-transit.
#
# All knobs are named flags; anything omitted falls back to the launch
# file default (see belief_and_reward.launch.py).
#
# Usage:
#   ./run_belief_and_reward.sh [flags]
#
# Flags:
#   --rebuild | -r                 Rebuild image from scratch before running
#   --robots <csv>                 Robot IDs, e.g. 0,1,2
#   --formation <name>             line | triangle | circle | custom
#   --spacing <float>              Line spacing or triangle/circle radius (m)
#   --center-x <float>             Formation centre x (m)
#   --center-y <float>             Formation centre y (m)
#   --assignment <name>            ordered | nearest
#   --with-waypoints               Publish UCB waypoints from belief nodes
#   --no-waypoints                 Disable waypoint publishing (viz only)
#   --collision                    Enable peer repulsion in the belief planner (off by default)
#   --no-collision                 Disable peer repulsion in the belief planner (default)
#   --scenario <name>              Episode name — CSV/JSON filename stem
#   --duration <sec>               Auto-stop logger after N seconds (0 = run until killed)
#   --sample-hz <float>            Logger sample rate
#   --record-video                 Record overhead camera + trajectory overlay to {scenario}.mp4
#   --no-record-video              Disable video recording (default)
#   --video-fps <float>            Output mp4 frame rate (default: 5, matches detection_overlay)
#   --run-name <name>              Override the auto-generated run folder name
#                                  (default: {scenario}_{YYYYMMDD_HHMMSS})
#
# Each invocation writes to its own folder under ./experiment_logs/ — prior
# runs are never overwritten. The folder contains:
#   {scenario}.csv, {scenario}.json, plots/ (populated by the plotter),
#   and {scenario}.mp4 when --record-video is set.
# Plot offline with:
#   python3 scripts/plot_experiment.py experiment_logs/{run_folder}/
#
# Examples:
#   ./run_belief_and_reward.sh                                   # all defaults
#   ./run_belief_and_reward.sh --scenario baseline --duration 120
#   ./run_belief_and_reward.sh --robots 0,1,2 --formation line --spacing 0.4
#   ./run_belief_and_reward.sh --formation triangle --center-x 1.25 --center-y 1.1
#   ./run_belief_and_reward.sh --rebuild --robots 0,1 --with-waypoints
#   ./run_belief_and_reward.sh --scenario run1 --duration 60 --record-video

set -e

REBUILD=0
ROBOT_IDS=""
FORMATION="line"
SPACING="1"
CENTER_X="1.5"
CENTER_Y="1.5"
ASSIGNMENT=""
PUBLISH_WAYPOINTS=""
COLLISION_AVOIDANCE=""
SCENARIO=""
DURATION="100"
SAMPLE_HZ=""
RECORD_VIDEO="true"
VIDEO_FPS=""
RUN_NAME=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rebuild|-r)
            REBUILD=1
            shift
            ;;
        --robots)
            ROBOT_IDS="$2"
            shift 2
            ;;
        --formation)
            FORMATION="$2"
            shift 2
            ;;
        --spacing)
            SPACING="$2"
            shift 2
            ;;
        --center-x)
            CENTER_X="$2"
            shift 2
            ;;
        --center-y)
            CENTER_Y="$2"
            shift 2
            ;;
        --assignment)
            ASSIGNMENT="$2"
            shift 2
            ;;
        --with-waypoints)
            PUBLISH_WAYPOINTS="true"
            shift
            ;;
        --no-waypoints)
            PUBLISH_WAYPOINTS="false"
            shift
            ;;
        --collision)
            COLLISION_AVOIDANCE="true"
            shift
            ;;
        --no-collision)
            COLLISION_AVOIDANCE="false"
            shift
            ;;
        --scenario)
            SCENARIO="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --sample-hz)
            SAMPLE_HZ="$2"
            shift 2
            ;;
        --record-video)
            RECORD_VIDEO="true"
            shift
            ;;
        --no-record-video)
            RECORD_VIDEO="false"
            shift
            ;;
        --video-fps)
            VIDEO_FPS="$2"
            shift 2
            ;;
        --run-name)
            RUN_NAME="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,51p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Run with --help to see flags."
            exit 1
            ;;
    esac
done

if [ "${REBUILD}" = "1" ]; then
    echo "Rebuilding image from scratch..."
    docker compose build --no-cache
else
    docker compose build
fi
# If build fails on Jetson with an iptables `raw` table error, run:
#   DOCKER_BUILDKIT=0 docker build --network=host -t robot_navigator-robot_navigator .
# or ensure `iptable_raw` is loaded: `sudo modprobe iptable_raw`

# One folder per invocation: {scenario}_{timestamp} unless --run-name is
# given. Pre-created on the host so it inherits user ownership; the
# bind-mounted volume surfaces container writes into it. Prior runs are
# never overwritten.
SCENARIO_NAME="${SCENARIO:-run}"
if [ -z "${RUN_NAME}" ]; then
    RUN_NAME="${SCENARIO_NAME}_$(date +%Y%m%d_%H%M%S)"
fi
HOST_RUN_DIR="./experiment_logs/${RUN_NAME}"
CONTAINER_RUN_DIR="/ros_ws/experiment_logs/${RUN_NAME}"
mkdir -p "${HOST_RUN_DIR}"

# Only forward args the user actually supplied; anything omitted falls
# back to the launch file's defaults. `scenario` and `log_dir` are always
# forwarded so the unique run folder is honoured.
LAUNCH_ARGS=()
LAUNCH_ARGS+=("scenario:=${SCENARIO_NAME}")
LAUNCH_ARGS+=("log_dir:=${CONTAINER_RUN_DIR}")
[ -n "${ROBOT_IDS}"         ] && LAUNCH_ARGS+=("robot_ids:=${ROBOT_IDS}")
[ -n "${FORMATION}"         ] && LAUNCH_ARGS+=("formation:=${FORMATION}")
[ -n "${SPACING}"           ] && LAUNCH_ARGS+=("formation_spacing:=${SPACING}")
[ -n "${CENTER_X}"          ] && LAUNCH_ARGS+=("formation_center_x:=${CENTER_X}")
[ -n "${CENTER_Y}"          ] && LAUNCH_ARGS+=("formation_center_y:=${CENTER_Y}")
[ -n "${ASSIGNMENT}"        ] && LAUNCH_ARGS+=("assignment:=${ASSIGNMENT}")
[ -n "${PUBLISH_WAYPOINTS}" ] && LAUNCH_ARGS+=("publish_waypoints:=${PUBLISH_WAYPOINTS}")
[ -n "${COLLISION_AVOIDANCE}" ] && LAUNCH_ARGS+=("collision_avoidance:=${COLLISION_AVOIDANCE}")
[ -n "${DURATION}"          ] && LAUNCH_ARGS+=("duration_sec:=${DURATION}")
[ -n "${SAMPLE_HZ}"         ] && LAUNCH_ARGS+=("sample_hz:=${SAMPLE_HZ}")
[ -n "${RECORD_VIDEO}"      ] && LAUNCH_ARGS+=("record_video:=${RECORD_VIDEO}")
[ -n "${VIDEO_FPS}"         ] && LAUNCH_ARGS+=("video_fps:=${VIDEO_FPS}")

CONTAINER_NAME="belief_and_reward"

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator belief_and_reward.launch.py "${LAUNCH_ARGS[@]}"

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
echo ""
echo "Run folder: ${HOST_RUN_DIR}"
echo ""
echo "Reward + per-robot beliefs are up. Launch navigators separately:"
echo "  ./run_navigator.sh 0 1 2"
echo ""
echo "Monitor reward:   ros2 topic echo /global_reward"
echo "Monitor waypoint: ros2 topic echo /robot_0/waypoint"

# Plot via a one-off container: reuses the robot_navigator image (which
# has a matching numpy/matplotlib/pandas from apt), so we don't depend on
# the Jetson host's Python stack. --user keeps PNG ownership on the host
# user; HOME=/tmp avoids matplotlib cache warnings under a non-root UID.
plot_in_container() {
    docker compose run --rm \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        --entrypoint python3 \
        robot_navigator \
        /ros_ws/scripts/plot_experiment.py "${CONTAINER_RUN_DIR}"
}

PLOT_FALLBACK_HINT="plotter container failed — rerun manually: docker compose run --rm --user \$(id -u):\$(id -g) -e HOME=/tmp --entrypoint python3 robot_navigator /ros_ws/scripts/plot_experiment.py ${CONTAINER_RUN_DIR}"

# Ctrl-C path: tell the container to shut down gracefully so video_recorder's
# atexit/SIGTERM handlers run (finalize the mp4 moov atom) before the
# process dies, then plot the trajectory overlays. `docker stop -t 15`
# gives ros2 launch and its children 15 s to unwind — plenty since the
# recorder's release is sub-second, but generous enough that the grace
# window won't clip a slow belief-node shutdown.
cleanup_on_interrupt() {
    trap - INT TERM                        # re-interrupt = hard abort
    echo ""
    echo "Interrupted — stopping ${CONTAINER_NAME} so the video finalizes..."
    docker stop -t 15 "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    echo "Generating plots in ${HOST_RUN_DIR}/plots/..."
    plot_in_container || echo "${PLOT_FALLBACK_HINT}"
    exit 130
}
trap cleanup_on_interrupt INT TERM

# Stay attached so Ctrl-C triggers the trap above, whether this is a
# --duration run (container self-exits when trajectory_logger's on_exit
# Shutdown() cascades) or unbounded (docker wait blocks until the user
# interrupts us or an external `docker stop`).
echo ""
if [ -n "${DURATION}" ] && awk "BEGIN{exit !(${DURATION}>0)}" 2>/dev/null; then
    echo "Waiting for ${CONTAINER_NAME} to exit (duration=${DURATION}s, Ctrl-C to stop early)..."
else
    echo "Run active — press Ctrl-C to stop, finalize the video, and plot."
fi
docker wait "${CONTAINER_NAME}" >/dev/null || true
trap - INT TERM

echo "${CONTAINER_NAME} exited — generating plots in ${HOST_RUN_DIR}/plots/"
plot_in_container || echo "${PLOT_FALLBACK_HINT}"
