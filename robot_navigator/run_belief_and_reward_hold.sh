#!/bin/bash
# Probe-model variant of ./run_belief_and_reward.sh — identical bundle
# (global_feedback + per-robot beliefs + trajectory logger + optional
# video recorder) but with the 1-bit hold/probe coordination channel
# enabled on every belief node. Use this to A/B against the baseline
# UCB + repulsion run.
#
# When a belief raises /field/robot_{id}/hold_request, every peer freezes
# at its current cell until the holder auto-releases (hold_duration_sec).
# Credit assignment becomes single-mover so the per-robot posterior isn't
# polluted by overlapping peer movement.
#
# Baseline comparison: ./run_belief_and_reward.sh (same flags, no hold).
# Output run folders are timestamp-prefixed so both variants live side
# by side under ./experiment_logs/.
#
# Usage:
#   ./run_belief_and_reward_hold.sh [flags]
#
# Flags: identical to ./run_belief_and_reward.sh — see --help below.

set -e

REBUILD=0
ROBOT_IDS=""
FORMATION=""
SPACING=""
CENTER_X=""
CENTER_Y=""
ASSIGNMENT=""
PUBLISH_WAYPOINTS=""
COLLISION_AVOIDANCE=""
SCENARIO=""
DURATION=""
SAMPLE_HZ=""
RECORD_VIDEO=""
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
            sed -n '2,24p' "$0"
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

# Distinct scenario/run-folder stem so probe runs don't overwrite or
# visually merge with baseline runs. Explicit --run-name still wins.
SCENARIO_NAME="${SCENARIO:-run_hold}"
if [ -z "${RUN_NAME}" ]; then
    RUN_NAME="${SCENARIO_NAME}_$(date +%Y%m%d_%H%M%S)"
fi
HOST_RUN_DIR="./experiment_logs/${RUN_NAME}"
CONTAINER_RUN_DIR="/ros_ws/experiment_logs/${RUN_NAME}"
mkdir -p "${HOST_RUN_DIR}"

LAUNCH_ARGS=()
LAUNCH_ARGS+=("scenario:=${SCENARIO_NAME}")
LAUNCH_ARGS+=("log_dir:=${CONTAINER_RUN_DIR}")
# The one knob that differs from the baseline script.
LAUNCH_ARGS+=("hold_enabled:=true")
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

# Separate container name so a stale baseline container doesn't block
# us (and vice-versa). Still a singleton — can't coexist with a live
# baseline run because the two fight over /global_reward.
CONTAINER_NAME="belief_and_reward_hold"

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator belief_and_reward.launch.py "${LAUNCH_ARGS[@]}"

echo "Started ${CONTAINER_NAME} (HOLD model) — follow with: docker logs -f ${CONTAINER_NAME}"
echo ""
echo "Run folder: ${HOST_RUN_DIR}"
echo ""
echo "Probe bit per robot:  ros2 topic echo /field/robot_0/hold_request"
echo "Reward + beliefs up.  Launch navigators separately: ./run_navigator.sh 0 1 2"
echo ""
echo "Monitor reward:   ros2 topic echo /global_reward"
echo "Monitor waypoint: ros2 topic echo /robot_0/waypoint"

plot_in_container() {
    docker compose run --rm \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        --entrypoint python3 \
        robot_navigator \
        /ros_ws/scripts/plot_experiment.py "${CONTAINER_RUN_DIR}"
}

PLOT_FALLBACK_HINT="plotter container failed — rerun manually: docker compose run --rm --user \$(id -u):\$(id -g) -e HOME=/tmp --entrypoint python3 robot_navigator /ros_ws/scripts/plot_experiment.py ${CONTAINER_RUN_DIR}"

cleanup_on_interrupt() {
    trap - INT TERM
    echo ""
    echo "Interrupted — stopping ${CONTAINER_NAME} so the video finalizes..."
    docker stop -t 15 "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    echo "Generating plots in ${HOST_RUN_DIR}/plots/..."
    plot_in_container || echo "${PLOT_FALLBACK_HINT}"
    exit 130
}
trap cleanup_on_interrupt INT TERM

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
