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
#
# Logs (CSV + JSON sidecar) land in ./experiment_logs/ on the host.
# Plot them offline with:
#   python3 scripts/plot_experiment.py experiment_logs/
#
# Examples:
#   ./run_belief_and_reward.sh                                   # all defaults
#   ./run_belief_and_reward.sh --scenario baseline --duration 120
#   ./run_belief_and_reward.sh --robots 0,1,2 --formation line --spacing 0.4
#   ./run_belief_and_reward.sh --formation triangle --center-x 1.5 --center-y 1.0
#   ./run_belief_and_reward.sh --rebuild --robots 0,1 --with-waypoints

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
        -h|--help)
            sed -n '2,43p' "$0"
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

# Only forward args the user actually supplied; anything omitted falls
# back to the launch file's defaults.
LAUNCH_ARGS=()
[ -n "${ROBOT_IDS}"         ] && LAUNCH_ARGS+=("robot_ids:=${ROBOT_IDS}")
[ -n "${FORMATION}"         ] && LAUNCH_ARGS+=("formation:=${FORMATION}")
[ -n "${SPACING}"           ] && LAUNCH_ARGS+=("formation_spacing:=${SPACING}")
[ -n "${CENTER_X}"          ] && LAUNCH_ARGS+=("formation_center_x:=${CENTER_X}")
[ -n "${CENTER_Y}"          ] && LAUNCH_ARGS+=("formation_center_y:=${CENTER_Y}")
[ -n "${ASSIGNMENT}"        ] && LAUNCH_ARGS+=("assignment:=${ASSIGNMENT}")
[ -n "${PUBLISH_WAYPOINTS}" ] && LAUNCH_ARGS+=("publish_waypoints:=${PUBLISH_WAYPOINTS}")
[ -n "${COLLISION_AVOIDANCE}" ] && LAUNCH_ARGS+=("collision_avoidance:=${COLLISION_AVOIDANCE}")
[ -n "${SCENARIO}"          ] && LAUNCH_ARGS+=("scenario:=${SCENARIO}")
[ -n "${DURATION}"          ] && LAUNCH_ARGS+=("duration_sec:=${DURATION}")
[ -n "${SAMPLE_HZ}"         ] && LAUNCH_ARGS+=("sample_hz:=${SAMPLE_HZ}")

CONTAINER_NAME="belief_and_reward"

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator belief_and_reward.launch.py "${LAUNCH_ARGS[@]}"

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
echo ""
echo "Reward + per-robot beliefs are up. Launch navigators separately:"
echo "  ./run_navigator.sh 0 1 2"
echo ""
echo "Monitor reward:   ros2 topic echo /global_reward"
echo "Monitor waypoint: ros2 topic echo /robot_0/waypoint"
