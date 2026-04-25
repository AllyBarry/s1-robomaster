#!/bin/bash
# Two-phase formation experiment: hold a line, then transition to a
# triangle. Restarting the belief+reward bundle between phases re-seeds
# every belief posterior — the previous learned gradient was for the
# line landscape and is stale once targets move.
#
# Edit the CONFIG block below and run:
#   ./experiment_line_to_triangle.sh
#
# Containers are torn down on exit (Ctrl+C, error, or natural finish).

set -e
cd "$(dirname "$0")"

# ============================================================
# CONFIG — edit each line as needed
# ============================================================
ROBOT_IDS="0,1,2"               # comma-separated, must be ≥ 3 for triangle
CENTER_X="1.5"                  # m, formation centre x in field frame
CENTER_Y="1.5"                  # m, formation centre y

LINE_SPACING="0.4"              # m, neighbour-to-neighbour spacing in the line
LINE_DURATION_SEC=30            # how long to hold the line formation (s)

TRIANGLE_SPACING="0.5"          # m, ring radius around the centre
TRIANGLE_DURATION_SEC=30        # how long to hold the triangle (s)

ASSIGNMENT="ordered"            # ordered | nearest
COLLISION="--collision"         # --collision | --no-collision
WAYPOINTS="--with-waypoints"    # --with-waypoints | --no-waypoints
SETTLE_SEC=2                    # grace period before flipping the runtime collision toggle
# ============================================================

NAV_IDS=$(echo "${ROBOT_IDS}" | tr ',' ' ')

cleanup() {
    echo ""
    echo "=== Cleanup ==="
    docker rm -f belief_and_reward >/dev/null 2>&1 || true
    for id in ${NAV_IDS}; do
        docker rm -f "robot_navigator_${id}" >/dev/null 2>&1 || true
    done
    echo "Containers removed."
}
trap cleanup EXIT INT TERM

# Spawns a one-shot container to publish the collision-avoidance toggle.
# The launch arg already enables the planner's repulsion; this flips the
# navigator's reactive layer (whose yaml default is off).
publish_collision_toggle() {
    local state="$1"
    docker compose run --rm robot_navigator \
        ros2 topic pub --once /collision_avoidance_enabled \
        std_msgs/Bool "{data: ${state}}" >/dev/null 2>&1 || true
}

run_phase() {
    local phase_name="$1"
    local formation="$2"
    local spacing="$3"
    local duration="$4"

    echo ""
    echo "=== ${phase_name}: ${formation} formation, ${duration}s ==="
    ./run_belief_and_reward.sh \
        ${COLLISION} \
        --robots "${ROBOT_IDS}" \
        --formation "${formation}" \
        --spacing "${spacing}" \
        --center-x "${CENTER_X}" \
        --center-y "${CENTER_Y}" \
        --assignment "${ASSIGNMENT}" \
        ${WAYPOINTS}

    if [ "${COLLISION}" = "--collision" ]; then
        sleep "${SETTLE_SEC}"
        publish_collision_toggle "true"
    fi

    sleep "${duration}"
}

# Navigators are launched once and kept alive across both phases — that
# way the robots coast smoothly while the belief bundle restarts and
# pick up the new UCB waypoints when they arrive.
echo "=== Starting navigators for: ${NAV_IDS} ==="
./run_navigator.sh ${NAV_IDS}

run_phase "Phase 1" "line" "${LINE_SPACING}" "${LINE_DURATION_SEC}"
run_phase "Phase 2" "triangle" "${TRIANGLE_SPACING}" "${TRIANGLE_DURATION_SEC}"

echo ""
echo "=== Experiment complete ==="
