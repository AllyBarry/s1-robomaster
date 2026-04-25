#!/bin/bash
# Spatiotemporal formation experiment: cycle through a sequence of
# formations, transitioning when the team has *actually reached* each
# one (not on a fixed timer).
#
# Per phase:
#   1. (Re)launch the belief+reward bundle with the new formation.
#      This kills the previous bundle, so beliefs reset to a zero-mean
#      posterior — the prior gradient was for the OLD reward landscape
#      and is stale once targets move.
#   2. Wait until /global_reward stays above REACH_THRESHOLD for
#      HOLD_SEC consecutive seconds (formation reached). Falls through
#      after TIMEOUT_SEC regardless, so a stuck phase doesn't hang.
#   3. Move on to the next phase.
#
# Navigators stay alive across all phases — robots coast briefly while
# beliefs re-seed instead of stopping mid-transit.
#
# Edit CONFIG below, then ./experiment_spatiotemporal.sh

set -e
cd "$(dirname "$0")"

# ============================================================
# CONFIG — edit each line
# ============================================================
ROBOT_IDS="0,1,2"               # comma-separated; triangle phases need ≥ 3
CENTER_X="1.5"                  # m, formation centre x
CENTER_Y="1.5"                  # m, formation centre y

# Phase sequence — each entry is "formation spacing".
# formation: line | triangle | circle
PHASES=(
    "triangle 0.5"
    "line 0.4"
    "triangle 0.5"
    "line 0.4"
)

# Transition trigger:
#   reward = -Σ(distance_to_target). Reaches 0 when every robot is on
#   its target. REACH_THRESHOLD = -0.2 means roughly 7 cm avg per-robot
#   error in a 3-robot team — tight enough to call "converged."
REACH_THRESHOLD="-0.2"          # higher (closer to 0) = stricter
HOLD_SEC="3.0"                  # must stay above threshold this long
TIMEOUT_SEC="60"                # max time per phase before giving up

ASSIGNMENT="ordered"            # ordered | nearest
COLLISION="--collision"         # --collision | --no-collision
WAYPOINTS="--with-waypoints"    # --with-waypoints | --no-waypoints
SETTLE_SEC=2                    # grace period after launching the bundle
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

publish_collision_toggle() {
    docker compose run --rm robot_navigator \
        ros2 topic pub --once /collision_avoidance_enabled \
        std_msgs/Bool "{data: $1}" >/dev/null 2>&1 || true
}

# Spawns a one-shot helper container that subscribes to /global_reward.
# Exits 0 when reward held above threshold for HOLD_SEC, exits 1 on
# timeout. Either way we move on — `|| true` keeps `set -e` happy.
wait_until_reached() {
    docker compose run --rm robot_navigator \
        ros2 run robot_navigator wait_for_reward \
        "${REACH_THRESHOLD}" "${HOLD_SEC}" "${TIMEOUT_SEC}" || true
}

run_phase() {
    local idx="$1"
    local formation="$2"
    local spacing="$3"

    echo ""
    echo "=== Phase ${idx}: formation=${formation} spacing=${spacing} ==="
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

    wait_until_reached
}

echo "=== Starting navigators for: ${NAV_IDS} ==="
./run_navigator.sh ${NAV_IDS}

idx=1
for phase in "${PHASES[@]}"; do
    formation=$(echo "${phase}" | awk '{print $1}')
    spacing=$(echo "${phase}" | awk '{print $2}')
    run_phase "${idx}" "${formation}" "${spacing}"
    idx=$((idx + 1))
done

echo ""
echo "=== All phases complete ==="
