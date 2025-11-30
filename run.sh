#!/usr/bin/env bash
set -eo pipefail

MODE=${1:-all}
shift $(( $# > 0 ? 1 : 0 )) 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
YOLO_ENV_PATH="${YOLO_ENV:-$HOME/yolo_clip_env}"

if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
else
  echo "[run.sh] ROS 2 setup file not found at ${ROS_SETUP}" >&2
  exit 1
fi

if [[ -f "${WS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
else
  echo "[run.sh] Workspace setup file not found at ${WS_SETUP}. Run 'colcon build' first." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"

run_navigation() {
  ros2 launch ri_pkg autonomous_navigation.launch.py "${@}"
}

run_perception_waffle() {
  if [[ -d "${YOLO_ENV_PATH}" ]]; then
    # shellcheck disable=SC1090
    source "${YOLO_ENV_PATH}/bin/activate"
  else
    echo "[run.sh] YOLO environment not found at ${YOLO_ENV_PATH}." >&2
    echo "           Set YOLO_ENV to the correct virtualenv path or create the environment." >&2
    exit 1
  fi
  echo "[run.sh] Running perception_waffle.py for hardware..."
  python3 "${WORKSPACE_DIR}/src/ri_pkg/ri_pkg/perception_waffle.py" "${@}"
}

run_perception_internvl() {
  if [[ -d "${YOLO_ENV_PATH}" ]]; then
    # shellcheck disable=SC1090
    source "${YOLO_ENV_PATH}/bin/activate"
  else
    echo "[run.sh] YOLO environment not found at ${YOLO_ENV_PATH}." >&2
    echo "           Set YOLO_ENV to the correct virtualenv path or create the environment." >&2
    exit 1
  fi
  echo "[run.sh] Running perception_internvl.py for simulation..."
  python3 "${WORKSPACE_DIR}/src/ri_pkg/ri_pkg/perception_internvl.py" "${@}"
}

case "${MODE}" in
  navigation)
    run_navigation "$@"
    ;;
  perception_waffle|waffle)
    run_perception_waffle "$@"
    ;;
  perception_internvl|internvl|gazebo)
    run_perception_internvl "$@"
    ;;
  all_waffle)
    echo "[run.sh] Starting autonomous navigation and perception (hardware)..."
    run_navigation "$@" &
    NAV_PID=$!

    sleep 5

    run_perception_waffle "$@" &
    PERC_PID=$!

    trap 'echo "[run.sh] Stopping..."; kill ${NAV_PID} ${PERC_PID} 2>/dev/null || true' INT TERM
    wait ${NAV_PID} ${PERC_PID}
    ;;
  all_internvl|all_gazebo|all)
    echo "[run.sh] Starting autonomous navigation and perception (simulation)..."
    run_navigation "$@" &
    NAV_PID=$!

    sleep 5

    run_perception_internvl "$@" &
    PERC_PID=$!

    trap 'echo "[run.sh] Stopping..."; kill ${NAV_PID} ${PERC_PID} 2>/dev/null || true' INT TERM
    wait ${NAV_PID} ${PERC_PID}
    ;;
  *)
    cat <<'USAGE'
Usage: run.sh [MODE] [-- ros2 args]

Modes:
  navigation                  Launch only the autonomous navigation stack
  
  Perception modes:
  perception_waffle | waffle  Run perception for hardware (TurtleBot3 Waffle Pi)
  perception_internvl | internvl | gazebo
                             Run perception for Gazebo simulation
  
  Combined modes:
  all | all_internvl | all_gazebo
                             Start navigation + perception (simulation) [default]
  all_waffle                 Start navigation + perception (hardware)

Environment variables:
  YOLO_ENV           Path to virtual environment for perception (default: $HOME/yolo_clip_env)
  TURTLEBOT3_MODEL   TurtleBot3 model to use (default: waffle_pi)

Examples:
  ./run.sh all                    # Run everything for simulation
  ./run.sh all_waffle             # Run everything for hardware
  ./run.sh navigation             # Run only navigation
  ./run.sh perception_internvl    # Run only perception for simulation
  ./run.sh waffle                 # Run only perception for hardware
USAGE
    exit 1
    ;;
esac

