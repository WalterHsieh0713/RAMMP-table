#!/usr/bin/env bash
# Move the arm to the scan pose, then start the table detector and the overlay
# viewer (rqt_image_view) -- one terminal.
# The detector only starts if scan_pose.py confirms the arm arrived; an abort,
# refusal or missed pose stops here.
#
#   ./scan_and_detect.sh                 # [ENTER] prompt before the arm moves
#   ./scan_and_detect.sh --yes           # no prompt (for recording)
#   ./scan_and_detect.sh --yes --ros-args -p bowl_radius:=0.075   # rest goes to the node
cd "$(dirname "$0")"

SCAN_ARGS=()
if [[ "$1" == "--yes" ]]; then
    SCAN_ARGS+=(--yes)
    shift
fi

python3 scan_pose.py "${SCAN_ARGS[@]}" || { echo "Not at scan pose -- detector not started."; exit 1; }

# Overlay viewer in the background; closed when the detector exits (incl. Ctrl-C).
# `ros2 run` is a wrapper, so kill its child too, not just the wrapper's PID.
ros2 run rqt_image_view rqt_image_view /table_detector/overlay >/dev/null 2>&1 &
VIEWER=$!
trap 'pkill -TERM -P $VIEWER 2>/dev/null; kill -TERM $VIEWER 2>/dev/null' EXIT

echo "Starting table detector ..."
python3 table_detector_node.py "$@"
