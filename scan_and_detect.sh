#!/usr/bin/env bash
# Start the table detector and the overlay viewer (rqt_image_view) -- one terminal.
# TABLE_CAMERA picks the camera (default "scene"):
#   scene - Sheppy's scene camera, fixed to the chair: the arm doesn't move.
#   wrist - first moves the arm to the scan pose; the detector only starts if
#           scan_pose.py confirms the arm arrived (an abort, refusal or missed pose stops here).
#
#   ./scan_and_detect.sh                         # scene camera
#   ./scan_and_detect.sh --ros-args -p max_plane_dist:=0.8   # rest goes to the node
#   TABLE_CAMERA=wrist ./scan_and_detect.sh       # [ENTER] prompt before the arm moves
#   TABLE_CAMERA=wrist ./scan_and_detect.sh --yes # no prompt (for recording)
#
# rqt_image_view needs a screen; over plain ssh it can't open (use save_frame.py --overlay).
cd "$(dirname "$0")"

SCAN_ARGS=()
if [[ "$1" == "--yes" ]]; then
    SCAN_ARGS+=(--yes)
    shift
fi

if [[ "${TABLE_CAMERA:-scene}" == "wrist" ]]; then
    python3 scan_pose.py "${SCAN_ARGS[@]}" || { echo "Not at scan pose -- detector not started."; exit 1; }
fi

# Overlay viewer in the background; closed when the detector exits (incl. Ctrl-C).
# `ros2 run` is a wrapper, so kill its child too, not just the wrapper's PID.
ros2 run rqt_image_view rqt_image_view /table_detector/overlay >/dev/null 2>&1 &
VIEWER=$!
trap 'pkill -TERM -P $VIEWER 2>/dev/null; kill -TERM $VIEWER 2>/dev/null' EXIT

echo "Starting table detector ..."
python3 table_detector_node.py "$@"
