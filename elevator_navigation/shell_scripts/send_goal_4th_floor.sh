#!/bin/bash

# Arguments with default fallbacks:
# Usage: ./send_goal.sh [BUILDING] [FLOOR] [X] [Y] [YAW]
BUILDING=${1:-"D-Block"}
FLOOR=${2:-4}
X=${3:-10.0}
Y=${4:-5.0}
YAW=${5:-1.57}

echo "=========================================="
echo "🚀 Dispatching Goal to Wheelchair Planner"
echo "   Building: $BUILDING"
echo "   Floor:    $FLOOR"
echo "   Target:   [$X, $Y, $YAW]"
echo "=========================================="

JSON_PAYLOAD="{\"bldg\": \"$BUILDING\", \"floor\": $FLOOR, \"pose\": [$X, $Y, $YAW]}"

ros2 topic pub --once /send_navigation_goal std_msgs/msg/String "{data: '$JSON_PAYLOAD'}"

echo "✅ Goal message dispatched to /send_navigation_goal."