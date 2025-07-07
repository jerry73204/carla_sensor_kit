#!/bin/bash
# ROS environment setup with command execution

# Check if RMW implementation is specified as environment variable or first argument
if [[ "$1" == "--rmw" ]] && [[ -n "$2" ]]; then
    export RMW_IMPLEMENTATION="$2"
    shift 2  # Remove --rmw and its value from arguments
else
    # Use environment variable or default to fastrtps
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
source /opt/ros/humble/setup.bash

if [ $# -eq 0 ]; then
    echo "ROS environment ready:"
    echo "  ROS_DISTRO=$ROS_DISTRO"
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo ""
    echo "Usage: $0 [--rmw <implementation>] <command> [args...]"
    echo "   or: RMW_IMPLEMENTATION=<impl> $0 <command> [args...]"
else
    exec "$@"
fi