#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/poliastro/install/setup.bash 

exec "$@"
