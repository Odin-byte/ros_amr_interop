#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /dev_ws/install/local_setup.bash

eval "$*"
