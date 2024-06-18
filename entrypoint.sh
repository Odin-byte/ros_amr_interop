#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /dev_ws/install/local_setup.bash

eval "$*"
