#!/usr/bin/env bash

set -e

xargs sudo apt-get install  -y < ros-noetic-packages-name-only.txt
