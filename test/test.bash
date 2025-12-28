#!/bin/bash 
# SPDX-FileCopyrightText: 2025 Hiroto Miura
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"   #引数があったら、そちらをホームに変える。            　
cd $dir/ros2_ws
colcon build
source $dir/.bashrc 　
timeout 15 ros2 launch mypkg pitchserver_client.launch.py > /tmp/mypkg.log

cat /tmp/mypkg.log |
grep 'Listen: 10'

