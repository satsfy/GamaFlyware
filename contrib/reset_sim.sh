#!/bin/bash
set -e
pkill -9 -f px4 || true
pkill -9 -f 'gz sim' || true
pkill -9 -f gazebo || true
pkill -9 -f 'ruby.*gz' || true
pkill -9 -f ign || true
rm -f /tmp/px4-sock-* /tmp/px4-*.lock
sleep 1
ps -ef | grep -iE 'px4|gz|gazebo|ruby|ign' | grep -v grep || true
