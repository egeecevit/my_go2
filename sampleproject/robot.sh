#!/bin/bash

export CFG=$QUADCONTROL_DIR/config
export CONFIG_DIR=$CFG/default
export VERSION_DIR=$CFG/versions/robotv1
export ROBOT_DIR=$CFG/robots/robotv1r1
build/robot "$@"
