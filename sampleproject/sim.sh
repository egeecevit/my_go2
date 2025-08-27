#!/bin/bash

export CFG=$QUADCONTROL_DIR/config
export CONFIG_DIR=$CFG/default
export VERSION_DIR=$CFG/versions/sim
export ROBOT_DIR=$CFG/robots/sim
build/simulation "$@"
