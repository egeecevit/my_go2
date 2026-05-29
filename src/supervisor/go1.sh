#!/bin/bash

export CFG=$PWD/../config
export CONFIG_DIR=$CFG/default
export VERSION_DIR=$CFG/versions/go1
export ROBOT_DIR=$CFG/robots/go1r1
./go1 "$@"
