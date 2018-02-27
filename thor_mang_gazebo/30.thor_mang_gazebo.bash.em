#!/bin/bash

export GAZEBO_MODEL_PATH=$(cd "@(CMAKE_SOURCE_DIR)"; pwd)/models:$GAZEBO_MODEL_PATH
