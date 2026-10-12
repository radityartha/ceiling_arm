#!/usr/bin/env bash
# bag.sh <out_dir> : 20 s light perception bag (no images).
source $(dirname $0)/env.sh
timeout -s INT 20 ros2 bag record -o $1 /detected_objects/tracks /detected_objects /target_object /tf_static \
  /rgbd/seg/instance_segmentation_labels /rgbd2/seg/instance_segmentation_labels /rgbd/seg/instance_segmentation_conf /rgbd2/seg/instance_segmentation_conf
