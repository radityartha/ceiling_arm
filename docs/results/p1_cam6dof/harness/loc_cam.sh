#!/usr/bin/env bash
# CAM-6DOF: object_localizer for ONE camera (no fusion) -> /loc_<ns>/{detected_objects,tracks,target_object,markers,
# target_collision_boxes}; /grasp_target input isolated too. Node loc_<ns>. Runs the entry script itself (exec, no `ros2 run`
# wrapper) so the PID printed by `$!`/ps IS the node: SIGINT that PID.   loc_cam.sh rgbd2
ns=${1:?ns}; source /home/user1/Documents/ceiling_arm/docs/results/p1_demo_hw/harness/env.sh; trap - INT
R="-r __node:=loc_$ns"
for s in "" _labels _conf; do R="$R -r /$ns/instance_segmentation$s:=/$ns/seg/instance_segmentation$s"; done
for t in detected_objects:detected_objects detected_objects/tracks:tracks target_object:target_object \
         detected_objects/markers:markers target_collision_boxes:target_collision_boxes grasp_target:grasp_target; do
  R="$R -r /${t%%:*}:=/loc_$ns/${t#*:}"; done
exec /home/user1/Documents/ceiling_arm/ros2_ws/install/reachability_gng/lib/reachability_gng/object_localizer \
  --ros-args $R -p "camera_namespaces:=['$ns']"
