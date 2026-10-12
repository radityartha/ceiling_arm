# DEMO-HW (docs/p1_demo_hw.md A0): domain 0 = HW stack. DEMO_DOMAIN=79 ONLY for the camera check with the stack off (step 1).
source /opt/ros/humble/setup.bash
source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${DEMO_DOMAIN:-0} DISPLAY=:1 PYTHONUNBUFFERED=1
