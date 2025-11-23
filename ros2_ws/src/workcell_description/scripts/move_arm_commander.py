import rclpy
import sys
import rclpy.duration
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


class MoveArmNode(Node):
    def __init__(self):
        super().__init__("move_arm_commander_node")
        self.logger = self.get_logger()

    def run(self):
        # --- 1. Initialize MoveIt Commander ---
        # We need to initialize this first for the MoveGroupCommander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot commander
        robot = moveit_commander.RobotCommander(robot_description="robot_description")

        # Initialize the planning scene
        scene = PlanningSceneInterface(synchronous=True)

        # Get the planning group for the arm
        self.logger.info("Initializing MoveGroupCommander for 'arm_1'...")
        arm_1 = MoveGroupCommander("arm_1", robot_description="robot_description")

        # Allow some time for things to initialize
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # --- 2. Set a New Goal Pose ---
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.header.stamp = self.get_clock().now().to_msg()

        # Set a simple target position (in meters)
        # BE CAREFUL: Make sure this is a safe, reachable pose!
        self.logger.info("Setting target pose...")
        pose_goal.pose.position.x = 0.5
        pose_goal.pose.position.y = 0.3
        pose_goal.pose.position.z = 0.5

        # Set a simple orientation (pointing straight down)
        pose_goal.pose.orientation.x = 1.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0

        arm_1.set_pose_target(pose_goal, end_effector_link="t1_a1_end_effector_link")

        # --- 3. Plan and Execute ---
        self.logger.info("Planning and executing...")

        # plan() returns a motion plan. go() plans and executes.
        # We'll use go() for simplicity.
        success = arm_1.go(wait=True)

        if success:
            self.logger.info("Movement executed successfully!")
        else:
            self.logger.error("Movement failed!")

        # --- 4. Shutdown ---
        arm_1.stop()
        arm_1.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():
    rclpy.init()

    # Create and run the node
    node = MoveArmNode()

    # Use a try-except block to catch shutdown exceptions
    try:
        node.run()
    except rclpy.exceptions.ROSInterruptException:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
