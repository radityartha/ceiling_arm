#!/usr/bin/env python3
"""G63: attach the taped boards to MoveIt's planning scene (AttachedCollisionObject), or remove them, and VERIFY.

MoveIt's own robot model has no board, so without this OMPL plans the arm through its own board / the env. The box
is the SAME as the payload URDF collision box (g63_urdf.board_box, mount JSON) -> MoveIt and the shared screens agree.
touch_links = the gripper links of that arm (the board is taped to a finger and rests against the gripper).

    python3 g63_attach.py --mount mount.json            # attach A and B, verify via /get_planning_scene
    python3 g63_attach.py --mount mount.json --remove   # detach + remove, verify gone
    python3 g63_attach.py --verify-only --boards A B    # rc 0 only if all named boards are attached
"""
import argparse
import json
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from g63_common import ARM_PREFIX, BOARD_ARM
from g63_urdf import board_box

TOUCH = ('gripper_base_link', 'left_finger_prox_link', 'left_finger_dist_link', 'right_finger_prox_link',
         'right_finger_dist_link', 'end_effector_link', 'tool_frame')


def object_id(board):
    return f'g63_board_{board}'


def attached_msg(board, mt, remove=False):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    pre = ARM_PREFIX[BOARD_ARM[board]]
    aco = AttachedCollisionObject()
    aco.link_name = pre + mt['link']
    aco.object.id = object_id(board)
    aco.object.header.frame_id = pre + mt['link']
    if remove:
        aco.object.operation = CollisionObject.REMOVE
        return aco
    c, s = board_box(mt)
    T_lb = np.eye(4)
    T_lb[:3, :3] = Rotation.from_euler('xyz', mt['rpy']).as_matrix()
    T_lb[:3, 3] = mt['xyz']
    p = T_lb[:3, :3] @ np.array(c) + T_lb[:3, 3]
    q = Rotation.from_matrix(T_lb[:3, :3]).as_quat()
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in s])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
    aco.object.primitives = [prim]
    aco.object.primitive_poses = [pose]
    aco.object.operation = CollisionObject.ADD
    aco.touch_links = [pre + t for t in TOUCH]
    return aco


def attached_ids(node):
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    cli = node.create_client(GetPlanningScene, '/get_planning_scene')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('/get_planning_scene tidak ada')
    rq = GetPlanningScene.Request()
    rq.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    f = cli.call_async(rq)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    if f.result() is None:
        raise RuntimeError('/get_planning_scene tidak menjawab')
    return {a.object.id: a.link_name for a in f.result().scene.robot_state.attached_collision_objects}


def apply(node, acos):
    import rclpy
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    cli = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('/apply_planning_scene tidak ada')
    ps = PlanningScene()
    ps.is_diff = True
    ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects = acos
    f = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    if f.result() is None or not f.result().success:
        raise RuntimeError('apply_planning_scene GAGAL')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mount')
    ap.add_argument('--boards', nargs='+', default=['A', 'B'])
    ap.add_argument('--remove', action='store_true')
    ap.add_argument('--verify-only', action='store_true')
    a = ap.parse_args()
    import rclpy
    rclpy.init()
    node = rclpy.create_node('g63_attach')
    try:
        if not a.verify_only:
            mount = json.load(open(a.mount))
            apply(node, [attached_msg(b, mount[b], a.remove) for b in a.boards])
            time.sleep(0.5)
        ids = attached_ids(node)
        want = {object_id(b) for b in a.boards}
        if a.remove:
            ok = not (want & set(ids))
        else:
            ok = want <= set(ids)
        print(f'attached di scene: {ids} -> {"OK" if ok else "GAGAL"}')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
