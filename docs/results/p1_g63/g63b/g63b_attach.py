#!/usr/bin/env python3
"""G63b attach: g63_attach.py (G63, TIDAK diubah) + upper_wrist_link sebagai touch link.

Rakitan G63b dimodelkan SATU kotak (dua lembar + ruang di antaranya, mount_g63b.json); tepi atas lembar 8 mm di atas flens
-> kotak beririsan housing pergelangan secara MODEL (lembar nyata di luar, menempel badan gripper). Tanpa touch link ini
MoveIt menilai setiap keadaan = tabrakan (START-JUMP). Hanya pasangan papan<->link lengan pembawa sendiri yang dibebaskan;
papan vs lingkungan / lengan lain tetap diperiksa penuh. Fase arm_2: --boards A saja (rakitan di arm_2).

    python3 g63b_attach.py --mount mount_g63b.json --boards A [--remove | --verify-only]
--remove juga menghapus objek DUNIA sisa detach dan memverifikasi scene bersih (gagal keras bila tersisa).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g63_attach  # noqa: E402

if 'upper_wrist_link' not in g63_attach.TOUCH:
    g63_attach.TOUCH = tuple(g63_attach.TOUCH) + ('upper_wrist_link',)



def purge_world(ids):
    """MoveIt detach leaves the box as a WORLD object at its last pose (mock 10-15: stale g63_board_A blocked every
    arm_3 plan with START-JUMP). Remove it from the world too, then VERIFY neither world nor attached has it."""
    import rclpy
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    rclpy.init()
    node = rclpy.create_node('g63b_attach_purge')
    try:
        ps = PlanningScene()
        ps.is_diff = True
        for i in ids:
            co = CollisionObject()
            co.id, co.operation = i, CollisionObject.REMOVE
            co.header.frame_id = 'world'
            ps.world.collision_objects.append(co)
        cli = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/apply_planning_scene tidak ada')
        f = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        if f.result() is None or not f.result().success:
            raise RuntimeError('hapus objek dunia GAGAL')
        g = node.create_client(GetPlanningScene, '/get_planning_scene')
        g.wait_for_service(timeout_sec=5.0)
        rq = GetPlanningScene.Request()
        rq.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        f = g.call_async(rq)
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        sc = f.result().scene
        left = sorted({o.id for o in sc.world.collision_objects} & set(ids)) + \
            sorted({a.object.id for a in sc.robot_state.attached_collision_objects} & set(ids))
        print(f'hapus dunia+terpasang {ids}: sisa {left} -> {"OK" if not left else "GAGAL"}')
        return 0 if not left else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rc = g63_attach.main()
    if rc == 0 and '--remove' in sys.argv:
        i = sys.argv.index('--boards')
        rest = sys.argv[i + 1:]
        boards = rest[:next((k for k, b in enumerate(rest) if b.startswith('-')), len(rest))]
        rc = purge_world([g63_attach.object_id(b) for b in boards])
    sys.exit(rc)
