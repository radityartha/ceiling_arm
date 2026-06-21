"""1-DOF software coupling for the Gen3 Lite 2F gripper.

The kortex hardware URDF leaves the 4 finger joints independent. PhysX hard-mimic
joints are unreliable in this Isaac build, so we couple them in software: command a
single value (the master, right_finger_bottom_joint) and drive the other 3 from the
real relationships taken from gen3_lite_2f_transmission_macro.xacro:

    right_finger_tip   = -0.676*master + 0.149
    left_finger_bottom = -1.0  *master + 0.0
    left_finger_tip    = -0.676*master + 0.149

Direction (verified empirically in sim): increasing the master joint OPENS the
gripper. master = 0.96 -> wide open (pad gap ~0.108 m); master = -0.09 -> closed.
"""

MASTER = "right_finger_bottom_joint"
# suffix -> (multiplier, offset)  (suffix match supports prefixed workcell joints)
MIMIC = {
    "right_finger_tip_joint": (-0.676, 0.149),
    "left_finger_bottom_joint": (-1.0, 0.0),
    "left_finger_tip_joint": (-0.676, 0.149),
}
OPEN = 0.96
CLOSED = -0.09


def gripper_indices(dof_names):
    """Return {master_idx: [(mimic_idx, mult, off), ...]} for every gripper in the
    articulation, grouped by prefix. Works for 1 gripper (single arm) or 4 (workcell)."""
    groups = {}
    for i, nm in enumerate(dof_names):
        if nm.endswith(MASTER):
            prefix = nm[: -len(MASTER)]
            groups[prefix] = {"master": i, "mimics": []}
    for i, nm in enumerate(dof_names):
        for suffix, (mult, off) in MIMIC.items():
            if nm.endswith(suffix):
                prefix = nm[: -len(suffix)]
                if prefix in groups:
                    groups[prefix]["mimics"].append((i, mult, off))
    return groups


def apply_gripper(target_positions, groups, value_by_prefix):
    """Write master + coupled mimic targets into the target_positions array.
    value_by_prefix: {prefix: master_value} or a single float applied to all grippers."""
    for prefix, g in groups.items():
        if isinstance(value_by_prefix, dict):
            v = value_by_prefix.get(prefix)
            if v is None:
                continue
        else:
            v = float(value_by_prefix)
        target_positions[g["master"]] = v
        for idx, mult, off in g["mimics"]:
            target_positions[idx] = mult * v + off
    return target_positions
