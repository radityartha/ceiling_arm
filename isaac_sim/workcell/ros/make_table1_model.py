#!/usr/bin/env python3
"""Generate a TABLE_1-ONLY robot_description + SRDF for the Isaac GNG view.

table_2 / arm_3 / arm_4 cannot be reliably hidden in RViz (the MotionPlanning
display renders the whole robot regardless of per-link visibility), so for the
clean GNG view we give move_group a model that simply does not contain them.

It filters the EXPANDED sim_isaac workcell model (topic_based ros2_control, on
/isaac_joint_commands /isaac_joint_states, ceiling mount, ±180 table rotation)
and the full SRDF, dropping every element under the `t2_` prefix:

  table1_isaac.urdf   <- workcell.urdf.xacro sim_isaac:=true, minus all t2_*
  trailer_table1.srdf <- trailer_workcell.srdf, minus arm_3/4, table_2, etc.

Re-run after URDF/SRDF edits:
  source /opt/ros/humble/setup.bash
  source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
  source /srv/data/users/raditya/workcell_overlay_ws/install/setup.bash
  python3 isaac_sim/workcell/ros/make_table1_model.py
"""
import os
import subprocess
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = get_package_share_directory("workcell_description")
MOVEIT = get_package_share_directory("workcell_moveit_config")
WORKCELL_XACRO = os.path.join(DESC, "urdf", "workcell.urdf.xacro")
FULL_SRDF = os.path.join(MOVEIT, "config", "trailer_workcell.srdf")

OUT_URDF = os.path.join(HERE, "table1_isaac.urdf")
OUT_SRDF = os.path.join(HERE, "trailer_table1.srdf")

DROP = "t2_"   # everything table_2 / arm_3 / arm_4 is under this prefix


def _has(s):
    return s is not None and DROP in s


def filter_urdf():
    xml = subprocess.check_output(
        ["xacro", WORKCELL_XACRO, "sim_isaac:=true", "use_fake_hardware:=false"],
        text=True)
    root = ET.fromstring(xml)
    for el in list(root):
        tag, name = el.tag, el.get("name", "")
        if tag in ("link", "joint") and DROP in name:
            root.remove(el); continue
        if tag == "gazebo" and _has(el.get("reference")):
            root.remove(el); continue
        if tag == "joint":
            p = el.find("parent"); c = el.find("child")
            if _has(p.get("link") if p is not None else None) or \
               _has(c.get("link") if c is not None else None):
                root.remove(el); continue
        if tag == "ros2_control":
            joints = el.findall("joint")
            jnames = [j.get("name", "") for j in joints]
            if joints and all(DROP in n for n in jnames):
                root.remove(el); continue          # pure-t2 system (arm/gripper)
            for j in joints:                         # mixed table system: drop t2 joints
                if DROP in j.get("name", ""):
                    el.remove(j)
    ET.ElementTree(root).write(OUT_URDF, xml_declaration=True, encoding="unicode")
    return OUT_URDF


def filter_srdf():
    root = ET.fromstring(open(FULL_SRDF).read())
    for el in list(root):
        tag = el.tag
        if tag == "group":
            # drop groups that ARE a t2 group or reference t2 joints/subgroups
            if DROP in el.get("name", ""):
                root.remove(el); continue
            if any(DROP in (j.get("name", "")) for j in el.findall("joint")):
                root.remove(el); continue
            if any(g.get("name", "") in ("arm_3", "arm_4", "table_2")
                   for g in el.findall("group")):
                root.remove(el); continue
        elif tag == "group_state":
            if any(DROP in j.get("name", "") for j in el.findall("joint")):
                root.remove(el); continue
        elif tag == "end_effector":
            if _has(el.get("parent_link")) or el.get("group", "") in ("gripper_3", "gripper_4"):
                root.remove(el); continue
        elif tag == "disable_collisions":
            if _has(el.get("link1")) or _has(el.get("link2")):
                root.remove(el); continue
    ET.ElementTree(root).write(OUT_SRDF, xml_declaration=True, encoding="unicode")
    return OUT_SRDF


if __name__ == "__main__":
    u = filter_urdf()
    s = filter_srdf()
    # quick report
    ur = ET.parse(u).getroot(); sr = ET.parse(s).getroot()
    links = [l.get("name") for l in ur.findall("link")]
    groups = [g.get("name") for g in sr.findall("group")]
    print(f"wrote {u}: {len(links)} links, t2 present={any(DROP in l for l in links)}")
    print(f"wrote {s}: groups={groups}")
