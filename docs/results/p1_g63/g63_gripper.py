#!/usr/bin/env python3
"""G63: read / close ONE Gen3 Lite gripper directly over the Kortex API (no ROS stack). Only the fingers move.

Operator 2026-10-15: board taped across BOTH fingers -> gripper closed first, never commanded again during G63.
  g63_gripper.py --arm arm_2            # read-only: gripper position + arm fault/servoing state, NO motion
  g63_gripper.py --arm arm_2 --close    # HW MOTION (fingers): one operator approval per invocation
Refuses to command if the arm reports a fault. Prints position before/after; success = position >= 0.97 (closed).
"""
import argparse
import collections
import collections.abc
import sys
import time

# kortex_api ships protobuf 3.5 which still uses collections.MutableMapping (removed in Python 3.10)
for _n in ('MutableMapping', 'Mapping', 'MutableSequence', 'Sequence', 'MutableSet', 'Set', 'Iterable'):
    setattr(collections, _n, getattr(collections.abc, _n))

IPS = {'arm_1': '192.168.2.13', 'arm_2': '192.168.2.12', 'arm_3': '192.168.2.11', 'arm_4': '192.168.2.10'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', required=True, choices=sorted(IPS))
    ap.add_argument('--close', action='store_true')
    a = ap.parse_args()
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Base_pb2, Session_pb2

    t = TCPTransport()
    router = RouterClient(t, lambda e: print('router error', e))
    t.connect(IPS[a.arm], 10000)
    sm = SessionManager(router)
    ci = Session_pb2.CreateSessionInfo(username='admin', password='admin', session_inactivity_timeout=60000,
                                       connection_inactivity_timeout=2000)
    sm.CreateSession(ci)
    try:
        base, cyc = BaseClient(router), BaseCyclicClient(router)

        def pos():
            req = Base_pb2.GripperRequest(mode=Base_pb2.GRIPPER_POSITION)
            f = base.GetMeasuredGripperMovement(req).finger
            return f[0].value if f else float('nan')
        st = base.GetArmState()
        fb = cyc.RefreshFeedback()
        print(f'{a.arm} {IPS[a.arm]}: arm_state {Base_pb2.ArmState.Name(st.active_state)}, '
              f'fault_bank_a {fb.base.fault_bank_a}, fault_bank_b {fb.base.fault_bank_b}, gripper {pos():.3f} (0 buka, 1 tutup)')
        if not a.close:
            return 0
        if fb.base.fault_bank_a or fb.base.fault_bank_b or st.active_state == Base_pb2.ARMSTATE_IN_FAULT:
            print('TOLAK: lengan melaporkan fault -- tidak memerintah gripper')
            return 2
        cmd = Base_pb2.GripperCommand(mode=Base_pb2.GRIPPER_POSITION)
        finger = cmd.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = 1.0
        base.SendGripperCommand(cmd)
        t0 = time.time()
        p = pos()
        while time.time() - t0 < 6.0:
            time.sleep(0.3)
            p = pos()
            if p >= 0.97:
                break
        fb = cyc.RefreshFeedback()
        print(f'sesudah: gripper {p:.3f}, fault_bank_a {fb.base.fault_bank_a}, fault_bank_b {fb.base.fault_bank_b} -> '
              f'{"TERTUTUP" if p >= 0.97 else "BELUM TERTUTUP"}')
        return 0 if p >= 0.97 else 1
    finally:
        sm.CloseSession()
        t.disconnect()


if __name__ == '__main__':
    sys.exit(main())
