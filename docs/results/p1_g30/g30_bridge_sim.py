"""G30 A0: crude simulation of the bridge chase for ONE rotation leg (pre-lock instrument, not data).
Model (from code, dual_table_controller._on_hw_command + moving_table.go_to_absolute):
  command callback every DT; dispatch if |sp - last_dispatched| >= 0.3 deg and no thread in flight;
  go_to_absolute: overhead T_OVH, skip (return, no motion) if |pos - tgt| <= 0.5 deg (50 pulses);
  else motor starts after dead time T_DEAD, moves at 10 deg/s to tgt; thread returns when |pos - tgt| <= 0.5
  (polled every 0.1 s). Motor keeps going to its last started target.
Unknowns swept: T_OVH in {0.05, 0.16, 0.3}, DT in {0.01, 0.02}."""
import itertools
import math


def leg(a, b, T, t_ovh, dt, t_dead=0.10, v=10.0):
    sp = lambda t: a + (b - a) * 0.5 * (1 - math.cos(math.pi * min(t, T) / T))
    pos, motor_tgt, motor_t0 = a, a, None
    last, thread_until, thread_tgt = a, -1.0, None
    t, first, stop = 0.0, None, None
    while t < T + 10.0:
        # motor
        if motor_t0 is not None and t >= motor_t0 and abs(motor_tgt - pos) > 1e-9:
            step = math.copysign(min(v * dt, abs(motor_tgt - pos)), motor_tgt - pos)
            pos += step
            if first is None:
                first = t
            stop = t + dt
        # thread
        if thread_tgt is not None and t >= thread_until:
            if abs(pos - thread_tgt) <= 0.5:
                thread_tgt = None
            else:
                thread_until = t + 0.1
        # callback
        s = sp(t)
        if thread_tgt is None and abs(s - last) >= 0.3:
            last = s
            if abs(pos - s) <= 0.5:
                pass                                    # "Already at absolute target" -> no motion
            else:
                motor_tgt, motor_t0 = s, t + t_ovh + t_dead
                thread_tgt, thread_until = s, t + t_ovh + 0.1
        t += dt
    return pos - b, first, stop


for a, b in ((0, 10), (10, 0), (0, -10), (-10, 0)):
    for t_ovh, dt in itertools.product((0.05, 0.16, 0.3), (0.01, 0.02)):
        err, f, s = leg(a, b, 3.0, t_ovh, dt)
        print(f'{a:+3d}->{b:+3d} ovh {t_ovh:.2f} dt {dt:.2f}: galat akhir {err:+.3f} deg '
              f'(searah gerak {"KURANG" if err * (b - a) < 0 else "lebih/0"}), t_rot {s - f:.2f} s')
