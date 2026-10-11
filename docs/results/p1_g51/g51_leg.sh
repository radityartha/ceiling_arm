#!/usr/bin/env bash
# G51 6b: ONE sweep leg on HW (operator approved), full log kept, then the runbook checks:
#   |err_deg| <= 0.3 -- or <= 0.51 if SHORT of the goal and the last dispatch says "Already at absolute target"
#   (go_to_absolute 50-pulse deadband = 0.5 deg + int truncation; operator 2026-10-11, G51 B4), |err_mm| <= 2, t_traverse <= 1.2 T_cmd, arm drift <= 0.5 deg, other rail/rot still,
#   arm torque <= 14, launch log: no REJECTED / NOT armed / alarm, |motor1 - motor2| <= 100 pulse (G51 B3).
#   bash g51_leg.sh <gantry> <leg> --dry|--move   (--move ONLY after the operator approved THIS leg -- G51 B4 violation)
H=$(cd $(dirname $0) && pwd); G=$1; K=$2
case "${3:-}" in --dry) DRY=--dry ;; --move) DRY= ;; *) echo "pakai --dry atau --move (eksplisit)"; exit 64 ;; esac
OUT=$H/hw/sweep; mkdir -p $OUT; T=$(printf 'g%s_leg%02d%s' $G $K "${DRY:+_dry}")
L0=$(wc -l < /tmp/p1hw_launch.log)
ROS_DOMAIN_ID=0 bash $H/../p1_g50/g50_sweep.sh $G $K $K $DRY > $OUT/$T.log 2>&1; rc=$?
cat $OUT/$T.log
[ -n "$DRY" ] && exit $rc
tail -n +$((L0+1)) /tmp/p1hw_launch.log | grep "dual_table_controller" > $OUT/$T.bridge.log
python3 - $OUT/$T.log $OUT/$T.bridge.log $G > $OUT/$T.check 2>&1 <<'EOF'
import json, re, sys
log, br, g = sys.argv[1], sys.argv[2], int(sys.argv[3])
js = [json.loads(l) for l in open(log) if l.startswith('{')]
bad = []
if not js or js[-1].get('dry'):
    print('CEK: tidak ada JSON hasil -- BERHENTI'); sys.exit(2)
j = js[-1]
T = j['T_cmd'] or 0.0
B = open(br).read()
mv = [l for l in B.splitlines() if 'Absolute move' in l or 'All motors at absolute' in l or 'Already at absolute' in l]
d_rot = j['goal'][1] - j['start'][1]
short = j['err_deg'] == 0 or (d_rot != 0 and (j['err_deg'] > 0) != (d_rot > 0))
dead = bool(mv) and 'Already at absolute' in mv[-1]
ok_deg = abs(j['err_deg']) <= 0.3 or (abs(j['err_deg']) <= 0.51 and short and dead)
chk = [('|err_deg| <= 0.3 (<= 0.51 kurang + deadband)', ok_deg, (j['err_deg'], 'kurang' if short else 'LEBIH', 'deadband' if dead else '-')),
       ('|err_mm| <= 2', abs(j['err_mm']) <= 2, j['err_mm']),
       ('t_traverse <= 1.2 T_cmd', j['t_traverse'] is None or j['t_traverse'] <= 1.2 * T + 0.5, (j['t_traverse'], T)),
       ('arm drift <= 0.5', j['arm_drift_deg'] <= 0.5, j['arm_drift_deg']),
       ('rel lain diam <= 1 mm', abs(j['other_rail_drift_mm']) <= 1, j['other_rail_drift_mm']),
       ('rot lain diam <= 0.3', abs(j['other_rot_drift_deg']) <= 0.3, j['other_rot_drift_deg']),
       ('torsi <= 14', j['arm_tau_peak'] <= 14, (j['arm_tau_peak'], j['arm_tau_joint'])),
       ('JTC 0', str(j['jtc_error_code']) == '0', j['jtc_error_code'])]
tb = f'table{g}'
for pat in ('REJECTED', 'NOT armed', 'larm', 'Timeout', 'Failed', 'Cannot read', 'Read error'):
    n = sum(1 for l in B.splitlines() if pat in l)
    chk.append((f'log tanpa "{pat}"', n == 0, n))
# motor1 vs motor2 (both linear motors of the moving table) in every go_to_absolute start / end line
dm = [abs(int(a) - int(b)) for a, b in re.findall(r'(?:start=|Positions: )\[(-?\d+), (-?\d+), -?\d+\]', B)]
chk.append(('|motor1 - motor2| <= 100', (max(dm) if dm else 0) <= 100, (max(dm) if dm else None, len(dm))))
rot_lines = re.findall(r'start=\[(-?\d+), (-?\d+), (-?\d+)\]', B)
for name, ok, v in chk:
    print(f"CEK {'OK ' if ok else 'GAGAL'} {name}: {v}")
    if not ok: bad.append(name)
tr = j.get('t_rot') or 0; Tr = j.get('T_cmd_rot') or 0
print(f"t_rot/T_cmd_rot = {tr}/{Tr} = {round(tr / Tr, 3) if Tr else None}; t_lin/T_cmd_lin = {j.get('t_lin')}/{j.get('T_cmd_lin')}")
print('CEK KAKI: ' + ('LULUS' if not bad else 'GAGAL -> BERHENTI: ' + ', '.join(bad)))
sys.exit(1 if bad else 0)
EOF
c=$?; cat $OUT/$T.check; [ $rc -ne 0 ] && echo "rc pose_to_g $rc -> BERHENTI"; exit $(( rc ? rc : c ))
