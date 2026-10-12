"""G57: quick summary of one HW run json: complete, makespan, stop, RESERVE-BLOCKED, top peak torques."""
import json
import sys
d = json.load(open(sys.argv[1]))
pt = sorted(d.get('peak_tau', {}).items(), key=lambda x: -abs(x[1]))[:3]
print('complete', d.get('complete'), 'makespan', d.get('makespan'), 'stop', d.get('stop'),
      'blocked', sum(e.get('n_blocked') or 0 for e in d['events']), 'peak', [(k, round(v, 2)) for k, v in pt])
