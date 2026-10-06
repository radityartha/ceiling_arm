"""K-RRn: return_rest with a cross-checker model that does NOT know t1_rotation_joint -> must REFUSE rc 1."""
import sys
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import interarm_collision as IC  # noqa: E402
import return_rest  # noqa: E402


class _M:
    def __init__(self, m):
        self._m = m

    def existJointName(self, n):
        return n != 't1_rotation_joint' and self._m.existJointName(n)

    def __getattr__(self, k):
        return getattr(self._m, k)


class Blind(IC.CrossGantryChecker):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.model = _M(self.model)


IC.CrossGantryChecker = Blind
sys.argv = ['return_rest.py'] + sys.argv[1:]
sys.exit(return_rest.main())
