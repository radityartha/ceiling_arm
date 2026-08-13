"""Board-free extrinsic sanity check: does each camera still put the FLOOR at z=0?

Deprojects each camera's live depth into `world` using its own camera_info and
the world->*_camera_optical TF that realsense_dual.launch.py is publishing right
now, then fits a plane to the lowest-lying points. If the extrinsics are still
good, that plane is z~0 and level. Needs no ChArUco board in view, so it can be
run any time. Baseline from the 2026-07-30 calibration: rgbd c=-0.035 m tilt<2deg,
rgbd2 c=-0.005 m tilt<2.3deg.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

NS = ['rgbd', 'rgbd2']


def quat_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Cap(Node):
    def __init__(self):
        super().__init__('floor_check')
        self.depth = {n: None for n in NS}
        self.info = {n: None for n in NS}
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        for n in NS:
            self.create_subscription(Image, f'/{n}/depth',
                                     lambda m, k=n: self.depth.__setitem__(k, m),
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{n}/camera_info',
                                     lambda m, k=n: self.info.__setitem__(k, m),
                                     qos_profile_sensor_data)


def main():
    rclpy.init()
    c = Cap()
    import time
    end = time.time() + 20
    while time.time() < end and not all(
            c.depth[n] is not None and c.info[n] is not None for n in NS):
        rclpy.spin_once(c, timeout_sec=0.1)
    # /tf_static is transient-local: a fresh listener needs a moment to receive the
    # latched transforms. Looking up immediately after the images arrive races it.
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(c, timeout_sec=0.05)

    clouds = {}
    for n in NS:
        if c.depth[n] is None or c.info[n] is None:
            print(f'{n}: NO DATA')
            continue
        d = c.depth[n]
        buf = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        K = np.array(c.info[n].k).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        step = 4
        ys, xs = np.mgrid[0:d.height:step, 0:d.width:step]
        z = buf[::step, ::step]
        m = np.isfinite(z) & (z > 0.3) & (z < 6.0)
        pts = np.stack([(xs[m] - cx) * z[m] / fx,
                        (ys[m] - cy) * z[m] / fy, z[m]], -1)
        frame = f'{n}_camera_optical'
        try:
            tf = c.buf.lookup_transform('world', frame, rclpy.time.Time())
        except Exception as e:
            print(f'{n}: no TF world->{frame}: {e}')
            continue
        t = tf.transform.translation
        r = tf.transform.rotation
        R = quat_mat(r.x, r.y, r.z, r.w)
        W = pts @ R.T + np.array([t.x, t.y, t.z])
        # Floor = the lowest 15% of points; fit a plane by least squares.
        thr = np.percentile(W[:, 2], 15)
        F = W[W[:, 2] <= thr]
        A = np.c_[F[:, 0], F[:, 1], np.ones(len(F))]
        coef, *_ = np.linalg.lstsq(A, F[:, 2], rcond=None)
        nrm = np.array([-coef[0], -coef[1], 1.0])
        nrm /= np.linalg.norm(nrm)
        tilt = np.degrees(np.arccos(abs(nrm[2])))
        print(f'{n:6s} n={len(W):7d} pts | cam height {t.z:.3f} m | '
              f'floor plane z0={coef[2]:+.3f} m, tilt {tilt:.2f} deg | '
              f'floor pts {len(F)}')
        clouds[n] = W

    # Cross-camera agreement: symmetric nearest-neighbour over the region BOTH
    # cameras see. Estimator-independent -- it compares the two cameras to each
    # other, not to an assumed floor. 2026-07-30 baseline: median 4.4 cm
    # (a broken calibration read ~36 cm).
    if len(clouds) == 2:
        from scipy.spatial import cKDTree
        A, B = clouds['rgbd'], clouds['rgbd2']
        lo = np.maximum(A.min(0), B.min(0))
        hi = np.minimum(A.max(0), B.max(0))
        def crop(P):
            m = np.all((P >= lo) & (P <= hi), axis=1)
            return P[m]
        A, B = crop(A), crop(B)
        print(f'\noverlap box {np.round(lo,2)} .. {np.round(hi,2)}  '
              f'| rgbd {len(A)} pts, rgbd2 {len(B)} pts')
        # The 2026-07-30 baseline was measured on SHARED FLOOR surfaces. The full
        # overlap box also contains the gantry/arms/objects, where one camera sees
        # a surface the other cannot -- that inflates NN for reasons unrelated to
        # calibration. Restrict to the floor to compare like with like.
        Af = A[np.abs(A[:, 2]) < 0.15]
        Bf = B[np.abs(B[:, 2]) < 0.15]
        if len(Af) > 100 and len(Bf) > 100:
            dab = cKDTree(Bf).query(Af)[0]
            dba = cKDTree(Af).query(Bf)[0]
            df = np.concatenate([dab, dba])
            print(f'FLOOR ONLY (|z|<0.15): {len(Af)}+{len(Bf)} pts, symmetric NN '
                  f'median {np.median(df)*100:.1f} cm, '
                  f'p75 {np.percentile(df,75)*100:.1f} cm')
        if len(A) > 100 and len(B) > 100:
            dab = cKDTree(B).query(A)[0]
            dba = cKDTree(A).query(B)[0]
            d = np.concatenate([dab, dba])
            print(f'symmetric NN: median {np.median(d)*100:.1f} cm, '
                  f'p25 {np.percentile(d,25)*100:.1f} cm, '
                  f'p75 {np.percentile(d,75)*100:.1f} cm')
            print('BASELINE 2026-07-30: median 4.4 cm | broken-calib ref: ~36 cm')
    c.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
