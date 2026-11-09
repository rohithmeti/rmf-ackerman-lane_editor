"""synthetic test of register_map_to_world: known transform, partial + noisy world map -> must be recovered.   python3 tests/test_register.py"""
import math, os, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import register_map_to_world as R

A = R.occupied_points(*R.load_map(os.path.join(os.path.dirname(__file__), '..', 'testmap', 'warehouse_slam.yaml')))
rng = np.random.default_rng(1)
fails = 0
for dx, dy, yaw, keep in ((13.55, -10.0, 0.0, 0.6), (-7.3, 21.4, 7.0, 0.5), (3.0, 4.0, -123.0, 0.6)):
    B = A @ R.rot(yaw).T + np.array([dx, dy])
    B = B[rng.random(len(B)) < keep]                                                  # only a part of the points was seen
    B = B + rng.normal(0, 0.03, B.shape)
    B = B[(B[:, 0] < B[:, 0].min() + 0.8 * (B[:, 0].max() - B[:, 0].min()))]          # partial coverage: drop the last 20 % in x
    t0 = time.time(); r = R.register_points(A, B, verbose=False); dt = time.time() - t0
    e_t = math.hypot(r['dx'] - dx, r['dy'] - dy); e_y = abs((r['yaw_deg'] - yaw + 180) % 360 - 180)
    ok = e_t < 0.06 and e_y < 0.5
    print(f"true ({dx:6.2f},{dy:6.2f},{yaw:7.1f} deg)  found ({r['dx']:6.2f},{r['dy']:6.2f},{r['yaw_deg']:7.2f} deg)  error {e_t*100:4.1f} cm {e_y:4.2f} deg  inliers {r['inlier_fraction_15cm']*100:3.0f}%  {dt:4.0f}s  {'ok' if ok else 'FAIL'}")
    fails += not ok
sys.exit(1 if fails else 0)
