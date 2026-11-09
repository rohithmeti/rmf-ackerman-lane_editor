#!/usr/bin/env python3
"""Find where the simulator's origin is on YOUR map ("the 0 of the environment").

Your map (from SLAM / a floor plan) and the simulator world use different coordinate frames. Give this tool
  --map        your occupancy map (.yaml + .pgm)                          (frame A, "map frame")
  --world-map  an occupancy map of the SIMULATOR world, in the world frame (frame B). Make it from the running simulator with
               agv_tag_nav/scripts/gz_floorplan.py (it scans the world with the robot's lidar at known poses, no SLAM drift)
and it finds the rigid transform  world = R(yaw) * map + (dx, dy)  that lays your map exactly over the world map.
Output: registration.json  ->  load it in the AprilTag Graph Maker (section "Simulator frame"), or pass it to export_gazebo.py.

How: occupied cells of both maps -> coarse search over yaw with an FFT cross-correlation (finds the translation for each yaw),
then a fine chamfer-distance refinement. Works for any rotation; the world map may cover only part of the world.

  python3 register_map_to_world.py --map testmap/warehouse_slam.yaml --world-map sim_warehouse.yaml --out registration.json
"""
import argparse, json, math, os
import numpy as np


def load_map(yaml_path):
    meta = {}
    for line in open(yaml_path):
        line = line.split('#')[0].strip()
        if ':' in line:
            k, v = line.split(':', 1); meta[k.strip()] = v.strip()
    res = float(meta['resolution']); org = [float(x) for x in meta['origin'].strip('[]').split(',')]
    img = open(os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'].strip('"\'')), 'rb').read()
    toks, pos = [], 0
    while len(toks) < 4:
        while img[pos:pos + 1].isspace(): pos += 1
        if img[pos:pos + 1] == b'#':
            while img[pos:pos + 1] != b'\n': pos += 1
            continue
        s = pos
        while not img[pos:pos + 1].isspace(): pos += 1
        toks.append(img[s:pos])
    pos += 1
    w, h = int(toks[1]), int(toks[2])
    gray = np.frombuffer(img, np.uint8, offset=pos, count=w * h).reshape(h, w)
    if int(float(meta.get('negate', 0))): gray = 255 - gray
    return gray, res, org[0], org[1]


def occupied_points(gray, res, ox, oy, thresh=50):
    rows, cols = np.nonzero(gray <= thresh)
    h = gray.shape[0]
    return np.stack([ox + (cols + 0.5) * res, oy + (h - 1 - rows + 0.5) * res], 1)


def rot(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def raster(pts, x0, y0, res, shape):
    img = np.zeros(shape, np.float32)
    ix = np.floor((pts[:, 0] - x0) / res).astype(int); iy = np.floor((pts[:, 1] - y0) / res).astype(int)
    ok = (ix >= 0) & (ix < shape[1]) & (iy >= 0) & (iy < shape[0])
    np.add.at(img, (iy[ok], ix[ok]), 1.0)
    return np.minimum(img, 1.0)


def register_points(A, B, yaw_range=(-180, 180), yaw_step=2.0, coarse_res=0.2, verbose=True):
    """A, B: (N,2) occupied points in frame A (map) and B (world). Returns dict(dx, dy, yaw_deg, ...) with world = R*map + t."""
    from scipy.ndimage import gaussian_filter, distance_transform_edt
    from scipy.signal import correlate
    rng = np.random.default_rng(0)
    sub = lambda P, n: P if len(P) <= n else P[rng.choice(len(P), n, replace=False)]
    Ac, Bc = sub(A, 20000), sub(B, 20000)
    extA = Ac.max(0) - Ac.min(0)
    # B grid padded by the size of A so that every shift is a 'valid' correlation position
    bx0, by0 = Bc.min(0) - extA - 1.0
    bx1, by1 = Bc.max(0) + extA + 1.0
    shape = (int((by1 - by0) / coarse_res) + 1, int((bx1 - bx0) / coarse_res) + 1)
    Bimg = gaussian_filter(raster(Bc, bx0, by0, coarse_res, shape), 1.5)
    best = []
    yaws = np.arange(yaw_range[0], yaw_range[1], yaw_step)
    for yaw in yaws:
        P = Ac @ rot(yaw).T; cA = P.mean(0); Q = P - cA
        qmin = Q.min(0); wa = int((Q[:, 0].max() - qmin[0]) / coarse_res) + 2; ha = int((Q[:, 1].max() - qmin[1]) / coarse_res) + 2
        if ha > shape[0] or wa > shape[1]: continue
        Aimg = raster(Q, qmin[0], qmin[1], coarse_res, (ha, wa))
        C = correlate(Bimg, Aimg, mode='valid', method='fft')
        iy, ix = np.unravel_index(np.argmax(C), C.shape)
        # A pixel (0,0) sits at B pixel (iy, ix); A pixel (0,0) <-> Q point qmin
        tx = bx0 + ix * coarse_res - qmin[0] - cA[0]; ty = by0 + iy * coarse_res - qmin[1] - cA[1]
        best.append((float(C[iy, ix]), float(yaw), tx, ty))
    best.sort(reverse=True)
    if verbose: print(f'coarse search: {len(yaws)} yaws, best score {best[0][0]:.0f} at yaw {best[0][1]:.1f}, runner-up {best[1][0]:.0f} at yaw {best[1][1]:.1f}')

    # fine: chamfer distance from the world points to the (map) occupied cells, searched around the best coarse candidates
    fres = 0.05
    ax0, ay0 = A.min(0) - 3.0; ax1, ay1 = A.max(0) + 3.0
    ashape = (int((ay1 - ay0) / fres) + 1, int((ax1 - ax0) / fres) + 1)
    dtA = distance_transform_edt(raster(A, ax0, ay0, fres, ashape) == 0) * fres
    Bsub = sub(B, 5000)

    def cost(yaw, tx, ty, cap=0.5):
        a = (Bsub - np.array([tx, ty])) @ rot(yaw)                      # world -> map: R(-yaw) (b - t)  ==  (b - t) @ R(yaw)
        ix = np.floor((a[:, 0] - ax0) / fres).astype(int); iy = np.floor((a[:, 1] - ay0) / fres).astype(int)
        ok = (ix >= 0) & (ix < ashape[1]) & (iy >= 0) & (iy < ashape[0])
        d = np.full(len(a), cap); d[ok] = np.minimum(dtA[iy[ok], ix[ok]], cap)
        return float(d.mean()), d

    result = None
    for score, yaw0, tx0, ty0 in best[:4]:
        cur = (cost(yaw0, tx0, ty0)[0], yaw0, tx0, ty0)
        for span, step, ystep, ya in ((0.6, 0.1, 0.5, 2.0), (0.15, 0.025, 0.1, 0.5), (0.04, 0.01, 0.02, 0.1)):
            c0 = cur
            for dy in np.arange(-ya, ya + 1e-9, ystep):
                for dx in np.arange(-span, span + 1e-9, step):
                    for dz in np.arange(-span, span + 1e-9, step):
                        c = cost(c0[1] + dy, c0[2] + dx, c0[3] + dz)[0]
                        if c < cur[0]: cur = (c, c0[1] + dy, c0[2] + dx, c0[3] + dz)
        if result is None or cur[0] < result[0]: result = cur
    c, yaw, tx, ty = result
    _, d = cost(yaw, tx, ty)
    return {'dx': tx, 'dy': ty, 'yaw_deg': yaw, 'chamfer_mean_m': c, 'median_m': float(np.median(d)),
            'inlier_fraction_15cm': float((d < 0.15).mean()), 'rms_m': float(np.sqrt((d ** 2).mean())), 'points_used': int(len(Bsub))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True); ap.add_argument('--world-map', required=True); ap.add_argument('--out', default='registration.json')
    ap.add_argument('--yaw-range', type=float, nargs=2, default=[-180, 180], metavar=('MIN', 'MAX'), help='degrees to search (default: all)')
    ap.add_argument('--yaw-step', type=float, default=2.0)
    a = ap.parse_args()
    A = occupied_points(*load_map(a.map)); B = occupied_points(*load_map(a.world_map))
    print(f'map: {len(A)} occupied cells   world map: {len(B)} occupied cells')
    r = register_points(A, B, tuple(a.yaw_range), a.yaw_step)
    r.update({'map': os.path.basename(a.map), 'world_map': os.path.basename(a.world_map), 'definition': 'world = R(yaw_deg) * map + (dx, dy)'})
    json.dump(r, open(a.out, 'w'), indent=1)
    print(f"world = R({r['yaw_deg']:.2f} deg) * map + ({r['dx']:.3f}, {r['dy']:.3f})   fit: median {r['median_m']*100:.1f} cm, rms {r['rms_m']*100:.1f} cm, "
          f"{r['inlier_fraction_15cm']*100:.0f}% of world points within 15 cm of your map")
    if r['inlier_fraction_15cm'] < 0.5: print('WARNING: weak match (< 50% inliers): check that both maps show the same place, or widen the world map.')
    print('-> saved', a.out)


if __name__ == '__main__':
    main()
