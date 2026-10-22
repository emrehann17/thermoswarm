#!/usr/bin/env python3
"""ThermoSwarm — read-only Gazebo world sanity check (SITL only).

Verifies, BEFORE any flight command is sent, that the running Gazebo world
contains exactly the expected X500 models and that no two vehicles share the
same physical spawn location.

Background (2026-10-19 failure): an orphan `x500_0` model from an earlier
single-vehicle session stayed in the still-running Gazebo server. `px4 -i 1`
re-used that server and spawned `x500_1` at the same pose (0,0). PX4-1 then
armed, entered Offboard and saturated all motors (thrust_z=-1.0) for ~35 s,
but the two interpenetrating airframes (~40 N weight vs ~34 N max thrust)
never left the ground.

This tool only runs `gz model --list` and `gz model -m <name> -p`; it never
spawns, removes or modifies anything.

Exit code: 0 = PASS, 1 = FAIL, 2 = Gazebo not reachable.
"""

import argparse
import math
import os
import re
import subprocess
import sys

DEFAULT_EXPECTED = ('x500_1', 'x500_2', 'x500_3')
STATIC_MODELS = ('ground_plane',)
_FLOAT = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'


def _gz(args, timeout):
    env = dict(os.environ)
    env.setdefault('GZ_IP', '127.0.0.1')  # same value the PX4 SITL make target uses
    out = subprocess.run(['gz'] + args, capture_output=True, text=True,
                         timeout=timeout, env=env, check=False)
    return out.stdout


def parse_model_list(text):
    """Return model names from `gz model --list` stdout."""
    return [m.group(1) for m in re.finditer(r'^\s*-\s+(\S+)\s*$', text, re.MULTILINE)]


def parse_pose(text):
    """Return (x, y, z) from `gz model -m NAME -p` stdout, or None."""
    m = re.search(r'Pose[^\n]*\n\s*\[\s*(%s)\s+(%s)\s+(%s)\s*\]' % (_FLOAT, _FLOAT, _FLOAT), text)
    return tuple(float(v) for v in m.groups()) if m else None


def evaluate(models, poses, expected, min_separation_m):
    """Pure check logic (unit-testable). Returns a list of problem strings."""
    problems = []
    vehicles = [m for m in models if m not in STATIC_MODELS]
    missing = [m for m in expected if m not in models]
    extra = [m for m in vehicles if m not in expected]
    if missing:
        problems.append(f'beklenen model(ler) yok: {missing}')
    if extra:
        problems.append(f'beklenmeyen/yetim model(ler) var: {extra} '
                        '(onceki oturumdan kalmis olabilir; Gazebo sunucusunu temiz baslatin)')
    for name in vehicles:
        if poses.get(name) is None:
            problems.append(f'{name}: poz okunamadi')
    named = [(n, poses[n]) for n in vehicles if poses.get(n) is not None]
    for i, (a, pa) in enumerate(named):
        for b, pb in named[i + 1:]:
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if d < min_separation_m:
                problems.append(f'{a} ile {b} ayni yerde/cakisiyor: yatay mesafe {d:.2f} m '
                                f'< {min_separation_m:.1f} m')
    return problems


def check_world(expected=DEFAULT_EXPECTED, min_separation_m=1.5, timeout=20, log=print):
    """Query Gazebo and evaluate. Returns (ok: bool|None, problems: list).

    ok is None when Gazebo could not be queried at all.
    """
    try:
        models = parse_model_list(_gz(['model', '--list'], timeout))
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, [f'gz sorgusu basarisiz: {exc}']
    if not models:
        return None, ['Gazebo dunyasi bulunamadi veya model listesi bos']
    poses = {}
    for name in models:
        if name in STATIC_MODELS:
            continue
        try:
            poses[name] = parse_pose(_gz(['model', '-m', name, '-p'], timeout))
        except (OSError, subprocess.TimeoutExpired):
            poses[name] = None
        p = poses[name]
        log(f'  {name}: ' + (f'x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}' if p else 'poz yok'))
    problems = evaluate(models, poses, list(expected), min_separation_m)
    return not problems, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--expected', default=','.join(DEFAULT_EXPECTED),
                    help='virgulle ayrilmis beklenen arac modelleri (varsayilan: %(default)s)')
    ap.add_argument('--min-separation', type=float, default=1.5,
                    help='araclar arasi minimum yatay mesafe, m (varsayilan: %(default)s)')
    args = ap.parse_args()
    expected = [m.strip() for m in args.expected.split(',') if m.strip()]
    print(f'Gazebo dunya denetimi: beklenen={expected}')
    ok, problems = check_world(expected, args.min_separation)
    if ok is None:
        print('GZ CHECK ERROR: ' + '; '.join(problems))
        return 2
    if ok:
        print('GZ CHECK PASS: model seti ve yerlesim uygun.')
        return 0
    for p in problems:
        print(f'GZ CHECK FAIL: {p}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
