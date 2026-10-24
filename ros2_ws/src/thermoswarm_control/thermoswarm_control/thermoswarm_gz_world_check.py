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

All queries are read-only, including world discovery and live SDF export.
No entity is spawned, removed or modified. CLI and ROS use this same module.

Exit code: 0 = PASS, 1 = FAIL, 2 = Gazebo not reachable.
"""

import argparse
import ast
from collections import Counter
import xml.etree.ElementTree as ET
import math
import os
import re
import subprocess
import sys

DEFAULT_EXPECTED = ('x500_1', 'x500_2', 'x500_3')
STATIC_MODELS = ('ground_plane',)
WORLD_NAMES = {'default': 'default', 'wildfire': 'thermoswarm_wildfire'}
PATCHES = {'fire_patch': 600, 'ambient_patch': 300}
TREES = tuple(f'tree_{i}' for i in range(1, 7))
WILDFIRE_STATIC = (*STATIC_MODELS, *PATCHES, *TREES)
_FLOAT = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'


def _gz(args, timeout):
    env = dict(os.environ)
    env.setdefault('GZ_IP', '127.0.0.1')  # same value the PX4 SITL make target uses
    out = subprocess.run(['gz'] + args, capture_output=True, text=True,
                         timeout=timeout, env=env, check=False)
    if out.returncode:
        raise OSError(f'gz exit {out.returncode}: {out.stderr.strip()}')
    return out.stdout


def parse_model_list(text):
    """Return model names from `gz model --list` stdout."""
    return [m.group(1) for m in re.finditer(r'^\s*-\s+(\S+)\s*$', text, re.MULTILINE)]


def parse_pose(text):
    """Return (x, y, z) from `gz model -m NAME -p` stdout, or None."""
    m = re.search(r'Pose[^\n]*\n\s*\[\s*(%s)\s+(%s)\s+(%s)\s*\]' % (_FLOAT, _FLOAT, _FLOAT), text)
    return tuple(float(v) for v in m.groups()) if m else None


def parse_sdf_response(output):
    """Decode the CLI's protobuf StringMsg without evaluating executable text."""
    match = re.fullmatch(r'\s*data:\s*("(?:[^"\\]|\\.)*")\s*', output, re.DOTALL)
    if not match:
        raise ValueError('canlı dünya SDF yanıtı okunamadı')
    return ET.fromstring(ast.literal_eval(match.group(1)))


def sdf_pose(element):
    pose = element.find('pose')
    if pose is not None and (pose.get('relative_to') or pose.get('degrees') == 'true'
                             or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy'):
        raise ValueError('desteklenmeyen SDF pose çerçevesi')
    values = tuple(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError('geçersiz SDF pose')
    return values


def wildfire_geometry(sdf, models):
    """Only live, static, collision-free thermal surfaces earn an exemption.

    Physical trees retain the 1.5m clearance, enlarged by their horizontal
    collision extent. Unsupported geometry/transforms fail closed.
    """
    if sdf is None:
        raise ValueError('wildfire için canlı SDF doğrulaması gerekli')
    world = sdf.find('world')
    if world is None or world.get('name') != WORLD_NAMES['wildfire']:
        raise ValueError('canlı SDF dünya adı uyuşmuyor')
    entities = world.findall('model')
    if Counter(m.get('name') for m in entities) != Counter(models):
        raise ValueError('canlı SDF/model listesi uyuşmuyor')
    if world.findall('.//include') or world.findall('actor'):
        raise ValueError('çözümlenmemiş include veya izinsiz actor')
    radii = {}
    for name in (*PATCHES, *TREES):
        model = world.find(f"model[@name='{name}']")
        if model is None or model.findtext('static', '').strip() not in ('true', '1'):
            raise ValueError(f'{name}: beklenen statik çevre modeli yok')
        if model.findall('.//model'):
            raise ValueError(f'{name}: izinsiz iç içe model')
        pose = sdf_pose(model)
        if any(abs(v) > 1e-6 for v in pose[3:5]):
            raise ValueError(f'{name}: eğik çevre modeli')
        if name in PATCHES:
            if model.findall('.//collision') or not 0 <= pose[2] <= .02:
                raise ValueError(f'{name}: collision içeriyor veya yerdeki yüzey değil')
            visuals = model.findall('link/visual')
            if len(model.findall('link')) != 1 or len(visuals) != 1:
                raise ValueError(f'{name}: beklenmeyen yüzey yapısı')
            link, visual = model.find('link'), visuals[0]
            if any(abs(v) > 1e-6 for e in (link, visual) for v in sdf_pose(e)):
                raise ValueError(f'{name}: beklenmeyen yerel yüzey pozu')
            size = tuple(map(float, visual.findtext('geometry/box/size', '').split()))
            if (len(size) != 3 or not all(map(math.isfinite, size))
                    or any(abs(a-b) > 1e-6 for a,b in zip(size,(1.2,1.2,.01)))):
                raise ValueError(f'{name}: beklenmeyen yüzey geometrisi')
            plugins = model.findall('.//plugin')
            if (len(plugins) != 1 or plugins[0] not in visual.findall('plugin')
                    or plugins[0].get('filename') != 'gz-sim-thermal-system'
                    or plugins[0].get('name') != 'gz::sim::systems::Thermal'
                    or float(plugins[0].findtext('temperature', 'nan')) != PATCHES[name]):
                raise ValueError(f'{name}: termal özellik doğrulanamadı')
            continue
        collisions = model.findall('link/collision')
        if not collisions:
            raise ValueError(f'{name}: fiziksel ağaç geometrisi eksik')
        radius = 0.0
        for link in model.findall('link'):
            lp = sdf_pose(link)
            if any(abs(v) > 1e-6 for v in lp[3:5]):
                raise ValueError(f'{name}: eğik link')
            for collision in link.findall('collision'):
                cp = sdf_pose(collision)
                if any(abs(v) > 1e-6 for v in cp[3:5]):
                    raise ValueError(f'{name}: eğik collision')
                geo = collision.find('geometry')
                if geo is None or len(geo) != 1:
                    raise ValueError(f'{name}: collision geometrisi okunamadı')
                shape = geo[0]
                if shape.tag in ('sphere', 'cylinder'):
                    extent = float(shape.findtext('radius', 'nan'))
                elif shape.tag == 'box':
                    size = tuple(map(float, shape.findtext('size', '').split()))
                    if len(size) != 3 or any(v <= 0 or not math.isfinite(v) for v in size):
                        raise ValueError(f'{name}: geçersiz box')
                    extent = math.hypot(*size[:2]) / 2
                else:
                    raise ValueError(f'{name}: desteklenmeyen collision {shape.tag}')
                if not math.isfinite(extent) or extent <= 0:
                    raise ValueError(f'{name}: geçersiz collision boyutu')
                # Triangle inequality bounds offsets even for yaw-rotated links.
                radius = max(radius, math.hypot(*lp[:2]) + math.hypot(*cp[:2]) + extent)
        radii[name] = radius
    return radii


def evaluate(models, poses, expected, min_separation_m, world='default', world_sdf=None):
    """Pure fail-closed policy shared by launcher, CLI and ROS preflight."""
    if world not in WORLD_NAMES:
        return [f'geçersiz dünya türü: {world}']
    if not math.isfinite(min_separation_m) or min_separation_m < 1.5:
        return ['minimum güvenlik mesafesi en az 1.5 m olmalı']
    static = WILDFIRE_STATIC if world == 'wildfire' else STATIC_MODELS
    problems = []
    if len(models) != len(set(models)):
        problems.append('tekrarlanan model isimleri var')
    missing = sorted(set(expected) - set(models))
    extra = sorted(set(models) - set(expected) - set(static))
    if missing:
        problems.append(f'beklenen model(ler) yok: {missing}')
    if extra:
        problems.append(f'beklenmeyen/yetim model(ler) var: {extra}')
    radii = {}
    exemptions = set(STATIC_MODELS)
    if world == 'wildfire':
        missing_decor = sorted(set(static) - set(models))
        if missing_decor:
            problems.append(f'eksik wildfire dekorları: {missing_decor}')
        try:
            radii = wildfire_geometry(world_sdf, models)
            exemptions.update(PATCHES)
        except (ValueError, TypeError) as exc:
            problems.append(str(exc))
    obstacles = [m for m in models if m not in exemptions]
    valid = {}
    for name in obstacles:
        p = poses.get(name)
        if p is None or len(p) != 3 or not all(map(math.isfinite, p)):
            problems.append(f'{name}: poz okunamadi/geçersiz')
        else:
            valid[name] = p
    named = list(valid.items())
    for i, (a, pa) in enumerate(named):
        for b, pb in named[i + 1:]:
            # Static-to-static proximity is irrelevant; every aircraft-to-object
            # pair, including unknown aircraft/objects, remains checked.
            if a in static and b in static:
                continue
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            limit = min_separation_m + radii.get(a, 0) + radii.get(b, 0)
            if d < limit:
                problems.append(f'{a} ile {b} ayni yerde/cakisiyor: yatay mesafe {d:.2f} m '
                                f'< {limit:.2f} m')
    return problems


def check_world(expected=DEFAULT_EXPECTED, min_separation_m=1.5, timeout=20, log=print, world='auto'):
    """Use the launcher's actual Gazebo world name; explicit selections must match.

    Auto never guesses from model names or falls back to default. World discovery
    must return exactly one supported world. This also works from installed ROS,
    independent of the current working directory and stale launcher manifests.
    """
    if world != 'auto' and world not in WORLD_NAMES:
        return False, [f'geçersiz dünya türü: {world}']
    try:
        discovered = _gz(['service', '-s', '/gazebo/worlds', '--reqtype', 'gz.msgs.Empty',
                          '--reptype', 'gz.msgs.StringMsg_V', '--timeout', str(int(timeout*1000)),
                          '--req', ''], timeout + 2)
        names = re.findall(r'^data:\s*"([^"\n]+)"\s*$', discovered, re.MULTILINE)
        if len(names) != 1 or names[0] not in WORLD_NAMES.values():
            return False, [f'belirsiz/desteklenmeyen Gazebo dünya listesi: {names}']
        detected = next(key for key, value in WORLD_NAMES.items() if value == names[0])
        if world != 'auto' and world != detected:
            return False, [f'beklenen dünya {world}; çalışan dünya {detected}']
        world = detected
        output = _gz(['model', '--list'], timeout)
        markers = re.findall(r'Requesting state for world \[([^]]+)\]', output)
        if markers != names:
            return False, ['model sorgusu farklı/belirsiz dünya döndürdü']
        models = parse_model_list(output)
        if not models:
            return None, ['Gazebo model listesi boş']
        sdf = None
        if world == 'wildfire':
            response = _gz(['service', '-s', f'/world/{names[0]}/generate_world_sdf',
                            '--reqtype', 'gz.msgs.SdfGeneratorConfig', '--reptype', 'gz.msgs.StringMsg',
                            '--timeout', str(int(timeout*1000)), '--req',
                            'global_entity_gen_config { expand_include_tags { data: true } }'], timeout + 2)
            sdf = parse_sdf_response(response)
        log(f'  Dünya: {world} ({names[0]})')
        poses = {}
        for name in models:
            if name in STATIC_MODELS or (world == 'wildfire' and name in PATCHES):
                continue
            output = _gz(['model', '-m', name, '-p'], timeout)
            if re.findall(r'Requesting state for world \[([^]]+)\]', output) != names:
                return False, [f'{name}: poz sorgusu farklı/belirsiz dünya döndürdü']
            poses[name] = parse_pose(output)
            p = poses[name]
            log(f'  {name}: ' + (f'x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}' if p else 'poz yok'))
        problems = evaluate(models, poses, list(expected), min_separation_m, world, sdf)
        return not problems, problems
    except (OSError, subprocess.TimeoutExpired, ValueError, SyntaxError, ET.ParseError) as exc:
        return None, [f'Gazebo salt-okunur doğrulaması başarısız: {exc}']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--expected', default=','.join(DEFAULT_EXPECTED))
    ap.add_argument('--world', choices=('auto', *WORLD_NAMES), default='auto')
    ap.add_argument('--min-separation', type=float, default=1.5)
    args = ap.parse_args()
    expected = [m.strip() for m in args.expected.split(',') if m.strip()]
    ok, problems = check_world(expected, args.min_separation, world=args.world)
    if ok:
        print('GZ CHECK PASS: model seti ve yerlesim uygun.')
        return 0
    for problem in problems:
        print(f'GZ CHECK FAIL: {problem}')
    return 2 if ok is None else 1


if __name__ == '__main__':
    sys.exit(main())
