"""Build the web scene from the project's bundled robot or simulation copy."""
import copy
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

WEB = Path(__file__).resolve().parents[1]
from model_source import BRINGUP, SOURCE, staged_bringup
from urdf_model import build_model, origin


def export(bringup=None, output=WEB):
    destination = output / 'public/models'
    (destination / 'meshes').mkdir(parents=True, exist_ok=True)
    bringup = bringup or BRINGUP
    urdf = ET.parse(bringup / 'description/urdf/sixaxis.urdf').getroot()
    model = build_model(bringup)
    with tempfile.TemporaryDirectory(prefix='zekeeparm-web-model-') as temporary:
        saved = Path(temporary) / 'arm.xml'
        mujoco.mj_saveLastXML(str(saved), model)
        generated = ET.parse(saved).getroot()
    assets = generated.find('asset')
    arm = generated.find("worldbody/body[@name='base_link']")
    for link in urdf.findall('link'):
        name = link.get('name')
        body = arm if name == 'base_link' else arm.find(f".//body[@name='{name}']")
        inertial = link.find('inertial')
        if inertial is not None and inertial.find('origin') is not None and any(
                float(v) for v in inertial.find('origin').get('rpy', '0 0 0').split()):
            # The shared converter ignores inertial rpy; rotate its tensor into the body frame.
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, np.fromstring(origin(inertial)['quat'], sep=' '))
            rotation = rotation.reshape(3, 3)
            values = inertial.find('inertia').attrib
            tensor = np.array([[float(values['i' + a + b] if 'i' + a + b in values else values['i' + b + a])
                                for b in 'xyz'] for a in 'xyz'])
            tensor = rotation @ tensor @ rotation.T
            body.find('inertial').attrib = dict(pos=origin(inertial)['pos'], mass=inertial.find('mass').get('value'),
                fullinertia=' '.join(str(tensor[i, j]) for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))))
        collision = link.find('collision')
        if collision is None:
            continue
        geom = body.find(f"geom[@name='{name}_collision']")
        if name in ('left_link', 'right_link'):
            geom = copy.deepcopy(body.find(f"geom[@name='{name}_visual']"))
            geom.attrib.update(name=name + '_collision', contype='1', conaffinity='3', **origin(collision))
            body.append(geom)
        mesh = collision.find('geometry/mesh')
        if mesh is not None and mesh.get('filename') != link.find('visual/geometry/mesh').get('filename'):
            mesh_name = name + '_collision_mesh'
            file = bringup / mesh.get('filename').split('package://zekeep_bringup/')[1]
            ET.SubElement(assets, 'mesh', name=mesh_name, file=str(file.resolve()))
            geom.set('mesh', mesh_name)
    mesh_files = {}
    for mesh in assets:
        original = Path(mesh.get('file'))
        name = 'zk_' + mesh.get('name') + original.suffix
        shutil.copyfile(original, destination / 'meshes' / name)
        mesh.set('file', name)
        mesh.set('name', 'zk_' + mesh.get('name'))
        mesh_files['package://zekeep_bringup/' + str(original.relative_to(BRINGUP))] = {
            'name': mesh.get('name'), 'file': name}
    # Static sites compile edits in WASM using the same bundled mesh whitelist.
    for folder in ('meshes', 'collision'):
        for file in sorted((BRINGUP / 'description' / folder).glob('*.STL')):
            key = f'package://zekeep_bringup/description/{folder}/{file.name}'
            if key in mesh_files:
                continue
            name = f'urdf_{folder}_{file.stem}'
            filename = name + '.STL'
            shutil.copyfile(file, destination / 'meshes' / filename)
            mesh_files[key] = {'name': name, 'file': filename}
    (destination / 'browser-model.json').write_text(json.dumps({
        'urdf': (bringup / 'description/urdf/sixaxis.urdf').read_text(), 'meshes': mesh_files}, ensure_ascii=False))
    for material in urdf.findall('material'):
        ET.SubElement(assets, 'material', name='zk_' + material.get('name'),
                      rgba=material.find('color').get('rgba'))
    body_names = {'left_link': 'gripper_left', 'right_link': 'gripper_right', 'grasp_tcp': 'gripper_end'}
    for body in arm.iter('body'):
        body.set('name', body_names.get(body.get('name'), body.get('name')))
    for geom in arm.iter('geom'):
        if 'mesh' in geom.attrib:
            geom.set('mesh', 'zk_' + geom.get('mesh'))
        geom.set('group', '1' if geom.get('name', '').endswith('_visual') else '3')
        if not geom.get('name', '').endswith('_visual'):
            geom.set('conaffinity', '3')
            geom.set('solref', '.002 1')
            geom.set('solimp', '.99 .999 .0005')
        if geom.get('name', '').endswith('_visual'):
            link_name = geom.get('name').removesuffix('_visual')
            material = urdf.find(f"link[@name='{link_name}']/visual/material")
            if material is not None and material.get('name'):
                geom.attrib.pop('rgba', None)
                geom.set('material', 'zk_' + material.get('name'))
            if material is not None and material.find('color') is not None:
                name = 'zk_inline_' + link_name
                ET.SubElement(assets, 'material', name=name, rgba=material.find('color').get('rgba'))
                geom.attrib.pop('rgba', None)
                geom.set('material', name)
    flange = arm.find(".//body[@name='link6']")
    # Gemini 305 enclosure: 42 × 42 × 23 mm; stereo baseline 18 mm.
    # The mount follows the user's screenshot, not a measured hand-eye calibration.
    camera_position = np.array([.00, .066, .05])
    camera_x_tilt_deg = 15.
    half_tilt = np.deg2rad(camera_x_tilt_deg) / 2
    camera_quat = np.array([-np.sin(half_tilt), np.cos(half_tilt), 0., 0.])
    camera_rotation = np.zeros(9)
    mujoco.mju_quat2Mat(camera_rotation, camera_quat)
    camera_rotation = camera_rotation.reshape(3, 3)
    pose_quat = ' '.join(map(str, camera_quat))
    ET.SubElement(flange, 'geom', name='wrist_camera_housing', type='box',
                  pos=' '.join(map(str, camera_position)), quat=pose_quat,
                  size='.021 .021 .0115', rgba='.12 .13 .14 1', group='1', contype='0', conaffinity='0')
    for eye, x in [('left', -.009), ('right', .009)]:
        position = camera_position + camera_rotation @ np.array([x, 0, -.0125])
        pos = ' '.join(map(str, position))
        if eye == 'left':
            ET.SubElement(flange, 'camera', name='wrist_rgb', pos=pos, quat=pose_quat, fovy='68')
        ET.SubElement(flange, 'geom', name=f'wrist_camera_{eye}_lens', type='cylinder',
                      pos=pos, quat=pose_quat, size='.004 .001', rgba='.025 .075 .10 1',
                      group='1', contype='0', conaffinity='0')
    scene = ET.parse(WEB / 'reference_models/rs_grasp_scene.xml')
    root = scene.getroot()
    for include in root.findall('include'):
        root.remove(include)
    root.set('model', 'zekeeparm_grasp_scene')
    contact = root.find('contact')
    contact.clear()
    srdf = ET.parse(BRINGUP / 'zekeep.srdf').getroot()
    for pair in srdf.findall('disable_collisions'):
        ET.SubElement(contact, 'exclude',
                      body1=body_names.get(pair.get('link1'), pair.get('link1')),
                      body2=body_names.get(pair.get('link2'), pair.get('link2')))
    asset = root.find('asset')
    for element in assets:
        asset.append(copy.deepcopy(element))
    world = root.find('worldbody')
    # The tabletop is at Z=0; keep the floor below its 30 mm thickness.
    # Coplanar visible surfaces cause depth-buffer flicker in both camera views.
    world.find("geom[@name='floor']").set('pos', '0 0 -.06')
    initial_xy = {'red_cube': (.24, -.25), 'blue_block': (.30, .06), 'yellow_cylinder': (.33, -.10)}
    zones = {'red': (.45, -.18), 'blue': (.36, 0), 'yellow': (.45, .18)}
    original_zone_y = {'red': -.16, 'blue': 0, 'yellow': .16}
    for element in world:
        if 'pos' not in element.attrib:
            continue
        pos = np.fromstring(element.get('pos'), sep=' ')
        name = element.get('name', '')
        if name in initial_xy:
            pos[:2] = initial_xy[name]
        for color, center in zones.items():
            if name.startswith(color + '_zone_'):
                pos[0] += center[0] - .49
                pos[1] += center[1] - original_zone_y[color]
        if name in (*initial_xy, 'task_table', 'ik_target', 'overhead_rgb') or '_zone_' in name:
            pos[2] -= .10
        element.set('pos', ' '.join(map(str, pos)))
    world.append(arm)
    obstacle = ET.SubElement(world, 'body', name='task_obstacle', pos='.30 -.30 .09', mocap='true')
    ET.SubElement(obstacle, 'geom', name='task_obstacle_geom', type='box', size='.025 .045 .09',
                  rgba='.75 .39 .16 0', contype='0', conaffinity='0', group='0')
    root.append(copy.deepcopy(generated.find('actuator')))
    root.find('option').set('timestep', '.001')
    root.find('option').set('integrator', 'implicitfast')
    limits = {j.get('name'): [float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))]
              for j in urdf.findall('joint') if j.get('type') != 'fixed'}
    home = {j.get('name'): float(j.get('value')) for j in srdf.find("group_state[@name='ready_pose']")}
    for name, amount in home.items():
        if name not in limits or not limits[name][0] <= amount <= limits[name][1]:
            raise ValueError(f'{name}: URDF 限位必须包含 SRDF ready_pose ({amount} rad)')
    data = mujoco.MjData(model)
    for name, amount in home.items():
        data.qpos[model.joint(name).qposadr[0]] = amount
    mujoco.mj_forward(model, data)
    pads = np.mean([data.geom(name).xpos for name in ('left_link_contact', 'right_link_contact')], axis=0)
    pinch_local = data.body('link6').xmat.reshape(3, 3).T @ (pads - data.body('link6').xpos)
    ET.SubElement(flange, 'site', name='pinch_center', pos=' '.join(map(str, pinch_local)), size='.002', rgba='0 0 0 0')
    closed_gap = float(np.linalg.norm(data.geom('left_link_contact').xpos - data.geom('right_link_contact').xpos)
                       - model.geom('left_link_contact').size[0] - model.geom('right_link_contact').size[0])
    storage_zones = {}
    for body_name in initial_xy:
        color = body_name.split('_', 1)[0]
        borders = [np.fromstring(world.find(f"geom[@name='{color}_zone_{side}']").get('pos'), sep=' ')
                   for side in ('n', 's', 'e', 'w')]
        center = np.mean(borders, axis=0)
        object_z = np.fromstring(world.find(f"body[@name='{body_name}']").get('pos'), sep=' ')[2]
        storage_zones[color] = {'x': float(center[0]), 'y': float(center[1]), 'z': float(object_z)}
    config = {'limits': limits, 'homePose': home,
              'velocityLimits': {j.get('name'): float(j.find('limit').get('velocity'))
                                 for j in urdf.findall('joint') if j.get('type') != 'fixed'},
              'originalGraspRotation': data.body('link6').xmat.tolist(),
              'gripperClosedGap': closed_gap, 'storageZones': storage_zones}
    scene.write(destination / 'rs_grasp_scene.xml', encoding='unicode')
    for mesh in sorted((destination / 'meshes').rglob('*')):
        if mesh.is_file() and mesh.suffix.lower() == '.stl':
            mesh.with_name(mesh.name + '.gzbin').write_bytes(gzip.compress(mesh.read_bytes(), mtime=0))
    version = hashlib.sha256()
    for file in sorted(destination.rglob('*')):
        if file.is_file():
            version.update(str(file.relative_to(destination)).encode())
            version.update(file.read_bytes())
    config['modelVersion'] = version.hexdigest()[:12]
    verified = mujoco.MjModel.from_xml_path(str(destination / 'rs_grasp_scene.xml'))
    (output / 'src').mkdir(parents=True, exist_ok=True)
    (output / 'src/robot-config.json').write_text(json.dumps(config, indent=2) + '\n')
    print(f'Arm: {verified.njnt} joints, {verified.ngeom} geoms, {verified.ncam} cameras')



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', type=Path, default=WEB)
    args = parser.parse_args()
    source = args.input or (WEB / 'simulation.urdf' if (WEB / 'simulation.urdf').exists()
                            else SOURCE)
    # Validate and compile in a temporary directory before publishing generated files.
    with staged_bringup(source.read_text()) as bringup, tempfile.TemporaryDirectory(prefix='zekeeparm-export-') as folder:
        staged = Path(folder)
        export(bringup, staged)
        (args.output / 'public').mkdir(parents=True, exist_ok=True)
        (args.output / 'src').mkdir(parents=True, exist_ok=True)
        shutil.copytree(staged / 'public/models', args.output / 'public/models', dirs_exist_ok=True)
        config = args.output / 'src/robot-config.json'
        shutil.copyfile(staged / 'src/robot-config.json', config.with_suffix('.json.tmp'))
        config.with_suffix('.json.tmp').replace(config)
