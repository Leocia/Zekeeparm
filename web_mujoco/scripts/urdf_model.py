"""Local copy of the workspace URDF converter; retains its gains and pad calibration."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco


def numbers(text):
    return np.fromstring(text, sep=' ')


def quaternion(rpy):
    r, p, y = numbers(rpy)/2
    cr, cp, cy, sr, sp, sy = np.cos(r), np.cos(p), np.cos(y), np.sin(r), np.sin(p), np.sin(y)
    return ' '.join(map(str, (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)))


def origin(element):
    value = element.find('origin')
    return {'pos': value.get('xyz', '0 0 0') if value is not None else '0 0 0',
            'quat': quaternion(value.get('rpy', '0 0 0') if value is not None else '0 0 0')}


def build_model(bringup, *, friction=1.2, object_mass=.04, table_z=0):
    if not .1 <= friction <= 3 or not .005 <= object_mass <= .5 or not -.2 <= table_z <= .1:
        raise ValueError('invalid simulation physics parameters')
    bringup = Path(bringup)
    urdf = ET.parse(bringup/'description/urdf/sixaxis.urdf').getroot()
    root = ET.Element('mujoco', model='zekeep_sixaxis_sim')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast', cone='elliptic')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='640', offheight='480')
    asset, world, actuators = (ET.SubElement(root, name) for name in ('asset', 'worldbody', 'actuator'))
    ET.SubElement(world, 'light', pos='0 0 2', dir='0 0 -1', diffuse='.8 .8 .8')
    ET.SubElement(world, 'camera', name='overhead', pos='.25 0 1.2', quat='1 0 0 0', fovy='55')
    ET.SubElement(world, 'geom', name='table', type='plane', pos=f'0 0 {table_z}', size='1 1 .05', rgba='.35 .4 .45 1', friction=f'{friction} .01 .001', contype='2', conaffinity='1')
    links = {link.get('name'): link for link in urdf.findall('link')}
    children = {}
    for joint in urdf.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)

    def add_link(parent, name, transform=None, joint=None):
        body = ET.SubElement(parent, 'body', name=name, **(transform or {}))
        link = links[name]
        inertial = link.find('inertial')
        if inertial is not None:
            inertia = inertial.find('inertia')
            ET.SubElement(body, 'inertial', pos=origin(inertial)['pos'], mass=inertial.find('mass').get('value'),
                          fullinertia=' '.join(inertia.get(key) for key in ('ixx','iyy','izz','ixy','ixz','iyz')))
        if joint is not None and joint.get('type') != 'fixed':
            limits = joint.find('limit')
            jname = joint.get('name')
            ET.SubElement(body, 'joint', name=jname, type='slide' if joint.get('type') == 'prismatic' else 'hinge',
                          axis=joint.find('axis').get('xyz'), range=f"{limits.get('lower')} {limits.get('upper')}",
                          damping='2' if joint.get('type') == 'revolute' else '1', armature='.01')
            effort = float(limits.get('effort'))
            ET.SubElement(actuators, 'position', name=jname, joint=jname, kp='400' if jname.startswith('joint') else '800',
                          kv='30' if jname.startswith('joint') else '8',
                          ctrlrange=f"{limits.get('lower')} {limits.get('upper')}", forcerange=f'{-effort} {effort}')
        for index, geom in enumerate(link.findall('visual')):
            mesh = geom.find('geometry/mesh')
            if mesh is None:
                continue
            meshname = f'{name}_{index}'
            file = bringup/mesh.get('filename').split('package://zekeep_bringup/')[1]
            ET.SubElement(asset, 'mesh', name=meshname, file=str(file.resolve()))
            ET.SubElement(body, 'geom', name=f'{name}_visual', type='mesh', mesh=meshname, **origin(geom),
                          contype='0', conaffinity='0', group='2', rgba='.75 .8 .85 1')
        for geom in link.findall('collision'):
            if name in ('left_link','right_link'):
                # ponytail: one measured pad box per finger, refine contact mesh if slip fidelity matters.
                # Pad coordinates match STL bounds at the distal finger (URDF local frame).
                x = .00688195 if name == 'left_link' else -.00725383
                ET.SubElement(body, 'geom', name=f'{name}_contact', type='box', pos=f'{x} 0 .142575',
                              size='.006 .002793 .008625', rgba='.25 .25 .25 1', contype='1', conaffinity='2',
                              friction=f'{friction} .01 .001')
            else:
                mesh = geom.find('geometry/mesh')
                if mesh is not None:
                    ET.SubElement(body, 'geom', name=f'{name}_collision', type='mesh', mesh=f'{name}_0',
                                  **origin(geom), rgba='.7 .7 .7 0', group='3', contype='1', conaffinity='2')
        if name == 'grasp_tcp':
            ET.SubElement(body, 'site', name='tcp', size='.004', rgba='0 1 0 1')
        for child in children.get(name, []):
            add_link(body, child.find('child').get('link'), origin(child), child)
    add_link(world, 'base_link')
    body = ET.SubElement(world, 'body', name='red', pos=f'.32 0 {table_z+.025}')
    ET.SubElement(body, 'freejoint', name='red_free')
    ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(object_mass), diaginertia=' '.join([str(object_mass*.05**2/6)]*3))
    ET.SubElement(body, 'geom', name='red_object', type='box', size='.025 .025 .025', rgba='1 .03 .02 1',
                  contype='2', conaffinity='3', friction=f'{friction} .01 .001')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
