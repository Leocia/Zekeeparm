"""Simulation copies use the bundled assets without modifying their default URDF."""
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from model_source import SOURCE, validate, staged_bringup
from export_model import export

original = SOURCE.read_bytes()
text = original.decode()
validate(text)
for invalid in [text.replace('name="joint1"', 'name="missing"'),
                text.replace('lower="-2.58"', 'lower="nan"'),
                text.replace('<axis xyz="0 0 -1" />', '<axis xyz="0 0 -1" /><dynamics damping="0.5" />', 1),
                text.replace('multiplier="1"', 'multiplier="2"'),
                text.replace('package://zekeep_bringup/description/meshes/base_link.STL',
                             'package://zekeep_bringup/../README.md'),
                '<!DOCTYPE robot><robot/>']:
    try:
        validate(invalid)
    except (ValueError, ET.ParseError):
        pass
    else:
        raise AssertionError('invalid URDF accepted')
arm = ET.fromstring(text)
arm.find("joint[@name='joint3']/origin").set('xyz', '.265 0 0')
arm.find("joint[@name='joint2']/limit").set('velocity', '.5')
arm.find("link[@name='link6']/collision/geometry/mesh").set(
    'filename', 'package://zekeep_bringup/description/collision/link6.STL')
arm.find("link[@name='link2']/inertial/origin").set('rpy', '0 0 1.5707963267948966')
with tempfile.TemporaryDirectory() as folder:
    output = Path(folder)
    with staged_bringup(ET.tostring(arm, encoding='unicode')) as staged:
        export(staged, output)
    model = mujoco.MjModel.from_xml_path(str(output / 'public/models/rs_grasp_scene.xml'))
    assert abs(model.body('link3').pos[0] - .265) < 1e-8
    assert model.nmesh == 11, 'collision meshes must be distinct from visual meshes'
    assert model.geom('link6_collision').dataid[0] != model.geom('link6_visual').dataid[0]
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, model.body('link2').iquat)
    rotation = rotation.reshape(3, 3)
    tensor = rotation @ np.diag(model.body('link2').inertia) @ rotation.T
    assert abs(tensor[0, 0] - .005877765) < 1e-7, 'inertial rpy must rotate the tensor'
    import json
    assert json.loads((output / 'src/robot-config.json').read_text())['velocityLimits']['joint2'] == .5
assert SOURCE.read_bytes() == original, 'export must not modify the bundled default URDF'
print('PASS: URDF edits reach the physics model; invalid input rejected; bundled default preserved')
