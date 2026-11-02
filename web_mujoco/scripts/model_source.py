"""Validate the existing arm contract and stage a simulation-only URDF."""
from contextlib import contextmanager
from pathlib import Path
import math
import tempfile
import xml.etree.ElementTree as ET

WEB = Path(__file__).resolve().parents[1]
BRINGUP = WEB / 'robot'
SOURCE = BRINGUP / 'description/urdf/sixaxis.urdf'
JOINTS = {f'joint{i}': ('revolute', 'base_link' if i == 1 else f'link{i-1}', f'link{i}')
          for i in range(1, 7)}
JOINTS.update({
    'gripper_base_fixed_joint': ('fixed', 'link6', 'gripper_base'),
    'gripper_joint': ('prismatic', 'gripper_base', 'left_link'),
    'right_joint': ('prismatic', 'gripper_base', 'right_link'),
    'grasp_tcp_fixed_joint': ('fixed', 'link6', 'grasp_tcp'),
})
LINKS = {link for _, parent, child in JOINTS.values() for link in (parent, child)}


def validate(text):
    if len(text.encode()) > 512 * 1024 or '<!DOCTYPE' in text.upper() or '<!ENTITY' in text.upper():
        raise ValueError('URDF 过大或包含不支持的 XML 声明')
    arm = ET.fromstring(text)
    if arm.tag != 'robot':
        raise ValueError('需要 robot 根节点')
    for tag in ('link', 'joint'):
        expected = JOINTS if tag == 'joint' else LINKS
        actual = {node.get('name'): node for node in arm.findall(tag)}
        if actual.keys() != set(expected) or len(actual) != len(arm.findall(tag)):
            raise ValueError('保留当前机械臂的 link/joint 名称与数量')
        if tag == 'joint':
            for name, node in actual.items():
                kind, parent, child = JOINTS[name]
                if node.get('type') != kind or any(node.find(side) is None or node.find(side).get('link') != link
                    for side, link in [('parent', parent), ('child', child)]):
                    raise ValueError(f'{name}: 保留关节类型和连接关系')
                if node.find('dynamics') is not None:
                    raise ValueError(f'{name}: 当前控制器不支持 dynamics')
                mimic = node.find('mimic')
                if (name == 'right_joint' and (mimic is None or mimic.get('joint') != 'gripper_joint' or
                        float(mimic.get('multiplier', '1')) != 1 or float(mimic.get('offset', '0')) != 0)) or (
                        name != 'right_joint' and mimic is not None):
                    raise ValueError(f'{name}: 保留当前夹爪 mimic 关系')
                if node.get('type') != 'fixed':
                    limit = node.find('limit')
                    lo, hi = float(limit.get('lower')), float(limit.get('upper'))
                    if not math.isfinite(lo + hi) or lo >= hi or float(limit.get('effort')) <= 0 or float(limit.get('velocity')) <= 0:
                        raise ValueError(f'{name}: 无效限位或力矩')
                    if name in ('gripper_joint', 'right_joint') and (lo != 0 or hi != .035):
                        raise ValueError('当前夹爪控制要求单指范围为 0–35 mm')
        else:
            for name, node in actual.items():
                for role in ('visual', 'collision'):
                    parts = node.findall(role)
                    if len(parts) != (0 if name == 'grasp_tcp' else 1) or any(part.find('geometry/mesh') is None for part in parts):
                        raise ValueError('当前转换器仅支持现有数量的网格 visual/collision')
                    if name in ('left_link', 'right_link'):
                        for part in parts:
                            if part.find('geometry/mesh').get('filename') != f'package://zekeep_bringup/description/meshes/{name}.STL':
                                raise ValueError('保留已标定指垫的手指网格')
                            transform = part.find('origin')
                            if role == 'collision' and transform is not None and any(
                                    float(v) != 0 for key in ('xyz', 'rpy') for v in transform.get(key, '0 0 0').split()):
                                raise ValueError('保留已标定指垫的碰撞原点')
    for node in arm.iter():
        for key in ('xyz', 'rpy', 'rgba', 'value', 'lower', 'upper', 'effort', 'velocity',
                    'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            if key in node.attrib and not all(math.isfinite(float(v)) for v in node.get(key).split()):
                raise ValueError(f'{node.tag}/{key}: 需要有限数值')
    for mesh in arm.findall('.//mesh'):
        prefix = 'package://zekeep_bringup/'
        filename = mesh.get('filename', '')
        if not filename.startswith(prefix):
            raise ValueError('网格仅支持内置机器人资源路径')
        path = (BRINGUP / filename[len(prefix):]).resolve()
        if not path.is_relative_to((BRINGUP / 'description').resolve()) or not path.is_file():
            raise ValueError('网格文件不存在或超出 description 目录')
        if mesh.get('scale', '1 1 1') != '1 1 1':
            raise ValueError('当前转换器不支持网格缩放，请使用原尺寸网格')
    return arm


@contextmanager
def staged_bringup(text):
    validate(text)
    with tempfile.TemporaryDirectory(prefix='zekeeparm-urdf-') as folder:
        path = Path(folder)
        (path / 'description/urdf').mkdir(parents=True)
        (path / 'description/urdf/sixaxis.urdf').write_text(text)
        for name in ('meshes', 'collision'):
            (path / 'description' / name).symlink_to(BRINGUP / 'description' / name)
        yield path
