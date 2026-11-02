import { Euler, Quaternion } from 'three';
import robot from './robot-config.json' with { type: 'json' };

const bodyName = name => ({ left_link: 'gripper_left', right_link: 'gripper_right', grasp_tcp: 'gripper_end' }[name] || name);
const children = (node, tag) => [...node.children].filter(child => child.tagName === tag);
const child = (node, tag) => children(node, tag)[0];
const named = (node, tag, name) => [...node.getElementsByTagName(tag)].find(item => item.getAttribute('name') === name);
function requireValue(condition, message) { if (!condition) throw new Error(message); }
function numbers(value, count, label) {
  const parts = (value ?? '').trim().split(/\s+/);
  const values = parts.map(Number);
  requireValue(parts[0] !== '' && values.length === count && values.every(Number.isFinite), `${label}: 需要 ${count} 个有限数值`);
  return values;
}
function attribute(node, key, count = 1) { return numbers(node?.getAttribute(key), count, `${node?.tagName || '缺少节点'}/${key}`); }
function parse(text, tag) {
  requireValue(typeof text === 'string' && new TextEncoder().encode(text).length <= 512 * 1024 && !/<!DOCTYPE|<!ENTITY/i.test(text), 'URDF 过大或包含不支持的 XML 声明');
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  requireValue(!doc.querySelector('parsererror') && doc.documentElement.tagName === tag, `需要有效的 ${tag} XML`);
  return doc.documentElement;
}
function pose(node) {
  const origin = child(node, 'origin');
  const position = numbers(origin?.getAttribute('xyz') ?? '0 0 0', 3, 'origin/xyz');
  const rpy = numbers(origin?.getAttribute('rpy') ?? '0 0 0', 3, 'origin/rpy');
  const q = new Quaternion().setFromEuler(new Euler(...rpy, 'ZYX'));
  return { pos: position.join(' '), quat: [q.w, q.x, q.y, q.z].join(' '), q };
}
function setPose(target, source) {
  const { pos, quat } = pose(source);
  target.setAttribute('pos', pos); target.setAttribute('quat', quat);
}
function element(parent, tag, attributes) {
  const node = parent.ownerDocument.createElement(tag);
  Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value));
  parent.appendChild(node);
  return node;
}

// Same fixed arm contract as model_source.py; XML stays data and never enters the page DOM.
export function convertBrowserUrdf(text, sceneXml, source) {
  const arm = parse(text, 'robot'), baseline = parse(source.urdf, 'robot');
  const scene = parse(sceneXml, 'mujoco'), asset = child(scene, 'asset');
  const config = structuredClone(robot);
  for (const tag of ['link', 'joint']) {
    const actual = children(arm, tag), expected = children(baseline, tag);
    requireValue(actual.length === expected.length && new Set(actual.map(n => n.getAttribute('name'))).size === expected.length &&
      expected.every(n => actual.some(a => a.getAttribute('name') === n.getAttribute('name'))), '保留当前机械臂的 link/joint 名称与数量');
  }
  for (const node of arm.getElementsByTagName('*')) {
    for (const key of ['xyz', 'rpy', 'rgba', 'value', 'lower', 'upper', 'effort', 'velocity', 'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']) {
      if (node.hasAttribute(key)) {
        const values = node.getAttribute(key).trim().split(/\s+/);
        requireValue(values[0] !== '' && values.every(v => Number.isFinite(Number(v))), `${node.tagName}/${key}: 需要有限数值`);
      }
    }
  }
  function meshFor(part) {
    const geometry = child(part, 'geometry'), mesh = geometry && child(geometry, 'mesh');
    requireValue(mesh && geometry.children.length === 1, '当前转换器仅支持网格 visual/collision');
    const entry = source.meshes[mesh.getAttribute('filename')];
    requireValue(entry && Object.hasOwn(source.meshes, mesh.getAttribute('filename')), '网格仅支持内置机器人资源路径');
    requireValue(!mesh.hasAttribute('scale') || mesh.getAttribute('scale') === '1 1 1', '当前转换器不支持网格缩放');
    if (!named(asset, 'mesh', entry.name)) element(asset, 'mesh', { name: entry.name, file: entry.file });
    return entry.name;
  }
  for (const link of children(arm, 'link')) {
    const name = link.getAttribute('name'), body = named(scene, 'body', bodyName(name));
    const inertial = child(link, 'inertial'), previousInertial = child(body, 'inertial');
    requireValue(Boolean(inertial) === Boolean(child(named(baseline, 'link', name), 'inertial')), `${name}: 保留惯性节点`);
    if (inertial) {
      const mass = attribute(child(inertial, 'mass'), 'value')[0];
      requireValue(mass > 0, `${name}: 质量必须为正数`);
      const inertia = child(inertial, 'inertia');
      const [xx, yy, zz, xy, xz, yz] = ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'].map(key => attribute(inertia, key)[0]);
      const { pos, q } = pose(inertial);
      const { x, y, z, w } = q;
      const r = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]];
      const tensor = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
      const rotated = (i, j) => r[i].reduce((sum, v, k) => sum + v * r[j].reduce((s, u, l) => s + u * tensor[k][l], 0), 0);
      previousInertial.remove();
      element(body, 'inertial', { pos, mass, fullinertia: [[0,0],[1,1],[2,2],[0,1],[0,2],[1,2]].map(([i,j]) => rotated(i,j)).join(' ') });
    }
    for (const role of ['visual', 'collision']) {
      const parts = children(link, role);
      requireValue(parts.length === (name === 'grasp_tcp' ? 0 : 1), '保留当前 visual/collision 数量');
      if (!parts.length) continue;
      const part = parts[0], mesh = meshFor(part);
      const finger = ['left_link', 'right_link'].includes(name);
      if (finger) {
        requireValue(child(child(part, 'geometry'), 'mesh').getAttribute('filename') === `package://zekeep_bringup/description/meshes/${name}.STL`, '保留已标定指垫的手指网格');
        if (role === 'collision') {
          const p = pose(part);
          requireValue(numbers(p.pos, 3, '指垫原点').every(v => v === 0) &&
            numbers(child(part, 'origin')?.getAttribute('rpy') ?? '0 0 0', 3, '指垫旋转').every(v => v === 0), '保留已标定指垫的碰撞原点');
        }
      }
      const geom = named(body, 'geom', `${name}_${role}`);
      geom.setAttribute('mesh', mesh); setPose(geom, part);
      if (role === 'visual') {
        const material = child(part, 'material');
        const globalMaterial = material && children(arm, 'material').find(node => node.getAttribute('name') === material.getAttribute('name'));
        const color = material && (child(material, 'color') || child(globalMaterial || material, 'color'));
        requireValue(!material || color, `${name}: 材质缺少颜色`);
        geom.removeAttribute('material');
        const rgba = color ? attribute(color, 'rgba', 4) : [.75, .8, .85, 1];
        requireValue(rgba.every(v => v >= 0 && v <= 1), 'rgba 必须在 0–1 内');
        geom.setAttribute('rgba', rgba.join(' '));
      }
    }
  }
  for (const joint of children(arm, 'joint')) {
    const name = joint.getAttribute('name'), expected = named(baseline, 'joint', name);
    requireValue(joint.getAttribute('type') === expected.getAttribute('type') && ['parent', 'child'].every(side =>
      child(joint, side)?.getAttribute('link') === child(expected, side).getAttribute('link')), `${name}: 保留关节类型和连接关系`);
    requireValue(!child(joint, 'dynamics'), `${name}: 当前控制器不支持 dynamics`);
    const mimic = child(joint, 'mimic');
    requireValue(name === 'right_joint' ? mimic && mimic.getAttribute('joint') === 'gripper_joint' &&
      Number(mimic.getAttribute('multiplier') ?? 1) === 1 && Number(mimic.getAttribute('offset') ?? 0) === 0 : !mimic, `${name}: 保留夹爪 mimic 关系`);
    setPose(named(scene, 'body', bodyName(child(joint, 'child').getAttribute('link'))), joint);
    if (joint.getAttribute('type') === 'fixed') continue;
    const limit = child(joint, 'limit');
    const [lo, hi, effort, velocity] = ['lower', 'upper', 'effort', 'velocity'].map(key => attribute(limit, key)[0]);
    requireValue(lo < hi && effort > 0 && velocity > 0, `${name}: 无效限位、力矩或速度`);
    requireValue(!name.includes('gripper') && name !== 'right_joint' || lo === 0 && hi === .035, '当前夹爪控制要求单指范围为 0–35 mm');
    const home = config.homePose[name];
    requireValue(home === undefined || home >= lo && home <= hi, `${name}: URDF 限位必须包含 ready_pose (${home} rad)`);
    const axis = attribute(child(joint, 'axis'), 'xyz', 3);
    requireValue(Math.hypot(...axis) > 1e-12, `${name}: 关节轴不能为零`);
    const target = named(scene, 'joint', name), actuator = named(child(scene, 'actuator'), 'general', name);
    target.setAttribute('axis', axis.join(' ')); target.setAttribute('range', `${lo} ${hi}`);
    actuator.setAttribute('ctrlrange', `${lo} ${hi}`); actuator.setAttribute('forcerange', `${-effort} ${effort}`);
    config.limits[name] = [lo, hi]; config.velocityLimits[name] = velocity;
  }
  return { xml: new XMLSerializer().serializeToString(scene), config };
}

export function finishBrowserModel(mj, model, data, config) {
  const id = (type, name) => mj.mj_name2id(model, mj.mjtObj[`mjOBJ_${type}`].value, name);
  for (const [name, value] of Object.entries(config.homePose)) data.qpos[model.jnt_qposadr[id('JOINT', name)]] = value;
  mj.mj_forward(model, data);
  const flange = id('BODY', 'link6'), pinch = id('SITE', 'pinch_center');
  const pads = ['left_link_contact', 'right_link_contact'].map(name => id('GEOM', name));
  const center = [0, 1, 2].map(i => (data.geom_xpos[pads[0]*3+i] + data.geom_xpos[pads[1]*3+i]) / 2 - data.xpos[flange*3+i]);
  const r = Array.from(data.xmat.slice(flange*9, flange*9+9));
  model.site_pos.set([0, 1, 2].map(col => center.reduce((sum, v, row) => sum + r[row*3+col]*v, 0)), pinch*3);
  config.originalGraspRotation = r;
  config.gripperClosedGap = Math.hypot(...[0,1,2].map(i => data.geom_xpos[pads[0]*3+i] - data.geom_xpos[pads[1]*3+i])) -
    model.geom_size[pads[0]*3] - model.geom_size[pads[1]*3];
  mj.mj_forward(model, data);
}

export function browserUrdfKey() { return `zekeeparm.urdf:${new URL(import.meta.env.BASE_URL, location.href).pathname}`; }
export function readBrowserUrdf() { return localStorage.getItem(browserUrdfKey()); }
export async function loadBrowserSource() {
  const response = await fetch(`${import.meta.env.BASE_URL}models/browser-model.json?v=${robot.modelVersion}`);
  if (!response.ok) throw new Error(`无法读取网站 URDF（HTTP ${response.status}）`);
  return response.json();
}
