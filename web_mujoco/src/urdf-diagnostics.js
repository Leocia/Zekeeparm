import robot from './robot-config.json' with { type: 'json' };

const direct = (node, tag) => [...node.children].filter(child => child.tagName === tag);
const one = (node, tag) => direct(node, tag)[0];

// DOMParser supplies XML locations; semantic locations follow the same element order in the source.
export function diagnoseUrdf(text, baseline) {
  const errors = [];
  const add = (node, message) => {
    const tag = node?.tagName || 'robot';
    const index = node ? [...node.ownerDocument.getElementsByTagName(tag)].indexOf(node) : 0;
    const visible = text.replace(/<!--[\s\S]*?-->/g, s => s.replace(/[^\n]/g, ' '));
    const offset = [...visible.matchAll(new RegExp(`<${tag}(?=[\\s/>])`, 'g'))][index]?.index || 0;
    errors.push({ line: text.slice(0, offset).split('\n').length, message });
  };
  if (new TextEncoder().encode(text).length > 512 * 1024 || /<!DOCTYPE|<!ENTITY/i.test(text)) {
    return [{ line: 1, message: 'URDF 最大 512 KiB，禁止 DOCTYPE / ENTITY 声明。' }];
  }
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  const parseError = doc.querySelector('parsererror');
  if (parseError) {
    const message = parseError.textContent.replace(/Below is a rendering[\s\S]*/i, '').trim();
    return [{ line: Number(message.match(/(?:line|行)\s*(\d+)/i)?.[1]) || 1, message: `XML: ${message}` }];
  }
  const root = doc.documentElement;
  if (root.tagName !== 'robot') return [{ line: 1, message: '根节点必须为 <robot>。' }];
  const expectedRoot = baseline ? new DOMParser().parseFromString(baseline, 'application/xml').documentElement : null;
  const links = direct(root, 'link'), joints = direct(root, 'joint');
  for (const [tag, nodes] of [['link', links], ['joint', joints]]) {
    const names = new Set();
    for (const node of nodes) {
      const name = node.getAttribute('name');
      if (!name || names.has(name)) add(node, `${tag} 名称为空或重复：${name || '(空)'}。`);
      names.add(name);
    }
    if (expectedRoot) {
      const required = direct(expectedRoot, tag).map(n => n.getAttribute('name'));
      if (names.size !== required.length || required.some(n => !names.has(n))) {
        add(root, `保留当前机械臂的 link/joint 名称与数量；${tag} 缺少：${required.filter(n => !names.has(n)).join(', ') || '无，存在额外节点'}。`);
      }
    }
  }
  for (const name of ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_joint', 'right_joint']) {
    if (!joints.some(n => n.getAttribute('name') === name)) add(root, `缺少必需关节 ${name}。`);
  }
  const linkNames = new Set(links.map(n => n.getAttribute('name'))), parents = new Map();
  for (const joint of joints) {
    const name = joint.getAttribute('name');
    const parent = one(joint, 'parent')?.getAttribute('link'), child = one(joint, 'child')?.getAttribute('link');
    if (!linkNames.has(parent) || !linkNames.has(child) || parent === child) add(joint, `${name}: parent/child 必须引用不同的现有 link。`);
    if (parents.has(child)) add(joint, `${name}: ${child} 存在多个父关节。`);
    parents.set(child, parent);
    if (expectedRoot) {
      const original = direct(expectedRoot, 'joint').find(n => n.getAttribute('name') === name);
      if (original && (joint.getAttribute('type') !== original.getAttribute('type') ||
          ['parent', 'child'].some(side => one(joint, side)?.getAttribute('link') !== one(original, side)?.getAttribute('link')))) {
        add(joint, `${name}: 保留当前关节类型与父子连接关系。`);
      }
    }
    if (joint.getAttribute('type') === 'fixed') continue;
    const limit = one(joint, 'limit');
    const [lo, hi, effort, velocity] = ['lower', 'upper', 'effort', 'velocity'].map(k => {
      const value = limit?.getAttribute(k)?.trim(); return value ? Number(value) : NaN;
    });
    if (![lo, hi, effort, velocity].every(Number.isFinite) || lo >= hi || effort <= 0 || velocity <= 0) {
      add(limit || joint, `${name}: limit 需要有限数值，lower < upper，effort / velocity > 0。`);
    } else if (robot.homePose[name] !== undefined && (robot.homePose[name] < lo || robot.homePose[name] > hi)) {
      add(limit, `${name}: 限位需包含 ready_pose (${robot.homePose[name]} rad)。`);
    }
    const axis = one(joint, 'axis');
    const values = (axis?.getAttribute('xyz') || '').trim().split(/\s+/).map(Number);
    if (values.length !== 3 || !values.every(Number.isFinite) || Math.hypot(...values) < 1e-12) add(axis || joint, `${name}: axis/xyz 需要非零三维向量。`);
    const mimic = one(joint, 'mimic');
    if (name === 'right_joint' && (!mimic || mimic.getAttribute('joint') !== 'gripper_joint' ||
        Number(mimic.getAttribute('multiplier') ?? 1) !== 1 || Number(mimic.getAttribute('offset') ?? 0) !== 0)) {
      add(mimic || joint, 'right_joint: 需要跟随 gripper_joint，multiplier=1，offset=0。');
    } else if (name !== 'right_joint' && mimic) add(mimic, `${name}: 当前模型不支持新增 mimic 关系。`);
  }
  if (links.filter(n => !parents.has(n.getAttribute('name'))).length !== 1) add(root, '拓扑需要且只能有一个根 link。');
  for (const link of links) {
    const seen = new Set(); let name = link.getAttribute('name');
    while (parents.has(name) && !seen.has(name)) { seen.add(name); name = parents.get(name); }
    if (seen.has(name)) { add(link, '关节连接形成闭环。'); break; }
    const mass = link.querySelector('inertial > mass');
    if (mass && (!Number.isFinite(Number(mass.getAttribute('value'))) || Number(mass.getAttribute('value')) <= 0)) add(mass, `${link.getAttribute('name')}: mass 必须为正数。`);
  }
  for (const node of doc.getElementsByTagName('*')) {
    for (const [key, count] of [['xyz', 3], ['rpy', 3], ['rgba', 4]]) {
      if (!node.hasAttribute(key)) continue;
      const values = node.getAttribute(key).trim().split(/\s+/);
      if (values.length !== count || values.some(v => v === '' || !Number.isFinite(Number(v)))) add(node, `${node.tagName}/${key} 需要 ${count} 个有限数值。`);
    }
  }
  return errors;
}
