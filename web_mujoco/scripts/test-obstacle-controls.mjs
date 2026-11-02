import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { createServer } from 'vite';
import { bindJoints } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';
import { createTaskMotion } from '../src/task-motion.js';

class Element extends EventTarget { value = ''; checked = false; disabled = false; textContent = ''; }
const elements = Object.fromEntries(['fields', 'enabled', 'x', 'y', 'length', 'width', 'height', 'status', 'form', 'demo'].map(name =>
  ['obstacle-' + name, new Element()]));
globalThis.document = { documentElement: {}, getElementById: name => elements[name] };
Object.defineProperty(globalThis, 'localStorage', { value: { getItem: () => null } });
const server = await createServer({ root: fileURLToPath(new URL('..', import.meta.url)), configFile: false,
  optimizeDeps: { noDiscovery: true, include: [] },
  server: { middlewareMode: true, hmr: false, watch: null }, logLevel: 'error' });
const { bindObstacleControls } = await server.ssrLoadModule('/src/obstacle-controls.js');
const mj = await loadMujoco(), vfs = new mj.MjVFS();
const root = new URL('../public/models/', import.meta.url);
const xml = fs.readFileSync(new URL('rs_grasp_scene.xml', root));
vfs.addBuffer('scene.xml', new Uint8Array(xml));
for (const match of xml.toString().matchAll(/file="([^"]+\.STL)"/g)) {
  const file = 'meshes/' + match[1];
  vfs.addBuffer(file, new Uint8Array(fs.readFileSync(new URL(file, root))));
}
const model = mj.MjModel.from_xml_path('scene.xml', vfs), data = new mj.MjData(model);
const joints = bindJoints(mj, model), physics = createPhysicsController(mj, model, data, joints);
const ik = createTcpIk(mj, model, data, joints), motion = createTaskMotion(mj, model, data, physics, ik);
physics.step(1000);
let locked = false, interaction, started = 0;
const controls = bindObstacleControls({ motion, view: { setObstacleInteraction: value => { interaction = value; } },
  isLocked: () => locked, beforeMove: () => { started++; } });
const inputX = elements['obstacle-x'], inputY = elements['obstacle-y'], enabled = elements['obstacle-enabled'];
const sizes = ['length', 'width', 'height'].map(name => elements[`obstacle-${name}`]);
const obstacle = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'task_obstacle_geom');
function apply(x, y, next = true, dimensions) {
  dimensions?.forEach((v, i) => { sizes[i].value = String(v); });
  inputX.value = String(x); inputY.value = String(y); enabled.checked = next;
  elements['obstacle-form'].dispatchEvent(new Event('submit', { cancelable: true }));
}
try {
  apply(350.5, -300.5);
  assert.equal(motion.isEnabled(), true);
  assert.equal(inputX.value, '350.5');
  assert.ok(Math.abs(motion.position()[0] - .3505) < 1e-12, 'fractional millimetres must apply');
  assert.equal(interaction.enabled, true);
  interaction.onStart(); interaction.onMove({ x: .4, y: -.3 });
  assert.deepEqual(motion.position(), [.4, -.3, .09]);
  assert.equal(inputX.value, '400.0'); assert.equal(inputY.value, '-300.0');
  assert.equal(started, 2, 'form apply and drag start must stop TCP dragging');
  interaction.onMove({ x: .24, y: -.25 });
  assert.deepEqual(motion.position(), [.4, -.3, .09], 'overlap must retain the last valid position');
  assert.equal(inputX.value, '400.0', 'failed move must show actual, not rejected coordinates');
  assert.match(elements['obstacle-status'].textContent, /红/);
  apply(100, -300);
  assert.equal(inputX.value, '400.0');
  assert.match(elements['obstacle-status'].textContent, /150/);
  const staleMove = interaction.onMove;
  locked = true; controls.refresh();
  assert.equal(elements['obstacle-fields'].disabled, true);
  assert.equal(interaction.enabled, false);
  staleMove({ x: .45, y: -.3 });
  assert.deepEqual(motion.position(), [.4, -.3, .09], 'task lock must reject an in-flight drag event');
  locked = false; controls.refresh();
  apply(400, -300, false);
  apply(240, -250, true);
  assert.equal(enabled.checked, false, 'failed enabling must restore the actual checkbox state');
  assert.equal(interaction.enabled, false);
  apply(400, -300, true);
  inputX.value = '500';
  physics.reset(); motion.restore(); controls.reset();
  assert.deepEqual(motion.position(), [.4, -.3, .09]);
  assert.equal(inputX.value, '400.0', 'reset must display the applied obstacle position');
  apply(400, -300, true, [80.5, 60, 120]);
  assert.deepEqual(motion.size(), [.0805, .06, .12]);
  assert.deepEqual(motion.position(), [.4, -.3, .06], 'resizing must keep the bottom on the table');
  assert.deepEqual(sizes.map(input => input.value), ['80.5', '60.0', '120.0']);
  interaction.onMove({ x: .42, y: -.3 });
  assert.deepEqual(motion.size(), [.0805, .06, .12], 'dragging must preserve applied dimensions');
  apply(420, -300, true, [400, 400, 400]);
  assert.deepEqual(motion.size(), [.0805, .06, .12], 'overlapping resize must restore the previous dimensions');
  assert.deepEqual(motion.position(), [.42, -.3, .06]);
  assert.match(elements['obstacle-status'].textContent, /红/);
  for (const bad of [0, -10, 401, NaN, Infinity]) {
    apply(420, -300, true, [bad, 60, 120]);
    assert.deepEqual(motion.size(), [.0805, .06, .12]);
    assert.equal(sizes[0].value, '80.5');
    assert.match(elements['obstacle-status'].textContent, /10–400/);
  }
  locked = true;
  apply(420, -300, true, [100, 60, 120]);
  assert.deepEqual(motion.size(), [.0805, .06, .12], 'tasks must lock dimensions too');
  locked = false;
  physics.reset(); motion.restore(); controls.reset();
  assert.deepEqual(motion.position(), [.42, -.3, .06]);
  assert.equal(sizes[0].value, '80.5', 'reset must retain the applied dimensions');
  assert.deepEqual(Array.from(model.geom_aabb.slice(obstacle * 6, obstacle * 6 + 6)), [0, 0, 0, .04025, .03, .06]);
  // Move the red cube into the enlarged edge, outside the original box's collision bounds.
  apply(420, -300, true, [200, 60, 120]);
  const red = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'red_cube');
  const address = model.jnt_qposadr[model.body_jntadr[red]];
  const saved = data.qpos.slice();
  data.qpos.set([.53, -.3, .03, 1, 0, 0, 0], address);
  mj.mj_forward(model, data);
  let contactFound = false;
  const contacts = data.contact;
  try {
    for (let i = 0; i < contacts.size(); i++) {
      const contact = contacts.get(i);
      try { if ([contact.geom1, contact.geom2].includes(obstacle) && contact.dist < 0) contactFound = true; }
      finally { contact.delete(); }
    }
  } finally { contacts.delete(); }
  assert.ok(contactFound, 'actual physics contacts must use the resized bounds and enabled collision masks');
  data.qpos.set(saved); mj.mj_forward(model, data);
  apply(420, -300, false);
  assert.equal(model.body_contype[model.geom_bodyid[obstacle]], 0);
  console.log('PASS: decimal XY/size apply, drag, resize collision rollback, invalid dimensions, task lock, reset, resized physics contacts');
} finally { await server.close(); motion.dispose(); data.delete(); model.delete(); vfs.delete(); }
