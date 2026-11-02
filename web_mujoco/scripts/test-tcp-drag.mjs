import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as THREE from 'three';
import loadMujoco from '@mujoco/mujoco';
import { createServer } from 'vite';
import { bindJoints } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';

class Element extends EventTarget {
  classList = { add() {}, remove() {}, toggle() {} };
  style = {};
  clientWidth = 900;
  captures = new Set();
  setPointerCapture(id) { this.captures.add(id); }
  hasPointerCapture(id) { return this.captures.has(id); }
  releasePointerCapture(id) { this.captures.delete(id); }
}
globalThis.window = new EventTarget();
globalThis.document = Object.assign(new EventTarget(), { documentElement: new Element(), body: new Element() });
Object.defineProperty(globalThis, 'localStorage', { value: { getItem: () => null } });
const server = await createServer({ root: fileURLToPath(new URL('..', import.meta.url)), configFile: false,
  optimizeDeps: { noDiscovery: true, include: [] },
  server: { middlewareMode: true, hmr: false, watch: null }, logLevel: 'error' });
const { createTcpDrag } = await server.ssrLoadModule('/src/tcp-drag.js');
const mj = await loadMujoco();
const root = new URL('../public/models/', import.meta.url);
const vfs = new mj.MjVFS();
const xml = fs.readFileSync(new URL('rs_grasp_scene.xml', root));
vfs.addBuffer('scene.xml', new Uint8Array(xml));
for (const match of xml.toString().matchAll(/file="([^"]+\.STL)"/g)) {
  const file = 'meshes/' + match[1];
  vfs.addBuffer(file, new Uint8Array(fs.readFileSync(new URL(file, root))));
}
const model = mj.MjModel.from_xml_path('scene.xml', vfs);
const data = new mj.MjData(model);
const joints = bindJoints(mj, model);
const physics = createPhysicsController(mj, model, data, joints);
const ik = createTcpIk(mj, model, data, joints);
const marker = new Element();
let hit, orbitEnabled = true;
const drag = createTcpDrag({ ik, physics, markerEl: marker, hostEl: new Element(),
  clusterEl: new Element(), toggleEl: new Element(), openEl: new Element(), closeEl: new Element(),
  panel: { setTarget() {}, closeChips() {} }, onStatus() {},
  view: { camera: new THREE.PerspectiveCamera(), projectWorld: () => ({ x: 450, y: 300 }),
    setOrbitEnabled(value) { orbitEnabled = value; }, setDragVisuals() {}, intersectPlane: () => hit } });
function pointer(host, type, pointerType, id = 1) {
  const event = new Event(type, { cancelable: true });
  Object.assign(event, { pointerType, pointerId: id, button: 0, clientX: 400, clientY: 300 });
  host.dispatchEvent(event);
}
try {
  for (const pointerType of ['mouse', 'touch']) {
    physics.reset(); physics.step(1000);
    drag.setEnabled(true);
    const start = ik.tcpPosition();
    hit = new THREE.Vector3(start.x + .025, start.y, start.z + .02);
    pointer(marker, 'pointerdown', pointerType);
    assert.equal(orbitEnabled, false);
    pointer(window, 'pointermove', pointerType);
    // A held pointer emits no more moves: the animation loop must continue advancing IK.
    const now = performance.now();
    for (let frame = 0; frame < 1000; frame++) { drag.update(now + frame * 16); physics.step(4); }
    const actual = ik.tcpPosition();
    const error = Math.hypot(actual.x - hit.x, actual.y - hit.y, actual.z - hit.z);
    assert.ok(error < .006, `${pointerType} held drag must reach target; error ${error * 1000} mm`);
    pointer(window, 'pointerup', pointerType);
    assert.equal(orbitEnabled, true);
    drag.stop(); drag.setEnabled(false);
  }
  physics.reset(); physics.step(1000); drag.setEnabled(true);
  const start = ik.tcpPosition();
  hit = new THREE.Vector3(start.x + .025, start.y, start.z + .02);
  pointer(marker, 'pointerdown', 'mouse'); pointer(window, 'pointermove', 'mouse');
  pointer(window, 'pointerup', 'mouse');
  marker.dispatchEvent(new Event('lostpointercapture'));
  const released = { ...physics.targets };
  drag.update(performance.now() + 100);
  assert.notDeepEqual(physics.targets, released, 'normal release must retain the settling motion');
  drag.stop();
  for (const interruption of ['lostpointercapture', 'blur', 'visibilitychange', 'disable']) {
    physics.reset(); physics.step(1000);
    drag.setEnabled(true);
    const start = ik.tcpPosition();
    hit = new THREE.Vector3(start.x + .025, start.y, start.z + .02);
    pointer(marker, 'pointerdown', 'mouse'); pointer(window, 'pointermove', 'mouse');
    if (interruption === 'disable') drag.setEnabled(false);
    else if (interruption === 'visibilitychange') {
      document.hidden = true; document.dispatchEvent(new Event(interruption)); document.hidden = false;
    } else (interruption === 'blur' ? window : marker).dispatchEvent(new Event(interruption));
    assert.equal(orbitEnabled, true, interruption + ' must unlock orbit');
    assert.equal(marker.hasPointerCapture(1), false, interruption + ' must release capture');
    const stopped = { ...physics.targets };
    drag.update(performance.now() + 100);
    assert.deepEqual(physics.targets, stopped, interruption + ' must stop advancing IK');
  }
  console.log('PASS: mouse/touch TCP drag reaches a 32 mm target; held motion, capture loss, blur and visibility cleanup');
} finally { await server.close(); data.delete(); model.delete(); vfs.delete(); }
