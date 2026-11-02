import assert from 'node:assert/strict';
import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import { bindJoints, ARM_JOINTS } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';
import { createTaskMotion } from '../src/task-motion.js';
import { createGraspDemo, STORAGE_ZONES, STACK_TARGETS } from '../src/grasp-demo.js';
import { createObjectPositionController } from '../src/object-position-control.js';
import { Scene } from 'three';
import { createTeachingView } from '../src/teaching-view.js';

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
const motion = createTaskMotion(mj, model, data, physics, ik);
const diagnostics = [];
motion.onDiagnostics(event => diagnostics.push(event));
let previous = '';
const demo = createGraspDemo({ mujoco: mj, model, data, joints, physics, ik, motion,
  onChange(state) {
    if (previous !== state.stage) console.log(state.stage, state.message, state.detours, motion.blocker());
    previous = state.stage;
  } });
const obstacle = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'task_obstacle_geom');
const buffer = new mj.DoubleBuffer(6);
let minimum = 1;
function run() {
  for (let frame = 0; demo.isRunning() && frame < 18000; frame++) {
    demo.update(); physics.step(12);
    for (let i = 0; i < model.ngeom; i++) {
      if (model.geom_group[i] !== 3 && !['red_cube_geom', 'blue_block_geom', 'yellow_cylinder_geom'].includes(
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM.value, i))) continue;
      minimum = Math.min(minimum, mj.mj_geomDistance(model, data, i, obstacle, .1, buffer));
    }
  }
  assert.equal(demo.isRunning(), false, 'task must terminate');
  assert.ok(minimum > .002, `actual obstacle clearance: ${minimum * 1000} mm`);
}
try {
  physics.step(1000);
  assert.throws(() => motion.configure(true, NaN, 0));
  assert.throws(() => motion.configure(true, .24, -.25), /过近/);
  motion.configure(true, .30, -.30);
  const beforeMeasure = Array.from(data.qpos);
  const nearest = motion.proximity();
  assert.ok(nearest.distance > .012 && nearest.distance < 1);
  assert.equal(nearest.clearance, .012);
  assert.ok(nearest.from.every(Number.isFinite) && nearest.to.every(Number.isFinite));
  assert.ok(Math.abs(Math.hypot(...nearest.from.map((v, i) => v - nearest.to[i])) - nearest.distance) < 1e-7);
  assert.deepEqual(Array.from(data.qpos), beforeMeasure, 'proximity must not change the simulation');
  const edit = createObjectPositionController({ mujoco: mj, model, data });
  edit.setEnabled(true);
  assert.equal(edit.moveTo(.30, -.30).reason, 'overlap', 'object editing must reject the obstacle');
  edit.setEnabled(false);
  assert.ok(motion.valid(ARM_JOINTS.map(j => j.home)), 'ready pose must be valid');
  demo.start('red'); run();
  assert.equal(demo.state().stage, 'complete', JSON.stringify(demo.state()));
  assert.ok(demo.state().detours > 0, 'the barrier must require a detour');
  const detour = diagnostics.find(event => event.type === 'planned' && event.direct.length);
  assert.ok(detour, 'diagnostics must expose a real rejected direct route and safe detour');
  assert.ok(detour.safe.length > 2 && detour.waypoints.length > 0);
  assert.equal(detour.safe.length, detour.samples + 1);
  assert.ok(detour.safe.flat().every(Number.isFinite));
  assert.ok(Number.isFinite(detour.elapsed) && detour.elapsed > 0);
  assert.ok(diagnostics.some(event => event.type === 'rejected' && event.sample > 0));
  assert.ok(detour.directFailure?.reason, 'a rejected route must have its actual failure reason');
  const scene = new Scene();
  const visuals = createTeachingView(scene, 1, () => {});
  visuals.setTrace(detour);
  const lines = scene.children.flatMap(group => group.children || []).filter(object => object.isLine);
  const red = lines.find(line => line.material.isLineDashedMaterial);
  const green = lines.find(line => !line.material.isLineDashedMaterial);
  assert.equal(red.geometry.getAttribute('position').count, detour.direct.length);
  assert.equal(green.geometry.getAttribute('position').count, detour.safe.length);
  assert.ok(Array.from(red.geometry.getAttribute('position').array).every((value, i) => Math.abs(value - detour.direct.flat()[i]) < 1e-7));
  visuals.dispose();
  assert.equal(scene.children.length, 0, 'clearing the teaching scene must release its overlays');
  const body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'red_cube');
  assert.ok(Math.hypot(data.xpos[body * 3] - STORAGE_ZONES.red.x,
    data.xpos[body * 3 + 1] - STORAGE_ZONES.red.y) < .012, 'avoidance must still place the object');
  for (const target of ['blue', 'yellow', 'stack', 'red']) {
    demo.reset(); physics.reset(); physics.step(1000);
    motion.configure(true, .30, -.30, target === 'red' ? [.04, .08, .16] : [.05, .09, .18]);
    if (target === 'stack') demo.startStack(); else demo.start(target);
    run();
    assert.equal(demo.state().stage, 'complete', target + ': ' + JSON.stringify(demo.state()));
    for (const object of demo.targets.filter(object => target === 'stack' || object.id === target)) {
      const id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, object.body);
      const expected = target === 'stack' ? STACK_TARGETS[object.id] : STORAGE_ZONES[object.id];
      assert.ok(Math.hypot(data.xpos[id * 3] - expected.x, data.xpos[id * 3 + 1] - expected.y) < .015);
      assert.ok(Math.abs(data.xpos[id * 3 + 2] - expected.z) < .015);
    }
  }
  demo.reset(); physics.reset(); physics.step(1000);
  motion.configure(true, .45, -.18, [.05, .09, .18]);
  physics.reset(); motion.restore(); physics.step(1000);
  assert.deepEqual(motion.position(), [.45, -.18, .09], 'reset must preserve the applied obstacle position');
  demo.start('red'); run();
  assert.equal(demo.state().stage, 'failed');
  assert.equal(demo.state().message, 'no-safe-path', 'blocked destination must stop the task');
  assert.ok(diagnostics.some(event => event.type === 'failed' && event.direct.length === 2));
  demo.reset(); physics.reset(); physics.step(1000);
  motion.configure(true, .30, -.30);
  demo.start('red'); demo.update(); demo.cancel();
  const stopped = { ...physics.targets };
  demo.update(); physics.step(100);
  assert.deepEqual(physics.targets, stopped, 'cancel must discard planned motion');
  console.log('PASS: obstacle detour, all three placements, stack, resized obstacle task, blocked destination, overlap rejection, cancellation;',
    'minimum actual clearance', (minimum * 1000).toFixed(2), 'mm');
} finally { motion.dispose(); buffer.delete(); data.delete(); model.delete(); vfs.delete(); }
