import assert from 'node:assert/strict';
import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import { createObjectPositionController } from '../src/object-position-control.js';
import { Physics } from '../src/physics.js';
import { bindJoints, ARM_JOINTS } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';
import { createGraspDemo, STORAGE_ZONES, STACK_TARGETS } from '../src/grasp-demo.js';
import { createContactTelemetry } from '../src/contact-telemetry.js';

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
mj.mj_forward(model, data);
const native = new Physics(mj, model, data);
const joints = bindJoints(mj, model);
const physics = createPhysicsController(mj, model, data, joints);
const ik = createTcpIk(mj, model, data, joints);
const telemetry = createContactTelemetry(mj, model, data);
let lastStage = '';
let deepestContact = 0;
let fastestJoint = 0;
const visitedStages = new Set();
let holdReturn = false;
const demo = createGraspDemo({ mujoco: mj, model, data, joints, physics, ik,
  onChange: state => {
    visitedStages.add(state.stage);
    if (holdReturn && state.stage === 'returning') {
      physics.setTargets(Object.fromEntries(ARM_JOINTS.map(joint =>
        [joint.name, data.qpos[joints.byName[joint.name].qposadr]])));
    }
    const signature = state.selectedId + ':' + state.stage;
    if (signature !== lastStage) console.log(signature, state.message);
    lastStage = signature;
  } });
const bodyPosition = name => {
  const id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, name);
  return Array.from(data.xpos.slice(id * 3, id * 3 + 3));
};
function run(expectedStage = 'complete') {
  visitedStages.clear();
  for (let frame = 0; demo.isRunning() && frame < 18000; frame++) {
    demo.update(); physics.step(12);
    fastestJoint = Math.max(fastestJoint, ...native.v.map(v => Math.abs(data.qvel[v])));
    const contacts = data.contact;
    try {
      for (let i = 0; i < contacts.size(); i++) {
        const contact = contacts.get(i);
        deepestContact = Math.min(deepestContact, contact.dist);
        contact.delete();
      }
    } finally { contacts.delete(); }
  }
  assert.equal(demo.state().stage, expectedStage, JSON.stringify(demo.state()));
  assert.ok(visitedStages.has('returning'), 'task must return to the ready pose');
  assert.ok(![...visitedStages].some(stage => /celebrate|shy/.test(stage)), 'no head gestures');
  if (expectedStage === 'complete') {
    for (const joint of ARM_JOINTS) {
      assert.ok(Math.abs(data.qpos[joints.byName[joint.name].qposadr] - joint.home) < .02,
        joint.name + ' must reach ready pose before task completion');
      assert.equal(physics.targets[joint.name], joint.home);
    }
  }
}
try {
  const floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'floor');
  const table = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'task_table_geom');
  const tableBottom = data.geom_xpos[table * 3 + 2] - model.geom_size[table * 3 + 2];
  assert.ok(data.geom_xpos[floor * 3 + 2] < tableBottom - .001,
    'floor must stay below the tabletop to prevent coplanar surface flicker');
  assert.equal(model.ncam, 2, 'overhead plus one Gemini 305 output');
  const leftEye = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA.value, 'wrist_rgb');
  const leftLens = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'wrist_camera_left_lens');
  const rightLens = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'wrist_camera_right_lens');
  assert.ok(Math.abs(Math.hypot(...[0,1,2].map(i => model.geom_pos[leftLens * 3 + i] - model.geom_pos[rightLens * 3 + i])) - .018) < 1e-6, 'Gemini 305 stereo housing baseline');
  assert.ok(Math.abs(model.cam_fovy[leftEye] - 68) < 1e-6);
  const flange = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'link6');
  const viewAxis = [0,1,2].map(i => -data.cam_xmat[leftEye * 9 + i * 3 + 2]);
  const localViewAxis = [0,1,2].map(axis => viewAxis.reduce(
    (sum, value, i) => sum + value * data.xmat[flange * 9 + i * 3 + axis], 0));
  assert.ok(Math.abs(Math.atan2(-localViewAxis[1], localViewAxis[2]) - Math.PI / 12) < 1e-6,
    'wrist camera must point 15 degrees about flange +X');
  assert.ok(Math.abs(localViewAxis[0]) < 1e-6, 'wrist camera must not tilt about flange Y');
  assert.equal(model.nmesh, 10, 'only the ten bundled arm meshes must be loaded');
  for (let i = 0; i < model.nmesh; i++) {
    assert.ok(mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH.value, i).startsWith('zk_'));
  }
  assert.deepEqual(native.limits, [[-2.58,2.58],[0,3.7],[-0.01,3.7],[-1.57,1.57],[-1.57,1.57],[-1.57,1.57]]);
  assert.equal(joints.byName.joint7.max, .035);
  assert.throws(() => native.setGripper(.071));
  assert.throws(() => native.setTargets([3,0,0,0,0,0]));
  assert.throws(() => native.solve([9,0,0]));
  physics.reset(); physics.step(20000);
  for (const joint of ARM_JOINTS) assert.ok(Math.abs(data.qpos[joints.byName[joint.name].qposadr] - joint.home) < .02, joint.name + ' idle stability');
  assert.ok(native.v.every(v => Math.abs(data.qvel[v]) < .02), 'robot must stay still while idle');
  for (const target of demo.targets) {
    physics.reset(); demo.reset(); physics.step(1000);
    const borders = ['n', 's', 'e', 'w'].map(side =>
      mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, `${target.id}_zone_${side}`));
    for (const [axis, index] of [['x', 0], ['y', 1]]) {
      const center = borders.reduce((sum, geom) => sum + data.geom_xpos[geom * 3 + index], 0) / 4;
      assert.ok(Math.abs(STORAGE_ZONES[target.id][axis] - center) < 1e-6,
        target.id + ' placement target must follow the storage frame');
    }
    demo.start(target.id); run();
    const actual = bodyPosition(target.body), expected = STORAGE_ZONES[target.id];
    assert.ok(Math.hypot(actual[0]-expected.x, actual[1]-expected.y)<.012, target.id+' storage position');
    assert.ok(Math.abs(actual[2]-expected.z)<.015, target.id+' storage height');
    assert.ok(Number.isFinite(telemetry.sample(target.id).graspForce));
  }
  for (const yaw of [Math.PI / 6, -Math.PI / 4]) {
    physics.reset(); demo.reset(); physics.step(1000);
    const edit = createObjectPositionController({ mujoco: mj, model, data });
    edit.setSelected('blue'); edit.setEnabled(true);
    assert.ok(edit.rotateTo(yaw).ok);
    edit.setEnabled(false); physics.step(500);
    demo.start('blue'); run();
    const actual = bodyPosition('blue_block');
    assert.ok(Math.hypot(actual[0] - STORAGE_ZONES.blue.x, actual[1] - STORAGE_ZONES.blue.y) < .012);
  }
  physics.reset(); demo.reset(); physics.step(1000);
  demo.startStack(); run();
  for (const target of demo.targets) {
    const actual = bodyPosition(target.body), expected = STACK_TARGETS[target.id];
    assert.ok(Math.hypot(actual[0]-expected.x, actual[1]-expected.y)<.015, target.id+' stack position');
    assert.ok(Math.abs(actual[2]-expected.z)<.015, target.id+' stack height');
  }
  physics.reset(); demo.reset(); physics.step(1000);
  holdReturn = true;
  demo.start('red'); run('failed');
  holdReturn = false;
  assert.equal(demo.state().message, 'return-timeout', 'blocked return must fail rather than report completion');
  demo.reset(); demo.start('red'); demo.update(); demo.cancel();
  const stopped = {...physics.targets}; physics.step(50); demo.update();
  assert.equal(demo.isRunning(), false); assert.deepEqual(physics.targets,stopped);
  assert.ok(deepestContact > -.0015, 'penetration exceeds 1.5 mm: ' + (-deepestContact * 1000).toFixed(3));
  assert.ok(fastestJoint < 5, 'joint motion must remain bounded, measured ' + fastestJoint);
  physics.reset();
  data.qvel[native.v[0]] = 30;
  assert.throws(() => native.step(), /物理状态异常/);
  physics.reset();
  console.log('Measured maximum penetration:',(-deepestContact * 1000).toFixed(3),'mm; maximum joint speed:',fastestJoint.toFixed(3),'rad/s');
  console.log('PASS: user joint limits, gripper mapping, all three physical put-away tasks, stack, cancellation, telemetry');
} finally { native.dispose(); data.delete(); model.delete(); vfs.delete(); }
