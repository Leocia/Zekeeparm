const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const policy = require('../public/js/ros/rebot-control-policy.js');
const THREE = require('../public/lib/three-r128.min.js');
const source = name => fs.readFileSync(path.join(__dirname, '../public/js', name), 'utf8');

function fixture(connected = true) {
  const fields = new Map(['x', 'y', 'z', 'duration', 'status'].map(name => [`ros-pose-${name === 'status' ? 'preview-status' : name}`, {
    value: name === 'duration' ? '2' : name === 'x' ? '.3' : name === 'z' ? '.3' : '0',
    textContent: '', events: {}, addEventListener(type, callback) { this.events[type] = callback; },
  }]));
  for (const id of ['ros-pose-preview', 'ros-pose-current', 'ros-pose-limits', 'ros-vlim']) {
    fields.set(id, {value: '.3', events: {}, addEventListener(type, callback) { this.events[type] = callback; }});
  }
  const events = {}, timers = new Map(), calls = [], drawings = [], pending = [];
  let clears = 0;
  const angles = Object.fromEntries(Array.from({length: 6}, (_, i) => [`joint${i + 1}`, 0]));
  const definitions = Object.keys(angles).map(name => ({name, min: -2, max: 3.7}));
  const client = {connected, addEventListener(type, callback) { events[type] = callback; },
    previewPoseIK(...args) { calls.push(args); return new Promise(resolve => pending.push(resolve)); }};
  const window = {reBotRos: client, ReBotControlPolicy: policy,
    rebotI18n: {t: (key, values) => key + JSON.stringify(values || {})},
    reBotSim: {getAngles: () => ({...angles}), getJointDefs: () => definitions,
      getPosePositionMax: () => ({x: .65, y: .65, z: .74}),
      getTcpPose: () => ({orientation: {x: 0, y: 0, z: 0, w: 1}}),
      previewPose(...args) { drawings.push(args); }, clearPosePreview() { clears += 1; }},
    setTimeout(callback) { const id = Symbol(); timers.set(id, callback); return id; },
    clearTimeout(id) { timers.delete(id); }, addEventListener(type, callback) { events[type] = callback; }};
  vm.runInNewContext(source('ros/rebot-pose-preview.js'), {window, document: {getElementById: id => fields.get(id)}});
  const runTimer = () => {const callback = [...timers.values()].at(-1); timers.clear(); return callback();};
  const result = q => ({error_code: {val: 1}, solution: {joint_state: {name: Object.keys(angles), position: Array(6).fill(q)}}});
  const preview = () => fields.get('ros-pose-preview').events.click();
  return {fields, calls, drawings, pending, runTimer, result, window, angles, events, timers, preview, get clears() { return clears; }};
}

test('only Preview Target starts pose preview; editing and lifecycle events do not', async () => {
  const html = fs.readFileSync(path.join(__dirname, '../public/index.html'), 'utf8');
  assert.doesNotMatch(html, /id="physics-|rebot-physics\.js|MuJoCo 物理仿真/);
  assert.match(html, /<button type="button" id="ros-pose-preview"/);
  const f = fixture();
  f.events['rebot-model-ready']();
  f.events.status();
  for (const name of ['x', 'y', 'z', 'duration']) {
    f.fields.get(`ros-pose-${name}`).events.input();
    f.fields.get(`ros-pose-${name}`).events.change();
  }
  f.fields.get('ros-vlim').events.input();
  assert.equal(f.timers.size, 0);
  assert.equal(f.calls.length, 0);
  assert.equal(f.drawings.length, 0);
  f.preview();
  const work = f.runTimer();
  assert.equal(f.calls.length, 1);
  f.pending.shift()(f.result(.2));
  await work;
  assert.equal(f.drawings.at(-1)[2].joint1, .2);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewReady/);
  f.fields.get('ros-pose-duration').value = '6';
  const count = f.drawings.length;
  const clears = f.clears;
  f.fields.get('ros-pose-duration').events.input();
  assert.equal(f.drawings.length, count);
  assert.equal(f.clears, clears + 1);
  assert.equal(f.timers.size, 0);
  f.preview();
  const next = f.runTimer(); f.pending.shift()(f.result(.2)); await next;
  assert.equal(f.drawings.at(-1)[1], 6);
  assert.equal(f.window.reBotPosePreview.getTarget().duration, 6);
  assert.equal(f.angles.joint1, 0);
});

test('Get Current Pose fills XYZ only and cancels the old preview without requesting motion', async () => {
  const f = fixture();
  f.preview(); const old = f.runTimer();
  f.window.reBotSim.getTcpPose = () => ({position: {x: .12345, y: -.23456, z: .34567},
    orientation: {x: 0, y: 0, z: 0, w: 1}});
  const count = f.drawings.length;
  f.fields.get('ros-pose-current').events.click();
  assert.deepEqual(['x', 'y', 'z', 'duration'].map(name => f.fields.get(`ros-pose-${name}`).value),
    ['0.12', '-0.23', '0.35', '2']);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /currentFilled/);
  assert.equal(f.timers.size, 0);
  assert.equal(f.calls.length, 1);
  f.pending[0](f.result(.2)); await old;
  assert.equal(f.drawings.length, count);
  assert.equal(f.window.reBotPosePreview.getTarget().pose.position.y, -.23);
  for (const pose of [null, {position: {x: NaN, y: 0, z: 0}}]) {
    f.window.reBotSim.getTcpPose = () => pose;
    f.fields.get('ros-pose-current').events.click();
    assert.equal(f.fields.get('ros-pose-x').value, '0.12');
    assert.match(f.fields.get('ros-pose-preview-status').textContent, /currentUnavailable/);
  }
});

test('pose bounds follow the loaded model without starting a preview', () => {
  const f = fixture();
  assert.deepEqual(['x', 'y', 'z'].map(axis => f.fields.get(`ros-pose-${axis}`).max), ['0.65', '0.65', '0.74']);
  assert.deepEqual(['x', 'y', 'z'].map(axis => f.fields.get(`ros-pose-${axis}`).min), ['-0.65', '-0.65', '0']);
  f.window.reBotSim.getPosePositionMax = () => null;
  assert.equal(f.window.reBotPosePreview.getTarget(), null);
  f.window.reBotSim.getPosePositionMax = () => ({x: .7, y: .7, z: .8});
  f.events['rebot-model-ready']();
  assert.deepEqual(['x', 'y', 'z'].map(axis => f.fields.get(`ros-pose-${axis}`).max), ['0.7', '0.7', '0.8']);
  assert.deepEqual(['x', 'y', 'z'].map(axis => f.fields.get(`ros-pose-${axis}`).min), ['-0.7', '-0.7', '0']);
  assert.match(f.fields.get('ros-pose-limits').textContent, /positionLimits/);
  assert.equal(f.drawings.length, 0);
  assert.equal(f.calls.length, 0);
});

test('out-of-range targets keep a red point but never request IK or execute', async () => {
  const checkIk = 'async ' + functionSource(source('ros/rebot-ros-ui.js'), 'checkIk');
  for (const connected of [false, true]) {
    const f = fixture(connected), messages = [];
    const context = vm.createContext({window: f.window, teachingHardwareBusy: false,
      els: {poseX: f.fields.get('ros-pose-x'), poseY: f.fields.get('ros-pose-y'), poseZ: f.fields.get('ros-pose-z'),
        control: {checked: true}}, client: {connected}, t: key => key, setMessage: message => messages.push(message),
      controlAllowed: () => assert.fail('Out-of-range target reached execution'),
      moveToPoseViaIkTrajectory: () => assert.fail('Out-of-range target reached IK')});
    f.window.reBotSim.moveToTcp = () => assert.fail('Out-of-range target reached simulation');
    vm.runInContext(checkIk, context);
    for (const [axis, boundary, outside] of [
      ['x', .65, .6501], ['y', .65, .6501], ['z', .74, .7401],
      ['x', -.65, -.6501], ['y', -.65, -.6501], ['z', 0, -.0001],
    ]) {
      const field = f.fields.get(`ros-pose-${axis}`);
      const saved = field.value;
      field.value = String(boundary);
      assert.ok(f.window.reBotPosePreview.getTarget());
      field.value = String(outside);
      assert.equal(f.window.reBotPosePreview.getTarget(), null);
      const drawings = f.drawings.length;
      f.preview();
      assert.equal(f.drawings.length, drawings + 1);
      assert.equal(f.drawings.at(-1)[0][axis], outside);
      assert.equal(f.drawings.at(-1)[2], null);
      assert.equal(f.drawings.at(-1)[3], true);
      assert.equal(f.calls.length, 0);
      assert.equal(f.timers.size, 0);
      await context.checkIk();
      assert.equal(messages.at(-1), 'action.previewInvalid');
      field.value = saved;
    }
    f.fields.get('ros-pose-y').value = '-0.3';
    assert.ok(f.window.reBotPosePreview.getTarget());
  }
});

test('stale IK replies and invalid values cannot replace the current preview', async () => {
  const f = fixture();
  f.preview();
  const old = f.runTimer();
  f.fields.get('ros-pose-x').value = '.4'; f.fields.get('ros-pose-x').events.input();
  const beforeReply = f.drawings.length;
  f.pending[0](f.result(.2)); await old;
  assert.equal(f.drawings.length, beforeReply);
  assert.equal(f.fields.get('ros-pose-preview-status').textContent, '');
  f.preview();
  const next = f.runTimer();
  f.pending[1](f.result(.4)); await next;
  assert.equal(f.drawings.at(-1)[0].x, .4);
  f.fields.get('ros-pose-x').value = ''; f.fields.get('ros-pose-x').events.input();
  assert.equal(f.window.reBotPosePreview.getTarget(), null);
  const count = f.drawings.length;
  f.preview();
  assert.equal(f.drawings.length, count);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewInvalid/);
  assert.ok(f.clears > 0);
});

test('offline and failed IK show target point with explicit status', async () => {
  const offline = fixture(false);
  assert.equal(offline.drawings.length, 0);
  offline.preview();
  assert.equal(offline.drawings.length, 1);
  assert.equal(offline.calls.length, 0);
  assert.match(offline.fields.get('ros-pose-preview-status').textContent, /previewOffline/);
  assert.notEqual(offline.drawings.at(-1)[3], true);
  const f = fixture(); f.preview(); const work = f.runTimer();
  f.pending.shift()({error_code: {val: -31}}); await work;
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewNoIK/);
  assert.equal(f.drawings.at(-1)[2], null);
  assert.equal(f.drawings.at(-1)[3], true);
});

test('XYZ edits and programmatic fills use two decimals without automatic preview', () => {
  const f = fixture();
  for (const [axis, value, expected] of [['x', '.23456', '0.23'], ['y', '-.1', '-0.10'], ['z', '0', '0.00']]) {
    const field = f.fields.get(`ros-pose-${axis}`);
    field.value = value;
    field.events.change();
    assert.equal(field.value, expected);
  }
  assert.equal(f.drawings.length, 0);
  f.fields.get('ros-pose-x').value = '.23456';
  assert.equal(f.window.reBotPosePreview.getTarget().pose.position.x, .23);
  assert.equal(f.fields.get('ros-pose-x').value, '0.23');
  f.fields.get('ros-pose-x').value = '';
  f.fields.get('ros-pose-x').events.change();
  assert.equal(f.fields.get('ros-pose-x').value, '');
  const els = {poseX: {}, poseY: {}, poseZ: {}};
  const context = vm.createContext({els});
  vm.runInContext(functionSource(source('ros/rebot-ros-ui.js'), 'writePoseInputs'), context);
  context.writePoseInputs({position: {x: .23456, y: -.1, z: 0}});
  assert.deepEqual(Object.values(els).map(field => field.value), ['0.23', '-0.10', '0.00']);
});

test('late failed IK cannot recolor a newer successful preview', async () => {
  const f = fixture();
  f.preview(); const old = f.runTimer();
  f.preview(); const next = f.runTimer();
  f.pending[1](f.result(.4)); await next;
  const count = f.drawings.length;
  f.pending[0]({error_code: {val: -31}}); await old;
  assert.equal(f.drawings.length, count);
  assert.notEqual(f.drawings.at(-1)[3], true);
});

test('clearing preview cancels queued IK and repeated clicks ignore older replies', async () => {
  const f = fixture();
  f.preview();
  assert.equal(f.timers.size, 1);
  f.fields.get('ros-vlim').events.input();
  assert.equal(f.timers.size, 0);
  f.preview(); const old = f.runTimer();
  f.preview(); const next = f.runTimer();
  f.pending[1](f.result(.4)); await next;
  const count = f.drawings.length;
  f.pending[0](f.result(.2)); await old;
  assert.equal(f.drawings.length, count);
  assert.equal(f.drawings.at(-1)[2].joint1, .4);
});

function functionSource(text, name) {
  const start = text.indexOf(`function ${name}(`);
  assert.ok(start >= 0, name);
  const brace = text.indexOf('{', start);
  let depth = 0;
  for (let i = brace; i < text.length; i += 1) {
    if (text[i] === '{') depth += 1;
    if (text[i] === '}' && --depth === 0) return text.slice(start, i + 1);
  }
  throw new Error(`Unclosed ${name}`);
}

test('rendered preview animates only ghost joints and converts ROS coordinates', () => {
  const names = ['joint1', 'joint2', 'joint3'];
  const current = {joint1: 0, joint2: 0, joint3: 0};
  const model = new THREE.Group(), ghost = new THREE.Group();
  const marker = new THREE.Mesh(new THREE.SphereGeometry(.01), new THREE.MeshBasicMaterial());
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial());
  let clock = 0;
  const context = vm.createContext({THREE, robot: model, ghostRobot: ghost, currentAngles: current,
    posePreview: null, targetGhost: marker, posePreviewLine: line,
    jointDefs: names.map(name => ({name})), performance: {now: () => clock},
    document: {getElementById: () => ({checked: true})},
    clamp: (v, lo, hi) => Math.max(lo, Math.min(v, hi)),
    setGhostJoint: (name, value) => {ghost.userData[name] = value;},
    getTcpPosition: root => new THREE.Vector3(root.userData.joint1 || 0, 0, 0),
    syncGhostToRobot: () => {names.forEach(name => {ghost.userData[name] = current[name];});}});
  const text = source('rebot-sim.js');
  for (const name of ['previewPose', 'updatePosePreview', 'clearPosePreview']) vm.runInContext(functionSource(text, name), context);
  context.previewPose({x: .3, y: -.1, z: .25}, 2, {joint1: .2, joint2: .4, joint3: .6});
  assert.deepEqual(marker.position.toArray(), [.3, .25, .1]);
  clock = 1000; context.updatePosePreview(clock);
  assert.equal(ghost.userData.joint1, .1);
  clock = 2000; context.updatePosePreview(clock);
  assert.equal(ghost.userData.joint1, .2);
  assert.deepEqual(current, {joint1: 0, joint2: 0, joint3: 0});
  context.previewPose({x: .6, y: .6, z: .7}, 2, null, true);
  clock = 3000; context.updatePosePreview(clock);
  assert.equal(marker.visible, true);
  assert.equal(marker.material.color.getHex(), 0xff6b5f);
  assert.equal(ghost.visible, false);
  assert.deepEqual(marker.position.toArray(), [.6, .7, -.6]);
  context.previewPose({x: .3, y: -.1, z: .25}, 2);
  context.updatePosePreview(clock);
  assert.equal(marker.visible, true);
  assert.equal(marker.material.color.getHex(), 0xf2a541);
  context.clearPosePreview();
  assert.equal(marker.visible, false); assert.equal(line.visible, false);
});

test('IK execution keeps control guards and avoids optimistic movement of feedback model', () => {
  const text = source('ros/rebot-ros-ui.js');
  const begin = text.indexOf('async function checkIk()');
  const end = text.indexOf('async function queryGravityCompensation', begin);
  const body = text.slice(begin, end);
  assert.ok(body.indexOf('controlAllowed(true)') < body.indexOf('moveToPoseViaIkTrajectory('));
  assert.doesNotMatch(body.slice(body.indexOf('controlAllowed(true)')), /moveToTcp\(/);
});

test('preview requests use only MoveIt IK and preserve scene attachments', async () => {
  const window = {};
  vm.runInNewContext(source('ros/rebot-ros-client.js'), {window, EventTarget, CustomEvent});
  const calls = [];
  const fake = {callService: (...args) => calls.push(args)};
  const pose = {position: {x: .3, y: 0, z: .3}, orientation: {x: 0, y: 0, z: 0, w: 1}};
  window.ReBotRosClient.prototype.previewPoseIK.call(fake, pose, ['joint1'], [0]);
  assert.equal(calls[0][0], '/compute_ik');
  assert.equal(calls[0][1], 'moveit_msgs/srv/GetPositionIK');
  assert.equal(calls[0][2].ik_request.avoid_collisions, true);
  assert.equal(calls[0][2].ik_request.robot_state.is_diff, true);
});
