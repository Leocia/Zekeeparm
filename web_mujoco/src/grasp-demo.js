import { ARM_JOINTS, GRIPPER_JOINTS } from './kinematics.js';
import robot from './robot-config.json' with { type: 'json' };

export const VISION_TARGETS = [
  { id: 'red', body: 'red_cube', labelKey: 'vision.red', color: '#ef5260' },
  { id: 'blue', body: 'blue_block', labelKey: 'vision.blue', color: '#45aef2' },
  { id: 'yellow', body: 'yellow_cylinder', labelKey: 'vision.yellow', color: '#e6d957' }
];

export const STORAGE_ZONES = robot.storageZones;

export const STACK_TARGETS = {
  blue: { x: 0.23, y: 0.20, z: 0.019 },
  red: { x: 0.23, y: 0.20, z: 0.0605 },
  yellow: { x: 0.23, y: 0.194, z: 0.109 }
};

const OPEN_WIDTH = GRIPPER_JOINTS[0].max;
const MOVE_TIMEOUT = 4.5;
// Cover IK/PD endpoint error so support is verified by actual table contact.
const PLACE_PRESS_DEPTH = 0.003;
const SUPPORT_SETTLE_TIME = 0.10;
const TOWER_SETTLE_TIME = 0.35;
const DROP_XY_TOLERANCE = 0.006;
const MAX_PLACEMENT_SPEED = 0.035;
const STACK_RETREAT_CLEARANCE = 0.12;
// Leave the stack away from the base to avoid folding link6 into link4.
const STACK_DEPART_OFFSET = { x: 0.05, y: -0.08 };
const READY_POSE = { ...robot.homePose };
const PREGRASP_PREPARE_TIMEOUT = 2.4;
const PREGRASP_ALIGN_TIMEOUT = 2.2;
const PREGRASP_JOINT_TOLERANCE = 0.055;
const STACK_CENTER_TIMEOUT = 1.4;
const STAGE_PROGRESS = {
  idle: 0,
  preparing: 0.025,
  aligning: 0.055,
  centering: 0.04,
  opening: 0.08,
  approach: 0.22,
  descend: 0.38,
  closing: 0.50,
  lift: 0.64,
  transfer: 0.76,
  place: 0.86,
  release: 0.94,
  retreat: 0.97,
  depart: 0.985,
  settling: 0.99,
  returning: 0.994,
  complete: 1,
  failed: 1
};

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function createGraspDemo({
  mujoco,
  model,
  data,
  joints,
  physics,
  ik,
  motion = null,
  workspaceOffset = { x: 0, y: 0 },
  onChange
}) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const tableBodyId = mujoco.mj_name2id(model, bodyType, 'task_table');
  const fingerBodies = new Set([
    mujoco.mj_name2id(model, bodyType, 'gripper_left'),
    mujoco.mj_name2id(model, bodyType, 'gripper_right')
  ]);
  const targets = VISION_TARGETS.map((target) => {
    const bodyId = mujoco.mj_name2id(model, bodyType, target.body);
    const jointAddress = model.body_jntadr[bodyId];
    return {
      ...target,
      bodyId,
      geomId: model.body_geomadr[bodyId],
      dofAddress: model.jnt_dofadr[jointAddress]
    };
  });
  if (targets.some((target) => target.bodyId < 0)) {
    throw new Error('抓取演示缺少颜色目标 body');
  }

  let selectedId = targets[0].id;
  let running = false;
  let stage = 'idle';
  let stageStartedAt = 0;
  let lastUpdateAt = 0;
  let ikAngles = {};
  let objectStart = null;
  let moveTarget = null;
  let dropTarget = null;
  let mode = 'put-away';
  let stackQueue = [];
  let pendingStackId = null;
  let expectedSupportBodyId = tableBodyId;
  let carryOffset = null;
  let supportContactSince = null;
  let pendingFailureReason = '';
  let pregraspPose = null;
  let pregraspAlignedPose = null;
  let alignmentJoint1 = null;
  let lastError = 0;
  let message = '';

  function selected() {
    return targets.find((target) => target.id === selectedId) || targets[0];
  }

  function bodyPosition(bodyId) {
    return {
      x: data.xpos[bodyId * 3],
      y: data.xpos[bodyId * 3 + 1],
      z: data.xpos[bodyId * 3 + 2]
    };
  }

  function workspacePoint(point) {
    return {
      ...point,
      x: point.x + (workspaceOffset.x || 0),
      y: point.y + (workspaceOffset.y || 0),
      z: point.z + (workspaceOffset.z || 0)
    };
  }

  function selectedLinearSpeed() {
    const start = selected().dofAddress;
    return Math.hypot(data.qvel[start], data.qvel[start + 1], data.qvel[start + 2]);
  }

  function snapshot() {
    return {
      running,
      stage,
      progress: STAGE_PROGRESS[stage] ?? 0,
      selectedId,
      error: lastError,
      message,
      mode,
      objectPosition: bodyPosition(selected().bodyId),
      objectStart,
      carryOffset: carryOffset ? { ...carryOffset } : null,
      alignmentJoint1,
      detours: motion?.detours() || 0
    };
  }

  function notify() {
    onChange?.(snapshot());
  }

  function enter(next, target = null) {
    stage = next;
    stageStartedAt = data.time;
    moveTarget = target;
    lastError = target ? distance(ik.tcpPosition(), target) : 0;
    notify();
  }

  function setSelected(next) {
    if (running || !targets.some((target) => target.id === next)) return false;
    selectedId = next;
    notify();
    return true;
  }

  function currentArmPose() {
    return Object.fromEntries(
      ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
    );
  }

  function closestJointAngle(desired, current, joint) {
    const candidates = [-1, 0, 1]
      .map((turn) => desired + turn * Math.PI * 2)
      .filter((angle) => angle >= joint.min && angle <= joint.max);
    if (candidates.length) {
      return candidates.reduce((best, angle) =>
        Math.abs(angle - current) < Math.abs(best - current) ? angle : best
      );
    }
    return Math.min(joint.max, Math.max(joint.min, desired));
  }

  function targetJoint1For(point, current) {
    const relativeX = point.x - (workspaceOffset.x || 0);
    const relativeY = point.y - (workspaceOffset.y || 0);
    const desired = -Math.atan2(relativeY, relativeX);
    return closestJointAngle(desired, current, joints.byName.joint1);
  }

  function armPoseReached(pose, tolerance = PREGRASP_JOINT_TOLERANCE) {
    return ARM_JOINTS.every((joint) => {
      const actual = data.qpos[joints.byName[joint.name].qposadr];
      return Math.abs((pose[joint.name] ?? actual) - actual) < tolerance;
    });
  }

  function begin(nextId, nextDropTarget, nextMode = 'put-away') {
    motion?.reset();
    ik.setGraspMode(true);
    selectedId = nextId;
    pendingStackId = null;
    mode = nextMode;
    objectStart = bodyPosition(selected().bodyId);
    const objectRotation = data.xmat.slice(selected().bodyId * 9, selected().bodyId * 9 + 9);
    const yaw = nextId === 'yellow' ? 0 : Math.atan2(objectRotation[3], objectRotation[0]);
    const c = Math.cos(yaw), s = Math.sin(yaw), base = robot.originalGraspRotation;
    ik.setOrientation([
      ...[0, 1, 2].map(i => c * base[i] - s * base[3 + i]),
      ...[0, 1, 2].map(i => s * base[i] + c * base[3 + i]),
      ...base.slice(6)
    ]);
    dropTarget = nextDropTarget;
    expectedSupportBodyId =
      nextMode === 'stack' && nextId === 'red'
        ? targets.find((target) => target.id === 'blue').bodyId
        : nextMode === 'stack' && nextId === 'yellow'
          ? targets.find((target) => target.id === 'red').bodyId
          : tableBodyId;
    carryOffset = null;
    supportContactSince = null;
    pendingFailureReason = '';
    ikAngles = currentArmPose();
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    pregraspPose = { ...READY_POSE, joint1: ikAngles.joint1 };
    alignmentJoint1 = targetJoint1For(objectStart, ikAngles.joint1);
    pregraspAlignedPose = { ...pregraspPose, joint1: alignmentJoint1 };
    message = '';
    lastUpdateAt = data.time;
    physics.setTarget('joint7', OPEN_WIDTH);
    commandPose(pregraspPose);
    enter('preparing');
  }

  function start(next = selectedId) {
    if (running) return false;
    if (!targets.some(target => target.id === next)) return false;
    setSelected(next);
    running = true;
    stackQueue = [];
    try { begin(next, workspacePoint(STORAGE_ZONES[next])); }
    catch (error) { stopBlocked(error.message); }
    return true;
  }

  function startStack() {
    if (running) return false;
    running = true;
    stackQueue = ['red', 'yellow'];
    try { begin('blue', workspacePoint(STACK_TARGETS.blue), 'stack'); }
    catch (error) { stopBlocked(error.message); }
    return true;
  }

  function cancel(reason = 'cancelled') {
    if (!running && stage === 'idle') return false;
    running = false;
    ik.setGraspMode(false);
    physics.setTargets(currentArmPose());
    motion?.reset();
    stage = 'idle';
    message = reason;
    lastError = 0;
    moveTarget = null;
    stackQueue = [];
    pendingStackId = null;
    mode = 'put-away';
    carryOffset = null;
    supportContactSince = null;
    pendingFailureReason = '';
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    notify();
    return true;
  }

  function fail(reason) {
    if (mode === 'stack') {
      returnToReady(reason);
      return;
    }
    running = false;
    ik.setGraspMode(false);
    message = reason;
    physics.setTarget('joint7', OPEN_WIDTH);
    enter('failed');
  }

  function stopBlocked(reason) {
    running = false;
    physics.setTargets(currentArmPose());
    motion?.reset();
    ik.setGraspMode(false);
    message = reason;
    enter('failed');
  }

  function commandPose(pose) {
    if (motion?.isEnabled()) motion.joints(pose);
    else physics.setTargets(pose);
  }

  function moveStep(dt) {
    if (motion?.isEnabled()) {
      const reached = motion.cartesian(moveTarget, stage);
      lastError = distance(ik.tcpPosition(), moveTarget);
      return reached;
    }
    const result = ik.servoStep(moveTarget, Math.max(0.001, Math.min(0.035, dt)), ikAngles);
    if (!result) return false;
    ikAngles = result.angles;
    physics.setTargets(ikAngles);
    lastError = distance(ik.tcpPosition(), moveTarget);
    return lastError < 0.006 && ik.orientationError() < .06;
  }

  function timedOut() {
    if (motion?.isEnabled() && !['returning', 'place'].includes(stage)) return false;
    if (motion?.isEnabled()) return data.time - stageStartedAt > 35;
    return data.time - stageStartedAt > MOVE_TIMEOUT;
  }

  function contactBodies() {
    const result = [];
    const contacts = data.contact;
    try {
      for (let index = 0; index < Math.min(data.ncon, contacts.size()); index++) {
        const contact = contacts.get(index);
        try { result.push([model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]]); }
        finally { contact.delete(); }
      }
    } finally { contacts.delete(); }
    return result;
  }

  function objectHasSupportContact() {
    const object = selected().bodyId;
    return contactBodies().some(([a, b]) =>
      (a === object && b === expectedSupportBodyId) || (b === object && a === expectedSupportBodyId));
  }

  function objectHasGripContact() {
    const object = selected().bodyId;
    const fingers = new Set();
    for (const [a, b] of contactBodies()) {
      if (a === object && fingerBodies.has(b)) fingers.add(b);
      if (b === object && fingerBodies.has(a)) fingers.add(a);
    }
    return fingers.size === 2;
  }

  function gripWidth() {
    const padIds = ['left_link_contact', 'right_link_contact'].map(name =>
      mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, name));
    const axis = [0, 1, 2].map(i => data.geom_xpos[padIds[0] * 3 + i] - data.geom_xpos[padIds[1] * 3 + i]);
    const length = Math.hypot(...axis);
    const { geomId } = selected();
    const size = model.geom_size.slice(geomId * 3, geomId * 3 + 3);
    const rotation = data.geom_xmat.slice(geomId * 9, geomId * 9 + 9);
    const projections = [0, 1, 2].map(col =>
      axis.reduce((sum, v, row) => sum + v / length * rotation[row * 3 + col], 0));
    const halfWidth = selectedId === 'yellow'
      ? size[0] * Math.sqrt(Math.max(0, 1 - projections[2] ** 2)) + size[1] * Math.abs(projections[2])
      : projections.reduce((sum, v, i) => sum + Math.abs(v) * size[i], 0);
    // Slight compression produces grip force without commanding the fingers through the object.
    return Math.max(0, Math.min(OPEN_WIDTH, halfWidth - robot.gripperClosedGap / 2 - .00075));
  }

  function objectIsAtDropTarget() {
    const object = bodyPosition(selected().bodyId);
    const horizontalError = Math.hypot(object.x - dropTarget.x, object.y - dropTarget.y);
    const verticalError = object.z - dropTarget.z;
    return horizontalError < DROP_XY_TOLERANCE && verticalError > -0.018 && verticalError < 0.030;
  }

  function rememberCarryOffset() {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    carryOffset = {
      x: object.x - tcp.x,
      y: object.y - tcp.y,
      z: object.z - tcp.z
    };
  }

  function gripAlignedDropTarget(z) {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    return {
      x: tcp.x + dropTarget.x - object.x,
      y: tcp.y + dropTarget.y - object.y,
      z
    };
  }

  function trackObjectOverDropTarget() {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    moveTarget.x = tcp.x + dropTarget.x - object.x;
    moveTarget.y = tcp.y + dropTarget.y - object.y;
  }

  function verticalRetreatTarget() {
    const tcp = ik.tcpPosition();
    return {
      x: tcp.x,
      y: tcp.y,
      z: dropTarget.z + (mode === 'stack' ? STACK_RETREAT_CLEARANCE : 0.11)
    };
  }

  function returnToReady(failureReason = '') {
    pendingFailureReason = failureReason;
    ik.setGraspMode(false);
    commandPose({ ...READY_POSE, joint7: OPEN_WIDTH });
    enter('returning');
  }

  function finishPlacement() {
    if (mode === 'stack' && stackQueue.length > 0) {
      pendingStackId = stackQueue.shift();
      commandPose({
        ...READY_POSE,
        joint7: OPEN_WIDTH,
        joint_left: OPEN_WIDTH,
        joint_right: OPEN_WIDTH
      });
      enter('centering');
    } else {
      returnToReady();
    }
  }

  function advance() {
    if (!running) return snapshot();
    const now = data.time;
    const dt = Math.max(0.001, now - lastUpdateAt);
    lastUpdateAt = now;
    const elapsed = now - stageStartedAt;
    if (motion?.isEnabled() && ['preparing', 'aligning', 'centering', 'returning'].includes(stage)) {
      if (!motion.tick()) { notify(); return snapshot(); }
    }

    if (stage === 'preparing') {
      if (armPoseReached(pregraspPose) || elapsed > PREGRASP_PREPARE_TIMEOUT) {
        if (mode === 'stack') {
          ikAngles = currentArmPose();
          enter('opening');
        } else {
          commandPose(pregraspAlignedPose);
          enter('aligning');
        }
      }
    } else if (stage === 'aligning') {
      if (armPoseReached(pregraspAlignedPose) || elapsed > PREGRASP_ALIGN_TIMEOUT) {
        ikAngles = currentArmPose();
        enter('opening');
      }
    } else if (stage === 'centering') {
      if (armPoseReached(READY_POSE) || elapsed > STACK_CENTER_TIMEOUT) {
        begin(pendingStackId, workspacePoint(STACK_TARGETS[pendingStackId]), 'stack');
      }
    } else if (stage === 'opening') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      if ((width > OPEN_WIDTH - 0.006 && elapsed > 0.18) || elapsed > 1.1) {
        enter('approach', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.10 });
      }
    } else if (stage === 'approach') {
      if (moveStep(dt)) enter('descend', { x: objectStart.x, y: objectStart.y, z: objectStart.z + .003 });
      else if (timedOut()) fail('approach-timeout');
    } else if (stage === 'descend') {
      const gripReady =
        elapsed > 0.15 &&
        distance(ik.tcpPosition(), moveTarget) < 0.018 &&
        ik.orientationError() < .06 &&
        objectHasGripContact();
      if (moveStep(dt) || gripReady) {
        physics.setTarget('joint7', gripWidth());
        enter('closing');
      } else if (timedOut()) fail('descend-timeout');
    } else if (stage === 'closing') {
      if (elapsed > 0.8) {
        motion?.carry(selected().bodyId);
        enter('lift', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.13 });
      }
    } else if (stage === 'lift') {
      if (moveStep(dt)) {
        const object = bodyPosition(selected().bodyId);
        if (object.z < objectStart.z + 0.035) fail('grasp-missed');
        else {
          rememberCarryOffset();
          enter('transfer', gripAlignedDropTarget(objectStart.z + 0.13));
        }
      } else if (timedOut()) fail('lift-timeout');
    } else if (stage === 'transfer') {
      if (moveStep(dt)) {
        const offset = carryOffset || { z: -0.006 };
        enter('place', gripAlignedDropTarget(dropTarget.z - offset.z - PLACE_PRESS_DEPTH));
      }
      else if (timedOut()) fail('transfer-timeout');
    } else if (stage === 'place') {
      trackObjectOverDropTarget();
      moveStep(dt);
      const supported =
        objectIsAtDropTarget() &&
        objectHasSupportContact() &&
        selectedLinearSpeed() < MAX_PLACEMENT_SPEED;
      if (supported && supportContactSince === null) supportContactSince = now;
      else if (!supported) supportContactSince = null;
      if (supportContactSince !== null && now - supportContactSince >= SUPPORT_SETTLE_TIME) {
        physics.setTarget('joint7', OPEN_WIDTH);
        motion?.carry(null);
        enter('release');
      } else if (timedOut()) fail('place-timeout');
    } else if (stage === 'release') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      const gripperIsClear = width > OPEN_WIDTH - 0.008;
      if ((elapsed > 0.35 && gripperIsClear) || elapsed > 1.1) {
        if (!motion?.isEnabled()) ik.setOrientation(null);
        enter('retreat', verticalRetreatTarget());
      }
    } else if (stage === 'retreat') {
      if (moveStep(dt)) {
        if (mode === 'stack') {
          enter('depart', {
            x: dropTarget.x + STACK_DEPART_OFFSET.x,
            y: dropTarget.y + STACK_DEPART_OFFSET.y,
            z: moveTarget.z
          });
        } else enter('settling');
      } else if (timedOut()) fail('retreat-timeout');
    } else if (stage === 'depart') {
      if (moveStep(dt)) enter('settling');
      else if (timedOut()) fail('depart-timeout');
    } else if (stage === 'settling') {
      if (
        (elapsed >= TOWER_SETTLE_TIME && selectedLinearSpeed() < MAX_PLACEMENT_SPEED * 0.5) ||
        elapsed > 1.2
      ) finishPlacement();
    } else if (stage === 'returning') {
      const ready = armPoseReached(READY_POSE, 0.02) &&
        Math.abs(data.qpos[joints.byName.joint7.qposadr] - OPEN_WIDTH) < 0.006;
      if (ready || timedOut()) {
        running = false;
        if (!ready) {
          physics.setTargets(currentArmPose());
          pendingFailureReason ||= 'return-timeout';
        }
        message = pendingFailureReason || (mode === 'stack' ? 'stack-complete' : 'complete');
        enter(pendingFailureReason ? 'failed' : 'complete');
      }
    }

    notify();
    return snapshot();
  }

  function update() {
    try { return advance(); }
    catch (error) {
      if (!motion?.isEnabled()) throw error;
      stopBlocked(error.message);
      return snapshot();
    }
  }

  function reset() {
    motion?.reset();
    running = false;
    ik.setGraspMode(false);
    stage = 'idle';
    stageStartedAt = data.time;
    lastUpdateAt = data.time;
    objectStart = null;
    moveTarget = null;
    dropTarget = null;
    pendingStackId = null;
    expectedSupportBodyId = tableBodyId;
    carryOffset = null;
    supportContactSince = null;
    pendingFailureReason = '';
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    lastError = 0;
    message = '';
    notify();
  }

  return {
    targets: targets.map(({ bodyId, ...target }) => ({ ...target })),
    setSelected,
    start,
    startStack,
    cancel,
    update,
    reset,
    state: snapshot,
    isRunning() {
      return running;
    }
  };
}
