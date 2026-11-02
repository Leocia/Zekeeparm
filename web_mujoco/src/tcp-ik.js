import { ARM_JOINTS } from './kinematics.js';
import { Physics, namedId } from './physics.js';

export const NOMINAL_REACH = .68;

export function createTcpIk(mujoco, model, data, joints) {
  const engine = new Physics(mujoco, model, data);
  const bodyId = namedId(mujoco, model, 'BODY', 'gripper_end');
  let orientation = null;
  const tcpPosition = () => {
    const [x, y, z] = engine.position;
    return { x, y, z };
  };
  function servoStep(target, dt, angles) {
    if (!target || dt <= 0) return null;
    const point = [target.x, target.y, target.z];
    const seed = ARM_JOINTS.map(j => angles[j.name]);
    engine.scratch.qpos.set(data.qpos);
    engine.q.forEach((address, i) => { engine.scratch.qpos[address] = seed[i]; });
    mujoco.mj_forward(model, engine.scratch);
    const plannedPosition = Array.from(engine.scratch.site_xpos.slice(engine.tcp * 3, engine.tcp * 3 + 3));
    const plannedRotation = Array.from(engine.scratch.site_xmat.slice(engine.tcp * 9, engine.tcp * 9 + 9));
    const aligned = !orientation || Math.hypot(...plannedRotation.map((v, i) => v - orientation[i])) < .06;
    const distance = Math.hypot(...plannedPosition.map((v, i) => v - point[i]));
    const fraction = aligned ? Math.min(1, .18 * dt / Math.max(distance, .00001)) : 1;
    const waypoint = plannedPosition.map((v, i) => v + (point[i] - v) * fraction);
    let solution;
    try { solution = engine.solve(waypoint, orientation, seed); }
    catch { return { error: Math.hypot(...engine.position.map((v, i) => v - point[i])), reached: false, angles: { ...angles } }; }
    // One scale preserves relative joint motion; per-axis clipping twists a held object.
    const scale = Math.min(1, 1.8 * dt / Math.max(.00001, ...solution.map((v, i) => Math.abs(v - seed[i]))));
    const next = {};
    ARM_JOINTS.forEach((joint, i) => {
      const delta = (solution[i] - seed[i]) * scale;
      next[joint.name] = Math.max(joint.min, Math.min(joint.max, seed[i] + delta));
    });
    const error = Math.hypot(...engine.position.map((v, i) => v - point[i]));
    return { error, reached: error < .0015, angles: next };
  }
  function clampToWorkspace(point) {
    const base = Array.from(data.xpos.slice(engine.base * 3, engine.base * 3 + 3));
    const offset = [point.x - base[0], point.y - base[1], point.z - base[2]];
    const distance = Math.hypot(...offset);
    const ratio = Math.min(1, NOMINAL_REACH / Math.max(distance, .0001));
    const next = { x: base[0] + offset[0] * ratio, y: base[1] + offset[1] * ratio,
      z: Math.max(base[2] + .02, base[2] + offset[2] * ratio) };
    return { point: next, clamped: ratio < 1 || next.z !== point.z };
  }
  const tcpSite = engine.tcp;
  const pinchSite = namedId(mujoco, model, 'SITE', 'pinch_center');
  return { bodyId,
    setGraspMode(next) { engine.tcp = next ? pinchSite : tcpSite; orientation = null; }, tcpPosition, servoStep, clampToWorkspace,
    orientationError() { return orientation ? Math.hypot(...engine.rotation.map((v, i) => v - orientation[i])) : 0; },
    setOrientation(value) { orientation = value; }, getOrientation() { return orientation; },
    armNames: ARM_JOINTS.map(j => j.name) };
}
