import { ARM_JOINTS, homePose } from './kinematics.js';
import { Physics } from './physics.js';
import robot from './robot-config.json' with { type: 'json' };

export const STEPS_PER_FRAME = 4;

// Reuse the bundled model's position actuators, gains and effort limits.
export function createPhysicsController(mujoco, model, data, joints, { beforeStep, afterStep } = {}) {
  const engine = new Physics(mujoco, model, data);
  const targets = homePose(joints);
  const commanded = { ...targets };
  function setTarget(name, value) {
    const joint = joints.byName[name];
    if (!joint || !Number.isFinite(value)) return;
    const next = Math.min(joint.max, Math.max(joint.min, value));
    targets[name] = next;
    if (['joint7', 'joint_left', 'joint_right'].includes(name)) {
      targets.joint7 = targets.joint_left = targets.joint_right = next;
    }
  }
  function setTargets(pose) { Object.entries(pose).forEach(([name, value]) => setTarget(name, value)); }
  function step(steps = STEPS_PER_FRAME) {
    for (let i = 0; i < steps; i++) {
      for (const joint of [...ARM_JOINTS, joints.byName.joint7]) {
        const rate = Math.min(joint.unit === 'm' ? .05 : 1.8,
          robot.velocityLimits[joint.name === 'joint7' ? 'gripper_joint' : joint.name]);
        const step = rate * model.opt.timestep;
        commanded[joint.name] += Math.max(-step, Math.min(step, targets[joint.name] - commanded[joint.name]));
      }
      engine.setTargets(ARM_JOINTS.map(j => commanded[j.name]));
      engine.setGripper(commanded.joint7 * 2);
      // Bias compensation through the position actuator; its force limit still applies.
      engine.act.forEach((actuator, index) => {
        const gain = model.actuator_gainprm[actuator * 10];
        const desired = commanded[ARM_JOINTS[index].name] + data.qfrc_bias[engine.v[index]] / gain;
        data.ctrl[actuator] = Math.max(engine.limits[index][0], Math.min(engine.limits[index][1], desired));
      });
      beforeStep?.();
      engine.step();
      afterStep?.();
    }
  }
  function reset() {
    mujoco.mj_resetData(model, data);
    Object.assign(targets, homePose(joints));
    Object.assign(commanded, targets);
    joints.all.forEach(j => { data.qpos[j.qposadr] = targets[j.name]; });
    mujoco.mj_forward(model, data);
  }
  function telemetry() {
    const samples = ARM_JOINTS.map((joint, i) => ({
      name: joint.name, target: targets[joint.name], position: data.qpos[engine.q[i]],
      error: targets[joint.name] - data.qpos[engine.q[i]],
      torque: data.actuator_force[engine.act[i]],
      torqueLimit: model.actuator_forcerange[engine.act[i] * 2 + 1],
    }));
    const finger = engine.fingers[0];
    samples.push({ name: 'joint7', target: targets.joint7, position: data.qpos[finger.q],
      error: targets.joint7 - data.qpos[finger.q], torque: data.actuator_force[finger.act],
      torqueLimit: model.actuator_forcerange[finger.act * 2 + 1] });
    const rmsError = Math.sqrt(samples.slice(0, 6).reduce((sum, s) => sum + s.error ** 2, 0) / 6);
    return { joints: samples, rmsError };
  }
  reset();
  return { targets, setTarget, setTargets, step, reset, telemetry };
}
