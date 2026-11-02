import { Physics, namedId, ARM_NAMES } from './physics.js';

const CLEARANCE = .012;
const maxDelta = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));

// Bounded Cartesian detours retain the grasp orientation; all joint edges are sampled.
// shortcut: one static box and a finite set of detours, use a general planner for dense scenes.
export function createTaskMotion(mj, model, data, physics, ik) {
  const engine = new Physics(mj, model, data);
  engine.tcp = namedId(mj, model, 'SITE', 'pinch_center');
  const obstacleBody = namedId(mj, model, 'BODY', 'task_obstacle');
  const obstacle = namedId(mj, model, 'GEOM', 'task_obstacle_geom');
  const mocap = model.body_mocapid[obstacleBody];
  const flange = namedId(mj, model, 'BODY', 'link6');
  const robotGeoms = Array.from({ length: model.ngeom }, (_, i) => i).filter(i =>
    model.geom_group[i] === 3 || mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM.value, i) === 'wrist_camera_housing');
  const robotBodies = new Set(robotGeoms.map(i => model.geom_bodyid[i]));
  const distanceBuffer = new mj.DoubleBuffer(6);
  const quatBuffer = new mj.DoubleBuffer(4);
  let enabled = false;
  let path = null;
  let key = null;
  let deadline = 0;
  let carried = null;
  let detours = 0;
  let blocker = '';
  let blockerInfo = null;
  let diagnosticListener = null;
  const emit = (type, details = {}) => diagnosticListener?.({ type, ...details });
  const tcpAt = d => Array.from(d.site_xpos.slice(engine.tcp * 3, engine.tcp * 3 + 3));
  let appliedPosition = position();

  function position() { return Array.from(data.mocap_pos.slice(mocap * 3, mocap * 3 + 3)); }
  function size() { return Array.from(model.geom_size.slice(obstacle * 3, obstacle * 3 + 3), v => v * 2); }
  function setSize(dimensions) {
    const half = dimensions.map(v => v / 2);
    model.geom_size.set(half, obstacle * 3);
    // This single-box mocap body has no BVH; update the compiled broad-phase bounds too.
    model.geom_rbound[obstacle] = Math.hypot(...half);
    model.geom_aabb.set([0, 0, 0, ...half], obstacle * 6);
  }
  function configure(next, x, y, dimensions = size()) {
    if (typeof next !== 'boolean' || !Number.isFinite(x) || !Number.isFinite(y) ||
        x < .15 || x > .58 || Math.abs(y) > .38) {
      throw Object.assign(new Error('障碍物位置超出工作台范围'), { code: 'bounds' });
    }
    if (!Array.isArray(dimensions) || dimensions.length !== 3 ||
        dimensions.some(v => !Number.isFinite(v) || v < .01 || v > .4)) {
      throw Object.assign(new Error('障碍物长、宽、高需为 10–400 mm'), { code: 'size' });
    }
    const old = position();
    const oldSize = size();
    setSize(dimensions);
    data.mocap_pos.set([x, y, dimensions[2] / 2], mocap * 3);
    mj.mj_forward(model, data);
    // Reject placements intersecting the robot or a task object, rather than injecting a collision.
    const objects = ['red_cube_geom', 'blue_block_geom', 'yellow_cylinder_geom'].map(name => namedId(mj, model, 'GEOM', name));
    let closest = null;
    if (next) for (const geom of [...robotGeoms, ...objects]) {
      const distance = mj.mj_geomDistance(model, data, geom, obstacle, CLEARANCE, distanceBuffer);
      if (distance < (closest?.distance ?? CLEARANCE)) closest = { geom, distance };
    }
    if (closest) {
      setSize(oldSize);
      data.mocap_pos.set(old, mocap * 3);
      mj.mj_forward(model, data);
      throw Object.assign(new Error('障碍物与机械臂或物体过近，请调整位置'), {
        code: 'overlap', distance: closest.distance, clearance: CLEARANCE,
        body: mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, model.geom_bodyid[closest.geom])
      });
    }
    enabled = next;
    model.geom_contype[obstacle] = next ? 2 : 0;
    model.geom_conaffinity[obstacle] = next ? 3 : 0;
    model.body_contype[obstacleBody] = next ? 2 : 0;
    model.body_conaffinity[obstacleBody] = next ? 3 : 0;
    model.geom_rgba[obstacle * 4 + 3] = next ? 1 : 0;
    mj.mj_forward(model, data);
    appliedPosition = position();
    reset();
    emit('configured', { enabled, position: position(), size: size() });
  }

  function carry(bodyId = null) {
    carried = null;
    if (bodyId === null) return;
    const r = data.xmat.slice(flange * 9, flange * 9 + 9);
    const offset = [0, 1, 2].map(i => data.xpos[bodyId * 3 + i] - data.xpos[flange * 3 + i]);
    const rotation = data.xmat.slice(bodyId * 9, bodyId * 9 + 9);
    const local = [0, 1, 2].map(c => offset.reduce((s, v, row) => s + r[row * 3 + c] * v, 0));
    const localRotation = Array.from({ length: 9 }, (_, i) => [0, 1, 2].reduce((s, k) =>
      s + r[k * 3 + Math.floor(i / 3)] * rotation[k * 3 + i % 3], 0));
    carried = { bodyId, local, localRotation, address: model.jnt_qposadr[model.body_jntadr[bodyId]],
      geom: model.body_geomadr[bodyId] };
  }

  function valid(angles) {
    blockerInfo = null;
    if (angles.length !== 6 || angles.some((v, i) => !Number.isFinite(v) || v < engine.limits[i][0] || v > engine.limits[i][1])) {
      blockerInfo = { reason: 'joint-limits' }; return false;
    }
    const d = engine.scratch;
    d.qpos.set(data.qpos);
    d.mocap_pos.set(data.mocap_pos);
    d.mocap_quat.set(data.mocap_quat);
    engine.q.forEach((q, i) => { d.qpos[q] = angles[i]; });
    mj.mj_kinematics(model, d);
    if (carried) {
      const { local, localRotation, address } = carried;
      const r = d.xmat.slice(flange * 9, flange * 9 + 9);
      [0, 1, 2].forEach(row => {
        d.qpos[address + row] = d.xpos[flange * 3 + row] + local.reduce((s, v, c) => s + r[row * 3 + c] * v, 0);
      });
      const rotation = Array.from({ length: 9 }, (_, i) => [0, 1, 2].reduce((s, k) =>
        s + r[Math.floor(i / 3) * 3 + k] * localRotation[k * 3 + i % 3], 0));
      mj.mju_mat2Quat(quatBuffer, rotation);
      d.qpos.set(quatBuffer.GetView(), address + 3);
      mj.mj_kinematics(model, d);
    }
    if (enabled) {
      const blocked = [...robotGeoms, ...(carried ? [carried.geom] : [])].find(i =>
        mj.mj_geomDistance(model, d, i, obstacle, CLEARANCE, distanceBuffer) < CLEARANCE);
      if (blocked !== undefined) {
        blocker = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM.value, blocked);
        const distance = mj.mj_geomDistance(model, d, blocked, obstacle, CLEARANCE, distanceBuffer);
        const points = Array.from(distanceBuffer.GetView());
        blockerInfo = { reason: 'obstacle-clearance', blocker, distance,
          collisionPoint: [0, 1, 2].map(i => (points[i] + points[i + 3]) / 2) };
        return false;
      }
    }
    mj.mj_collision(model, d);
    const contacts = d.contact;
    let safe = true;
    try {
      for (let i = 0; i < contacts.size(); i++) {
        const contact = contacts.get(i);
        try {
          const a = model.geom_bodyid[contact.geom1], b = model.geom_bodyid[contact.geom2];
          const penetrationLimit = robotBodies.has(a) && robotBodies.has(b) ? 0 : -.001;
          if (contact.dist < penetrationLimit && (robotBodies.has(a) || robotBodies.has(b)) &&
              a !== carried?.bodyId && b !== carried?.bodyId) {
            safe = false;
            blocker = [a, b].map(id => mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, id)).join('/');
            blockerInfo = { reason: 'collision', blocker, distance: contact.dist, collisionPoint: Array.from(contact.pos) };
          }
        } finally { contact.delete(); }
      }
    } finally { contacts.delete(); }
    return safe;
  }

  function edge(a, b) {
    const steps = Math.max(1, Math.ceil(maxDelta(a, b) / .012));
    for (let i = 0; i <= steps; i++) {
      if (!valid(a.map((v, j) => v + (b[j] - v) * i / steps))) return false;
    }
    return true;
  }

  function reset() { path = null; key = null; detours = 0; carried = null; blocker = ''; }
  function install(next, nextKey) {
    if (nextKey === 'joints') {
      const start = engine.angles, goal = next[0];
      const steps = Math.max(1, Math.ceil(maxDelta(start, goal) / .025));
      next = Array.from({ length: steps }, (_, i) => start.map((v, j) => v + (goal[j] - v) * (i + 1) / steps));
    }
    path = next; key = nextKey;
    deadline = data.time + next.reduce((sum, q, i) => sum + maxDelta(i ? next[i - 1] : engine.angles, q) / .18, 0) + 10;
  }
  function tick() {
    if (!path?.length) return true;
    if (data.time > deadline) throw new Error('avoidance-timeout');
    const actual = engine.angles;
    if (!valid(actual)) throw new Error('obstacle-clearance');
    const next = path[0];
    if (maxDelta(actual, next) < (path.length === 1 ? .004 : .018)) {
      physics.setTargets(Object.fromEntries(ARM_NAMES.map((name, i) => [name, next[i]])));
      path.shift();
      if (!path.length) return true;
    }
    const goal = path[0];
    if (!edge(actual, goal)) throw new Error('obstacle-clearance');
    physics.setTargets(Object.fromEntries(ARM_NAMES.map((name, i) => [name, goal[i]])));
    return false;
  }
  function joints(pose) {
    if (!enabled) { physics.setTargets(pose); return; }
    const goal = ARM_NAMES.map((name, i) => pose[name] ?? engine.angles[i]);
    if (!edge(engine.angles, goal)) throw new Error('no-safe-path');
    install([goal], 'joints');
    if (Number.isFinite(pose.joint7)) physics.setTarget('joint7', pose.joint7);
  }

  function cartesian(target, stage) {
    if (key !== stage) {
      const start = ik.tcpPosition();
      const point = [target.x, target.y, target.z];
      const direct = [[start.x, start.y, start.z], point];
      const strategies = ['direct', 'over', 'side-negative', 'side-positive', 'outside'];
      emit('target', { stage, point, spacing: .006, jointSpacing: .012 });
      const [x, y, z] = position();
      const [length, width, boxHeight] = size();
      const height = Math.max(start.z, target.z, z + boxHeight / 2 + .12);
      const routes = [[], [[start.x, start.y, height], [point[0], point[1], height]],
        ...[-1, 1].map(sign => [[start.x, y + sign * (width / 2 + .145), height], [point[0], y + sign * (width / 2 + .145), height]]),
        [[x + length / 2 + .135, start.y, height], [x + length / 2 + .135, point[1], height]]];
      let result = null;
      const started = performance.now();
      let directFailure = null;
      for (let candidate = 0; candidate < routes.length && performance.now() - started < 1800; candidate++) {
        let seed = engine.angles, previous = [start.x, start.y, start.z];
        const trajectory = [];
        const sampledTcp = [[...previous]];
        let failure = null, sample = 0;
        emit('candidate', { stage, strategy: strategies[candidate], index: candidate });
        let safe = true;
        for (const waypoint of [...routes[candidate], point]) {
          const steps = Math.max(1, Math.ceil(Math.hypot(...waypoint.map((v, i) => v - previous[i])) / .006));
          for (let i = 1; i <= steps; i++) {
            sample++;
            try {
              const q = engine.solve(previous.map((v, j) => v + (waypoint[j] - v) * i / steps), ik.getOrientation(), seed);
              if (!edge(seed, q)) { failure = { ...blockerInfo }; safe = false; break; }
              sampledTcp.push(tcpAt(engine.scratch));
              trajectory.push(q); seed = q;
            } catch (error) { failure = { reason: 'ik', detail: error.message }; safe = false; break; }
          }
          if (!safe) break;
          previous = waypoint;
        }
        if (safe) {
          result = trajectory; detours += candidate > 0 ? 1 : 0;
          emit('planned', { stage, strategy: strategies[candidate], direct: candidate ? direct : [],
            collisionPoint: directFailure?.collisionPoint || null, directFailure,
            safe: sampledTcp, waypoints: routes[candidate], samples: trajectory.length,
            elapsed: performance.now() - started });
          break;
        }
        if (candidate === 0) directFailure = failure;
        emit('rejected', { stage, strategy: strategies[candidate], sample, ...failure });
      }
      if (!result) {
        emit('failed', { stage, direct, collisionPoint: directFailure?.collisionPoint || null,
          directFailure, elapsed: performance.now() - started });
        throw new Error('no-safe-path');
      }
      install(result, stage);
    }
    if (!path?.length && data.time > deadline) throw new Error('avoidance-timeout');
    const reached = tick();
    return reached && Math.hypot(ik.tcpPosition().x - target.x, ik.tcpPosition().y - target.y,
      ik.tcpPosition().z - target.z) < .008 && ik.orientationError() < .08;
  }
  return { configure, position, size, joints, tick, cartesian, carry, reset, valid, edge,
    onDiagnostics(listener) { diagnosticListener = listener; },
    proximity() {
      if (!enabled) return null;
      let nearest = null;
      for (const geom of robotGeoms) {
        const distance = mj.mj_geomDistance(model, data, geom, obstacle, 2, distanceBuffer);
        if (!Number.isFinite(distance) || distance >= (nearest?.distance ?? 2)) continue;
        const points = Array.from(distanceBuffer.GetView());
        nearest = { distance, clearance: CLEARANCE, from: points.slice(0, 3), to: points.slice(3),
          body: mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, model.geom_bodyid[geom]) };
      }
      return nearest;
    },
    restore() { data.mocap_pos.set(appliedPosition, mocap * 3); mj.mj_forward(model, data); reset(); },
    blocker: () => blocker,
    isEnabled: () => enabled, detours: () => detours,
    dispose() { engine.dispose(); distanceBuffer.delete(); quatBuffer.delete(); } };
}
