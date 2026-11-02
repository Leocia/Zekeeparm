// Browser and Node share this engine. It has no ROS or hardware command path.
export const ARM_NAMES = Array.from({ length: 6 }, (_, i) => `joint${i + 1}`);
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

export function namedId(mj, model, type, name) {
  const id = mj.mj_name2id(model, mj.mjtObj[`mjOBJ_${type}`].value, name);
  if (id < 0) throw new Error(`模型缺少 ${name}`);
  return id;
}

export class Physics {
  constructor(mj, model, data) {
    this.mj = mj;
    this.model = model;
    this.data = data;
    this.scratch = new mj.MjData(model);
    const id = (type, name) => namedId(mj, model, type, name);
    this.joints = ARM_NAMES.map(name => id('JOINT', name));
    this.q = this.joints.map(j => model.jnt_qposadr[j]);
    this.v = this.joints.map(j => model.jnt_dofadr[j]);
    this.act = ARM_NAMES.map(name => id('ACTUATOR', name));
    this.limits = this.joints.map(j => Array.from(model.jnt_range.slice(j * 2, j * 2 + 2)));
    this.fingers = ['gripper_joint', 'right_joint'].map(name => ({
      q: model.jnt_qposadr[id('JOINT', name)], act: id('ACTUATOR', name),
    }));
    this.tcp = id('SITE', 'tcp');
    this.base = id('BODY', 'base_link');
    this.jacp = new mj.DoubleBuffer(3 * model.nv);
    this.jacr = new mj.DoubleBuffer(3 * model.nv);
    this.solvers = [3, 6].map(n => ({ n, matrix: new mj.DoubleBuffer(n * n), result: new mj.DoubleBuffer(n) }));
  }

  get angles() { return this.q.map(q => this.data.qpos[q]); }
  get position() { return Array.from(this.data.site_xpos.slice(this.tcp * 3, this.tcp * 3 + 3)); }
  get rotation() { return Array.from(this.data.site_xmat.slice(this.tcp * 9, this.tcp * 9 + 9)); }

  setTargets(values) {
    if (values.length !== 6 || !values.every((v, i) => Number.isFinite(v) && v >= this.limits[i][0] && v <= this.limits[i][1])) {
      throw new Error('关节目标必须是限位内的六个有限数值');
    }
    this.act.forEach((a, i) => { this.data.ctrl[a] = values[i]; });
  }

  setGripper(width) {
    if (!Number.isFinite(width) || width < 0 || width > .07) throw new Error('夹爪总开口范围为 0–70 mm');
    this.fingers.forEach(f => { this.data.ctrl[f.act] = width / 2; });
  }

  step(count = 1) {
    for (let i = 0; i < count; i++) this.mj.mj_step(this.model, this.data);
    if (!this.data.qpos.every(Number.isFinite) ||
        this.v.some(v => Math.abs(this.data.qvel[v]) > 12) ||
        this.q.some((q, i) => this.data.qpos[q] < this.limits[i][0] - .08 || this.data.qpos[q] > this.limits[i][1] + .08)) {
      throw new Error('物理状态异常，请复位场景');
    }
  }

  solve(position, rotation = null, seed = this.angles) {
    const base = this.data.xpos.slice(this.base * 3, this.base * 3 + 3);
    if (position.length !== 3 || !position.every(Number.isFinite) || Math.hypot(...position.map((v, i) => v - base[i])) > 1.1) {
      throw new Error('TCP 目标不可达或数值无效');
    }
    const { mj, model, scratch: d } = this;
    d.qpos.set(this.data.qpos);
    this.q.forEach((q, i) => { d.qpos[q] = seed[i]; });
    const solver = this.solvers[rotation ? 1 : 0];
    const n = solver.n;
    for (let iteration = 0; iteration < 400; iteration++) {
      mj.mj_forward(model, d);
      const p = d.site_xpos.slice(this.tcp * 3, this.tcp * 3 + 3);
      const e = position.map((v, i) => v - p[i]);
      if (rotation) {
        const current = d.site_xmat.slice(this.tcp * 9, this.tcp * 9 + 9);
        const orientation = [0, 0, 0];
        for (let col = 0; col < 3; col++) {
          const a = [current[col], current[col + 3], current[col + 6]];
          const b = [rotation[col], rotation[col + 3], rotation[col + 6]];
          orientation[0] += .5 * (a[1] * b[2] - a[2] * b[1]);
          orientation[1] += .5 * (a[2] * b[0] - a[0] * b[2]);
          orientation[2] += .5 * (a[0] * b[1] - a[1] * b[0]);
        }
        // The cross-product error is zero at 180°, so also check the matrix distance.
        const aligned = Math.hypot(...current.map((v, i) => v - rotation[i])) < .03;
        if (Math.hypot(...e) < .0015 && aligned) return this.q.map(q => d.qpos[q]);
        e.push(...orientation);
      } else if (Math.hypot(...e) < .0015) return this.q.map(q => d.qpos[q]);
      mj.mj_jacSite(model, d, this.jacp, this.jacr, this.tcp);
      const jp = this.jacp.GetView(), jr = this.jacr.GetView();
      const J = Array.from({ length: n }, (_, row) => this.v.map(v =>
        row < 3 ? jp[row * model.nv + v] : jr[(row - 3) * model.nv + v]));
      const matrix = solver.matrix.GetView();
      for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) {
        matrix[r * n + c] = J[r].reduce((sum, v, i) => sum + v * J[c][i], 0) + (r === c ? .001 : 0);
      }
      mj.mju_cholFactor(solver.matrix, 1e-12);
      mj.mju_cholSolve(solver.result, Array.from(solver.matrix.GetView()), e);
      const delta = solver.result.GetView();
      this.q.forEach((q, i) => {
        const dq = J.reduce((sum, row, r) => sum + row[i] * delta[r], 0);
        d.qpos[q] = clamp(d.qpos[q] + clamp(dq, -.08, .08), ...this.limits[i]);
      });
    }
    throw new Error('TCP 目标不可达；请调整位置或取消保持姿态');
  }

  dispose() {
    [this.scratch, this.jacp, this.jacr,
      ...this.solvers.flatMap(s => [s.matrix, s.result])].forEach(resource => resource.delete());
  }
}
