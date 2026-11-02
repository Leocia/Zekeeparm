import * as THREE from 'three';

export function createTeachingView(scene, layer, setPose) {
  const frames = new THREE.Group(), trajectories = new THREE.Group();
  const proximity = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xb36913, depthTest: false }));
  const axes = [];
  let showProximity = true;
  frames.visible = false; proximity.visible = false;
  scene.add(frames, trajectories, proximity);
  function onMainView(object) {
    object.traverse(child => { child.layers.set(layer); child.renderOrder = 6; });
    return object;
  }
  onMainView(proximity);
  function clearTrace() {
    while (trajectories.children.length) {
      const item = trajectories.children[0];
      trajectories.remove(item); item.geometry.dispose(); item.material.dispose();
    }
  }
  function line(points, color, dashed = false) {
    if (points.length < 2) return;
    const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
    const material = dashed ? new THREE.LineDashedMaterial({ color, dashSize: .009, gapSize: .005, depthTest: false })
      : new THREE.LineBasicMaterial({ color, depthTest: false });
    const object = new THREE.Line(geometry, material);
    if (dashed) object.computeLineDistances();
    trajectories.add(onMainView(object));
  }
  return {
    build(mj, model) {
      for (const axis of axes) { frames.remove(axis); axis.geometry.dispose(); axis.material.dispose(); }
      axes.length = 0;
      for (let body = 1; body < model.nbody; body++) {
        const name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, body);
        if (!/^(base_link|link[1-6]|gripper_(base|left|right|end))$/.test(name || '')) continue;
        const axis = onMainView(new THREE.AxesHelper(.045));
        axis.name = name; axis.userData.bodyId = body; axis.material.depthTest = false;
        frames.add(axis); axes.push(axis);
      }
    },
    sync(data) { if (frames.visible) axes.forEach(axis => setPose(axis, data.xpos, data.xmat, axis.userData.bodyId)); },
    setFrames(enabled) { frames.visible = enabled; return axes.length; },
    setTrajectoriesVisible(enabled) { trajectories.visible = enabled; },
    setProximityVisible(enabled) { showProximity = enabled; if (!enabled) proximity.visible = false; },
    setProximity(value) {
      proximity.visible = Boolean(showProximity && value && [...value.from, ...value.to].every(Number.isFinite));
      if (!proximity.visible) return;
      proximity.geometry.dispose();
      proximity.geometry = new THREE.BufferGeometry().setFromPoints([value.from, value.to].map(p => new THREE.Vector3(...p)));
      proximity.material.color.setHex(value.distance < value.clearance ? 0xb82f2f : value.distance < .04 ? 0xb36913 : 0x23794a);
    },
    setTrace(trace) {
      clearTrace();
      line(trace.direct || [], 0xc33333, true);
      line(trace.safe || [], 0x16864e);
      if (trace.waypoints?.length) {
        const points = new THREE.Points(new THREE.BufferGeometry().setFromPoints(trace.waypoints.map(p => new THREE.Vector3(...p))),
          new THREE.PointsMaterial({ color: 0x16864e, size: .013, depthTest: false }));
        trajectories.add(onMainView(points));
      }
      if (trace.collisionPoint?.every(Number.isFinite)) {
        const marker = new THREE.Mesh(new THREE.OctahedronGeometry(.009), new THREE.MeshBasicMaterial({ color: 0xc33333, depthTest: false }));
        marker.position.set(...trace.collisionPoint); trajectories.add(onMainView(marker));
      }
    },
    clearTrace,
    dispose() {
      clearTrace();
      for (const axis of axes) { axis.geometry.dispose(); axis.material.dispose(); }
      proximity.geometry.dispose(); proximity.material.dispose(); scene.remove(frames, trajectories, proximity);
    }
  };
}
