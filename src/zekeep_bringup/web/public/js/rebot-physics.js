(function () {
  'use strict';
  const el = id => document.getElementById(id);
  const t = key => window.rebotI18n ? window.rebotI18n.t(key) : key;
  let connected = false, state = null, polling = false;
  async function request(path, body) {
    const response = await fetch(path, body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok || value.ok === false) throw new Error(value.error || `HTTP ${response.status}`);
    return value;
  }
  async function poll() {
    if (!connected || polling) return;
    polling = true;
    try {
      state = await request('/api/sim/state');
      if (state.source !== 'mujoco' || state.namespace !== '/zekeep_sim') throw new Error(t('p45.incompatible'));
      el('physics-status').textContent = `${state.phase} · contact=${state.contact} · ${state.result?.message || state.camera_error || ''}`;
      el('physics-pick').disabled = state.busy || state.mode !== 'physics' || !state.detections?.length;
      if (el('physics-mirror').checked) {
        if (el('ros-control-enable').checked || el('ros-mirror').checked || window.reBotAIActive || window.reBotTaskState?.owner) {
          el('physics-mirror').checked = false; window.reBotPhysicsActive = false;
          throw new Error(t('p45.mirrorGuard'));
        }
        window.reBotPhysicsActive = true;
        window.reBotSim?.setAngles(state.joints, {source: 'physics'});
        window.reBotSim?.syncMujocoObjectStates(state.objects.map(object => ({name: 'red_cube', position: object.position, quat_wxyz: object.quaternion})));
      } else window.reBotPhysicsActive = false;
      const camera = await request('/api/sim/camera');
      if (typeof camera.image === 'string' && camera.image.startsWith('data:image/png;base64,')) el('physics-camera').src = camera.image;
    } catch (error) {
      state = null; el('physics-pick').disabled = true; el('physics-status').textContent = error.message;
    } finally { polling = false; }
  }
  async function command(body) {
    if (!connected) return;
    if (body.command === 'reset') el('physics-live-mirror').checked = false;
    try { await request('/api/sim/command', body); await poll(); } catch (error) { el('physics-status').textContent = error.message; }
  }
  function init() {
    if (!el('physics-connect')) return;
    el('physics-connect').onclick = () => { connected = true; poll(); };
    el('physics-disconnect').onclick = () => { connected = false; state = null; window.reBotPhysicsActive = false; el('physics-mirror').checked = false; el('physics-live-mirror').checked = false; el('physics-pick').disabled = true; el('physics-status').textContent = t('p45.disconnected'); };
    el('physics-reset').onclick = () => command({command: 'reset'});
    el('physics-stop').onclick = () => command({command: 'stop'});
    el('physics-pick').onclick = () => { if (state?.detections?.length) command({command: 'pick_place', detection_id: state.detections[0].id}); };
    el('physics-joints').onclick = () => {
      try { const positions = JSON.parse(el('physics-positions').value); command({command: 'joints', positions}); }
      catch (error) { el('physics-status').textContent = error.message; }
    };
    el('physics-open').onclick = () => command({command: 'gripper', width_m: .07});
    el('physics-close').onclick = () => command({command: 'gripper', width_m: 0});
    let lastMirror = 0, mirrorPending = false;
    window.reBotRos?.subscribe('/zekeep/joint_states', 'sensor_msgs/msg/JointState', async message => {
      if (!connected || !el('physics-live-mirror').checked || mirrorPending || Date.now()-lastMirror < 200) return;
      const stamp = Number(message.header?.stamp?.sec) + Number(message.header?.stamp?.nanosec)/1e9;
      const age = Date.now()/1000-stamp;
      if (!Number.isFinite(age) || age < -.1 || age > .5) return;
      if (!Array.isArray(message.name) || !Array.isArray(message.position)) return;
      const joints = Object.fromEntries(message.name.map((name, index) => [name, message.position[index]]));
      const positions = Array.from({length: 6}, (_, index) => joints[`joint${index+1}`]);
      const finger = Number.isFinite(joints.gripper_joint) ? joints.gripper_joint : joints.right_joint;
      if (!positions.every(Number.isFinite) || !Number.isFinite(finger)) return;
      mirrorPending = true; lastMirror = Date.now();
      try { await request('/api/sim/command', {command: 'mirror', positions, width_m: finger*2}); }
      catch (error) { el('physics-status').textContent = error.message; }
      finally { mirrorPending = false; }
    }, {throttleRate: 200});
    setInterval(poll, 250);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();
})();
