(function () {
  'use strict';
  const t = (key, values) => window.rebotI18n ? window.rebotI18n.t(key, values) : key;
  const fields = ['x', 'y', 'z', 'duration'].map(name => document.getElementById(`ros-pose-${name}`));
  const status = document.getElementById('ros-pose-preview-status');
  const client = window.reBotRos;
  if (!status || fields.some(field => !field)) return;
  let target = null, targetKey = '', generation = 0, timer = null;

  function updatePositionLimits() {
    const maximum = window.reBotSim?.getPosePositionMax();
    if (!maximum) return null;
    ['x', 'y', 'z'].forEach((axis, index) => {
      fields[index].min = String(axis === 'z' ? 0 : -maximum[axis]);
      fields[index].max = String(maximum[axis]);
    });
    const hint = document.getElementById('ros-pose-limits');
    if (hint) hint.textContent = t('action.positionLimits', {
      x: maximum.x.toFixed(2), y: maximum.y.toFixed(2), z: maximum.z.toFixed(2)});
    return maximum;
  }

  function getTarget() {
    const values = fields.map(field => field.value.trim() === '' ? NaN : Number(field.value));
    if (!values.every(Number.isFinite) || values[3] < .4 || values[3] > 8) return null;
    const maximum = updatePositionLimits();
    if (!maximum || fields.slice(0, 3).some((field, index) =>
      values[index] < Number(field.min) || values[index] > Number(field.max))) return null;
    fields.slice(0, 3).forEach((field, index) => {
      field.value = values[index].toFixed(2);
      values[index] = Number(field.value);
    });
    const current = window.reBotSim?.getTcpPose();
    if (!current) return null;
    const key = values.join(',');
    if (!target || key !== targetKey) {
      targetKey = key;
      target = {pose: {position: {x: values[0], y: values[1], z: values[2]},
        orientation: {...current.orientation}}, duration: values[3]};
    }
    return {pose: {position: {...target.pose.position}, orientation: {...target.pose.orientation}}, duration: target.duration};
  }

  function clear() {
    generation += 1;
    window.clearTimeout(timer);
    target = null;
    window.reBotSim?.clearPosePreview();
    status.textContent = '';
  }

  function refresh() {
    clear();
    const selected = getTarget();
    if (!selected) {
      const values = fields.map(field => field.value.trim() === '' ? NaN : Number(field.value));
      if (values.every(Number.isFinite) && values[3] >= .4 && values[3] <= 8) {
        window.reBotSim?.previewPose({x: values[0], y: values[1], z: values[2]}, values[3], null, true);
      }
      status.textContent = t('action.previewInvalid');
      return;
    }
    window.reBotSim.previewPose(selected.pose.position, selected.duration);
    if (!client?.connected) {
      status.textContent = t('action.previewOffline');
      return;
    }
    status.textContent = t('action.previewSolving');
    const epoch = generation;
    timer = window.setTimeout(async () => {
      try {
        const definitions = window.reBotSim.getJointDefs().filter(item => item.name !== 'gripper');
        const names = definitions.map(item => item.name);
        const current = window.reBotSim.getAngles();
        const start = names.map(name => current[name]);
        const result = await client.previewPoseIK(selected.pose, names, start);
        if (epoch !== generation || !client.connected) return;
        const state = result.solution?.joint_state;
        const solution = new Map((state?.name || []).map((name, index) => [name, state.position?.[index]]));
        const angles = names.map(name => solution.get(name));
        if (result.error_code?.val !== 1 || !angles.every((value, index) => Number.isFinite(value)
            && value >= definitions[index].min && value <= definitions[index].max)) {
          window.reBotSim.previewPose(selected.pose.position, selected.duration, null, true);
          status.textContent = t('action.previewNoIK', {code: result.error_code?.val ?? '?'});
          return;
        }
        const velocity = Math.max(.05, Math.min(1.5, Number(document.getElementById('ros-vlim')?.value) || .3));
        const seconds = window.ReBotControlPolicy.computeJointTrajectoryDuration(start, angles, velocity, selected.duration);
        window.reBotSim.previewPose(selected.pose.position, seconds, Object.fromEntries(names.map((name, index) => [name, angles[index]])));
        status.textContent = t('action.previewReady', {sec: seconds.toFixed(1)});
      } catch (error) {
        if (epoch === generation) status.textContent = t('action.previewFailed', {error: error.message});
      }
    }, 250);
  }

  document.getElementById('ros-pose-current')?.addEventListener('click', () => {
    clear();
    const position = window.reBotSim?.getTcpPose()?.position;
    if (!position || ![position.x, position.y, position.z].every(Number.isFinite)) {
      status.textContent = t('action.currentUnavailable');
      return;
    }
    [position.x, position.y, position.z].forEach((value, index) => {
      fields[index].value = value.toFixed(2);
    });
    status.textContent = t('action.currentFilled');
  });
  fields.forEach((field, index) => {
    field.addEventListener('input', clear);
    field.addEventListener('change', () => {
      if (index < 3 && field.value.trim() !== '' && Number.isFinite(Number(field.value))) {
        field.value = Number(field.value).toFixed(2);
      }
      clear();
    });
  });
  document.getElementById('ros-pose-preview')?.addEventListener('click', refresh);
  document.getElementById('ros-vlim')?.addEventListener('input', clear);
  window.addEventListener('rebot-model-ready', () => {clear(); updatePositionLimits();});
  client?.addEventListener('status', clear);
  window.rebotI18n?.onLangChange?.(updatePositionLimits);
  window.reBotPosePreview = {getTarget, refresh, clear};
  updatePositionLimits();
})();
