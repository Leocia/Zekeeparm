import { t, onLangChange } from './i18n.js';

export function bindObstacleControls({ motion, view, isLocked, beforeMove, onSelectDemo, onDemoReady }) {
  const fields = document.getElementById('obstacle-fields');
  const enabled = document.getElementById('obstacle-enabled');
  const x = document.getElementById('obstacle-x');
  const y = document.getElementById('obstacle-y');
  const sizes = ['length', 'width', 'height'].map(name => document.getElementById(`obstacle-${name}`));
  const status = document.getElementById('obstacle-status');
  let message = { key: 'obstacle.off' };
  function renderMessage() {
    const vars = { ...message.vars };
    if (vars.body) vars.body = t(`obstacle.body.${vars.body}`);
    status.textContent = t(message.key, vars);
  }
  function say(key, vars) { message = { key, vars }; renderMessage(); }
  function syncFields() {
    const position = motion.position();
    x.value = (position[0] * 1000).toFixed(1);
    y.value = (position[1] * 1000).toFixed(1);
    motion.size().forEach((v, i) => { sizes[i].value = (v * 1000).toFixed(1); });
    enabled.checked = motion.isEnabled();
  }
  function refresh() {
    fields.disabled = isLocked();
    const [x, y, z] = motion.position();
    view.setObstacleInteraction({ enabled: motion.isEnabled() && !isLocked(), pose: { x, y, z },
      onStart: beforeMove, onMove: point => apply(true, point.x, point.y) });
  }
  function apply(next, nextX, nextY, dimensions) {
    if (isLocked()) { say('obstacle.locked'); return false; }
    let ok = false;
    try {
      motion.configure(next, nextX, nextY, dimensions);
      say(next ? 'obstacle.position' : 'obstacle.off', {
        x: (nextX * 1000).toFixed(1), y: (nextY * 1000).toFixed(1) });
      ok = true;
    } catch (error) {
      if (error.code === 'overlap') say(error.distance < 0 ? 'obstacle.intersects' : 'obstacle.overlap', {
        body: error.body,
        distance: (error.distance * 1000).toFixed(1), clearance: (error.clearance * 1000).toFixed(0) });
      else say(error.code === 'bounds' ? 'obstacle.bounds' : error.code === 'size' ? 'obstacle.sizeBounds' : 'obstacle.rejected');
    }
    syncFields(); refresh();
    return ok;
  }
  document.getElementById('obstacle-form').addEventListener('submit', event => {
    event.preventDefault();
    if (!isLocked()) beforeMove?.();
    apply(enabled.checked, Number.parseFloat(x.value) / 1000, Number.parseFloat(y.value) / 1000,
      sizes.map(input => Number.parseFloat(input.value) / 1000));
  });
  document.getElementById('obstacle-demo').addEventListener('click', () => {
    if (isLocked()) return;
    beforeMove?.(); onSelectDemo?.();
    if (apply(true, .30, -.30)) onDemoReady?.();
  });
  onLangChange(renderMessage);
  syncFields(); refresh();
  return { refresh,
    reset() {
      syncFields(); refresh();
      const [x, y] = motion.position();
      say(motion.isEnabled() ? 'obstacle.position' : 'obstacle.off', {
        x: (x * 1000).toFixed(1), y: (y * 1000).toFixed(1) });
    },
    update(state) {
      refresh();
      if (motion.isEnabled() && (state.running || state.stage === 'failed' || state.stage === 'complete')) {
        say(state.stage === 'failed' ? `obstacle.error.${state.message}` : 'obstacle.active', { count: state.detours });
      }
    }
  };
}
