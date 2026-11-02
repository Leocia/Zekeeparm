import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, configureRobot, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { createJointCallouts } from './ui.js';
import { createTcpIk } from './tcp-ik.js';
import { createTcpDrag } from './tcp-drag.js';
import { createPartExplorer } from './part-explorer.js';
import { createGraspDemo } from './grasp-demo.js';
import { createContactTelemetry } from './contact-telemetry.js';
import { createTelemetryPanel } from './telemetry-panel.js';
import { OBJECT_MOVE_STEP, createObjectPositionController } from './object-position-control.js';
import { t, bindLangSwitch, applyStaticI18n, onLangChange } from './i18n.js';
import { persistAppShell } from './register-service-worker.js';
import { createTaskMotion } from './task-motion.js';
import { bindModelEditor } from './model-editor.js';
import { bindObstacleControls } from './obstacle-controls.js';
import { bindTeachingControls } from './teaching-controls.js';
const statusEl = document.getElementById('status');
const calloutsEl = document.getElementById('callouts');
const resetEl = document.getElementById('reset');
const panelHintEl = document.getElementById('panel-hint');
const partExplorerEl = document.getElementById('part-explorer');
const toggleDragEl = document.getElementById('toggle-drag');
const toggleGuidesEl = document.getElementById('toggle-guides');
const toggleCameraViewsEl = document.getElementById('toggle-camera-views');
const cameraFeedGridEl = document.getElementById('camera-feed-grid');
const cameraLayoutRowEl = document.getElementById('camera-layout-row');
const cameraLayoutColumnEl = document.getElementById('camera-layout-column');
const toggleCameraModelEl = document.getElementById('toggle-camera-model');
const overheadCameraFeedEl = document.getElementById('overhead-camera-feed');
const wristCameraFeedEl = document.getElementById('wrist-camera-feed');
const collapseOverheadCameraEl = document.getElementById('collapse-overhead-camera');
const collapseWristCameraEl = document.getElementById('collapse-wrist-camera');
const graspControlsEl = document.getElementById('grasp-controls');
const graspHintEl = document.getElementById('grasp-hint');
const visionTargetsEl = document.getElementById('vision-targets');
const startGraspEl = document.getElementById('start-grasp');
const startStackEl = document.getElementById('start-stack');
const cancelGraspEl = document.getElementById('cancel-grasp');
const graspProgressEl = document.getElementById('grasp-progress');
const graspProgressValueEl = document.getElementById('grasp-progress-value');
const graspStageEl = document.getElementById('grasp-stage');
const objectEditHudEl = document.getElementById('object-edit-hud');
const objectEditSelectionEl = document.getElementById('object-edit-selection');
const telemetryControlsEl = document.getElementById('telemetry-controls');
const toggleContactVisualsEl = document.getElementById('toggle-contact-visuals');
const dragMarkerEl = document.getElementById('drag-marker');
const dragClusterEl = document.getElementById('drag-cluster');
const gripperOpenEl = document.getElementById('gripper-open');
const gripperCloseEl = document.getElementById('gripper-close');
const viewportEl = document.getElementById('viewport');
const pauseSimulationEl = document.getElementById('pause-simulation');
let simulationPaused = false;
let simulationFault = false;
const langSwitchEl = document.getElementById('lang-select');
const loadingOverlayEl = document.getElementById('loading-overlay');
const loadingTextEl = document.getElementById('loading-text');
const loadingProgressEl = document.getElementById('loading-progress');
const loadingProgressFillEl = document.getElementById('loading-progress-fill');
const loadingProgressValueEl = document.getElementById('loading-progress-value');

const LOAD_STAGE_PROGRESS = {
  'status.booting': 12,
  'status.loadingWasm': 49,
  'status.download': 69,
  'status.downloadProgress': 69,
  'status.loadingAssets': 69,
  'status.preparingRuntime': 89,
  'status.compiling': 94,
  'status.compiled': 99
};

bindLangSwitch(langSwitchEl);
applyStaticI18n();

function setStatus(text) {
  statusEl.textContent = text;
}

let loadProgress = { key: 'status.booting', vars: {} };
let loadingComplete = false;
let visualProgress = 0;
let visualProgressTarget = 0;
let visualProgressFrame = 0;
let visualProgressQueue = Promise.resolve();
let visualProgressCreepFrame = 0;
let visualProgressCreepActive = false;

function renderVisualProgress(value) {
  const rounded = Math.round(value);
  if (loadingProgressFillEl) loadingProgressFillEl.style.width = `${value}%`;
  if (loadingProgressValueEl) loadingProgressValueEl.textContent = `${rounded}%`;
  loadingProgressEl?.setAttribute('aria-valuenow', String(rounded));
}

function animateVisualProgress(next) {
  return new Promise((resolve) => {
    const start = visualProgress;
    const startedAt = performance.now();
    const duration = next >= 100 ? 120 : 240;

    const animate = (now) => {
      const ratio = Math.max(0, Math.min(1, (now - startedAt) / duration));
      const eased = 1 - (1 - ratio) ** 3;
      visualProgress = start + (next - start) * eased;
      renderVisualProgress(visualProgress);
      if (ratio < 1) {
        visualProgressFrame = requestAnimationFrame(animate);
      } else {
        visualProgress = next;
        renderVisualProgress(next);
        resolve();
      }
    };
    visualProgressFrame = requestAnimationFrame(animate);
  });
}

function setVisualProgress(next) {
  if (next <= visualProgressTarget) return visualProgressQueue;
  visualProgressTarget = next;
  visualProgressQueue = visualProgressQueue.then(() => animateVisualProgress(next));
  return visualProgressQueue;
}

function stopVisualProgressCreep() {
  visualProgressCreepActive = false;
  cancelAnimationFrame(visualProgressCreepFrame);
}

function startVisualProgressCreep() {
  if (visualProgressCreepActive) return;
  visualProgressCreepActive = true;

  void visualProgressQueue.then(() => {
    if (!visualProgressCreepActive || visualProgressTarget > 69) return;
    const start = visualProgress;
    const startedAt = performance.now();

    const creep = (now) => {
      if (!visualProgressCreepActive || visualProgressTarget > 69) return;
      const elapsedSeconds = Math.max(0, now - startedAt) / 1000;
      visualProgress = Math.min(88, Math.max(visualProgress, start + elapsedSeconds * 0.7));
      renderVisualProgress(visualProgress);
      visualProgressCreepFrame = requestAnimationFrame(creep);
    };

    visualProgressCreepFrame = requestAnimationFrame(creep);
  });
}

function renderLoadProgress() {
  const text = t(loadProgress.key, loadProgress.vars);
  setStatus(text);
  if (loadingTextEl) loadingTextEl.textContent = text;
}

function setLoadProgress(progress) {
  loadProgress = typeof progress === 'string' ? { key: progress, vars: {} } : progress;
  const nextProgress = LOAD_STAGE_PROGRESS[loadProgress.key];
  if (nextProgress === 69) {
    void setVisualProgress(nextProgress);
    startVisualProgressCreep();
  } else {
    stopVisualProgressCreep();
    if (nextProgress != null) void setVisualProgress(nextProgress);
  }
  renderLoadProgress();
}

function finishLoading() {
  loadingComplete = true;
  stopVisualProgressCreep();
  void setVisualProgress(100).then(() => {
    window.setTimeout(() => loadingOverlayEl?.classList.add('is-hidden'), 80);
  });
  void persistAppShell();
}

renderLoadProgress();
setVisualProgress(LOAD_STAGE_PROGRESS['status.booting']);

let panel = null;
let tcpDrag = null;
let partExplorer = null;
let graspDemo = null;
let objectPositionController = null;
let telemetryPanel = null;
let obstacleControls = null;
let readyCount = null;
let guidesVisible = false;
let overheadCameraVisible = false;
let wristCameraVisible = false;
let cameraViewsVisible = false;
let cameraViewsAvailable = false;
let cameraModelVisible = true;
let cameraModelAvailable = false;
let cameraLayout = 'row';
let overheadCameraCollapsed = false;
let wristCameraCollapsed = false;
let selectedVisionTarget = 'red';
let contactVisualsEnabled = true;
let graspState = { running: false, stage: 'idle', progress: 0, message: '' };
let objectControlState = {
  enabled: false,
  selectedId: 'red',
  position: { x: 0.24, y: -0.30, z: 0.1225 },
  quaternion: { w: 1, x: 0, y: 0, z: 0 },
  yaw: 0,
  lastResult: null
};

function renderGuidesToggle() {
  if (!toggleGuidesEl) return;
  toggleGuidesEl.textContent = t(guidesVisible ? 'btn.guidesOff' : 'btn.guidesOn');
  toggleGuidesEl.classList.toggle('active', guidesVisible);
  toggleGuidesEl.setAttribute('aria-pressed', String(guidesVisible));
}

function renderCameraViewsToggle() {
  if (toggleCameraViewsEl) {
    toggleCameraViewsEl.textContent = t(cameraViewsVisible ? 'camera.hideAll' : 'camera.showAll');
    toggleCameraViewsEl.classList.toggle('active', cameraViewsVisible);
    toggleCameraViewsEl.setAttribute('aria-checked', String(cameraViewsVisible));
    toggleCameraViewsEl.disabled = !cameraViewsAvailable;
  }
  if (cameraFeedGridEl) cameraFeedGridEl.hidden = !cameraViewsVisible;
  overheadCameraFeedEl?.classList.toggle('is-live', overheadCameraVisible);
  wristCameraFeedEl?.classList.toggle('is-live', wristCameraVisible);
}

function renderCameraModelToggle() {
  if (!toggleCameraModelEl) return;
  toggleCameraModelEl.textContent = t(cameraModelVisible ? 'camera.modelHide' : 'camera.modelShow');
  toggleCameraModelEl.setAttribute('aria-checked', String(cameraModelVisible));
  toggleCameraModelEl.classList.toggle('active', cameraModelVisible);
  toggleCameraModelEl.disabled = !cameraModelAvailable;
}

function renderCameraLayout() {
  const column = cameraLayout === 'column';
  cameraFeedGridEl?.classList.toggle('is-column', column);
  cameraLayoutRowEl?.classList.toggle('active', !column);
  cameraLayoutColumnEl?.classList.toggle('active', column);
  cameraLayoutRowEl?.setAttribute('aria-pressed', String(!column));
  cameraLayoutColumnEl?.setAttribute('aria-pressed', String(column));

  const renderCollapse = (feed, button, collapsed) => {
    feed?.classList.toggle('is-collapsed', collapsed);
    if (!button) return;
    button.textContent = collapsed ? '⌄' : '⌃';
    button.setAttribute('aria-expanded', String(!collapsed));
    button.setAttribute('aria-label', t(collapsed ? 'camera.expand' : 'camera.collapse'));
  };
  renderCollapse(overheadCameraFeedEl, collapseOverheadCameraEl, overheadCameraCollapsed);
  renderCollapse(wristCameraFeedEl, collapseWristCameraEl, wristCameraCollapsed);
}

function renderGraspControls() {
  visionTargetsEl?.querySelectorAll('[data-target]').forEach((button) => {
    const active = button.dataset.target === selectedVisionTarget;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
    button.disabled = graspState.running;
  });
  if (startGraspEl) startGraspEl.disabled = simulationFault || graspState.running;
  if (startStackEl) startStackEl.disabled = simulationFault || graspState.running;
  if (cancelGraspEl) cancelGraspEl.disabled = !graspState.running;
  if (graspProgressEl) graspProgressEl.value = graspState.progress || 0;
  if (graspProgressValueEl) graspProgressValueEl.textContent = `${Math.round((graspState.progress || 0) * 100)}%`;
  if (graspStageEl) graspStageEl.textContent = t(`grasp.stage.${graspState.stage}`);
  document.body.classList.toggle('grasp-running', graspState.running);
  if (toggleDragEl) toggleDragEl.disabled = graspState.running;
  if (gripperOpenEl) gripperOpenEl.disabled = graspState.running;
  if (gripperCloseEl) gripperCloseEl.disabled = graspState.running;
}

function renderObjectEditHud() {
  if (!objectEditHudEl) return;
  objectEditHudEl.hidden = false;
  objectEditHudEl.classList.toggle('is-locked', !objectControlState.enabled);
  if (objectEditSelectionEl) {
    objectEditSelectionEl.textContent = t('object.hudSelected', {
      name: t(`vision.${objectControlState.selectedId}`)
    });
  }
}

function renderContactVisualsToggle() {
  if (!toggleContactVisualsEl) return;
  toggleContactVisualsEl.textContent = t(contactVisualsEnabled ? 'telemetry.hideContacts' : 'telemetry.showContacts');
  toggleContactVisualsEl.setAttribute('aria-checked', String(contactVisualsEnabled));
  toggleContactVisualsEl.classList.toggle('active', contactVisualsEnabled);
}

onLangChange(() => {
  applyStaticI18n();
  panel?.applyLang();
  tcpDrag?.applyLang();
  partExplorer?.applyLang();
  renderGuidesToggle();
  renderCameraViewsToggle();
  renderCameraModelToggle();
  renderCameraLayout();
  renderGraspControls();
  renderObjectEditHud();
  renderContactVisualsToggle();
  if (!loadingComplete) {
    renderLoadProgress();
  } else if (!tcpDrag?.isEnabled() && readyCount != null) {
    setStatus(t('status.ready', { n: readyCount }));
  }
});

async function main() {
  setLoadProgress('status.loadingWasm');
  const mujocoPromise = loadMujocoModule();
  const { mujoco, model, data, config, source, browserError, files, materialProps } = await loadRsScene(
    mujocoPromise,
    setLoadProgress
  );
  configureRobot(config);
  // Renderer, PMREM and lighting initialization are deliberately deferred so
  // they do not block the WASM and model requests on the first load.
  const view = createSceneView(viewportEl);
  const joints = bindJoints(mujoco, model);
  const physics = createPhysicsController(mujoco, model, data, joints, {
    beforeStep: () => {
      objectPositionController?.enforcePose();
    }
  });
  const ik = createTcpIk(mujoco, model, data, joints);
  const motion = createTaskMotion(mujoco, model, data, physics, ik);
  const contactTelemetry = createContactTelemetry(mujoco, model, data);

  let syncObjectInteractionState = null;
  objectPositionController = createObjectPositionController({
    mujoco,
    model,
    data,
    onChange: (state) => {
      objectControlState = state;
      syncObjectInteractionState?.(state);
      renderObjectEditHud();
    }
  });
  objectControlState = objectPositionController.state();
  const objectByBodyName = new Map(
    objectPositionController.objects.map((object) => [object.body, object])
  );
  syncObjectInteractionState = (state = objectPositionController.state()) => {
    const selectedObject = objectPositionController.objects.find(
      (object) => object.id === state.selectedId
    );
    view.setObjectInteraction({
      enabled: state.enabled,
      selectedBodyName: selectedObject?.body,
      pose: { ...state.position, quaternion: state.quaternion },
      onSelect: (bodyName) => {
        const object = objectByBodyName.get(bodyName);
        if (!object || object.id === objectControlState.selectedId) return;
        selectedVisionTarget = object.id;
        objectPositionController.setSelected(object.id);
        graspDemo?.setSelected(object.id);
      },
      onMove: ({ x, y }) => {
        reportObjectMove(objectPositionController.moveTo(x, y));
      },
      onRotate: (quaternion) => {
        reportObjectRotation(objectPositionController.setQuaternion(quaternion));
      }
    });
  };
  syncObjectInteractionState(objectControlState);
  objectPositionController.setEnabled(true);

  setLoadProgress({ key: 'status.compiled', vars: { ngeom: model.ngeom } });
  view.build(mujoco, model, materialProps);
  view.sync(data);
  const teachingControls = bindTeachingControls({ motion, view });
  cameraViewsAvailable = view.hasOverheadCamera() || view.hasWristCamera();
  overheadCameraVisible = view.setOverheadCameraEnabled(true);
  wristCameraVisible = view.setWristCameraEnabled(true);
  cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
  cameraModelAvailable = view.hasCameraModel();
  cameraModelVisible = view.setCameraModelVisible(true);

  function setCameraViewsVisible(enabled) {
    overheadCameraVisible = view.setOverheadCameraEnabled(enabled);
    wristCameraVisible = view.setWristCameraEnabled(enabled);
    cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
    renderCameraViewsToggle();
    return cameraViewsVisible;
  }

  panel = createJointCallouts(calloutsEl, joints, (name, value) => {
    physics.setTarget(name, value);
  });
  panel.setGuidesVisible(guidesVisible);
  toggleGuidesEl?.addEventListener('click', () => {
    guidesVisible = !guidesVisible;
    panel.setGuidesVisible(guidesVisible);
    renderGuidesToggle();
  });
  renderGuidesToggle();
  toggleCameraViewsEl?.addEventListener('click', () => {
    const visible = setCameraViewsVisible(!cameraViewsVisible);
    setStatus(t(visible ? 'status.cameraViewsOn' : 'status.cameraViewsOff'));
  });
  renderCameraViewsToggle();
  toggleCameraModelEl?.addEventListener('click', () => {
    cameraModelVisible = view.setCameraModelVisible(!cameraModelVisible);
    renderCameraModelToggle();
    setStatus(t(cameraModelVisible ? 'status.cameraModelOn' : 'status.cameraModelOff'));
  });
  renderCameraModelToggle();
  cameraLayoutRowEl?.addEventListener('click', () => {
    cameraLayout = 'row';
    renderCameraLayout();
  });
  cameraLayoutColumnEl?.addEventListener('click', () => {
    cameraLayout = 'column';
    renderCameraLayout();
  });
  collapseOverheadCameraEl?.addEventListener('click', () => {
    overheadCameraCollapsed = !overheadCameraCollapsed;
    renderCameraLayout();
  });
  collapseWristCameraEl?.addEventListener('click', () => {
    wristCameraCollapsed = !wristCameraCollapsed;
    renderCameraLayout();
  });
  renderCameraLayout();
  let previousGraspStage = 'idle';
  graspDemo = createGraspDemo({
    mujoco,
    model,
    data,
    joints,
    physics,
    ik,
    motion,
    onChange: (state) => {
      graspState = state;
      teachingControls.updateTask(state);
      selectedVisionTarget = state.selectedId;
      if (objectControlState.selectedId !== state.selectedId) {
        objectPositionController.setSelected(state.selectedId);
      }
      renderGraspControls();
      renderObjectEditHud();
      obstacleControls?.update(state);
      if (state.stage !== previousGraspStage) {
        previousGraspStage = state.stage;
        if (state.stage === 'complete') {
          setStatus(t(state.mode === 'stack' ? 'status.stackComplete' : 'status.graspComplete'));
        }
        else if (state.stage === 'failed') setStatus(motion.isEnabled() ? t(`obstacle.error.${state.message}`) : t(
          state.message === 'return-timeout' ? 'status.returnFailed' : 'status.graspFailed'));
        else if (state.running) setStatus(t(`status.grasp.${state.stage}`));
      }
      const editingAvailable = !state.running;
      if (objectPositionController.isEnabled() !== editingAvailable) {
        objectPositionController.setEnabled(editingAvailable);
      }
    }
  });
  telemetryPanel = createTelemetryPanel(telemetryControlsEl);
  bindModelEditor(() => graspDemo.isRunning() || simulationFault,
    mujoco, source, browserError);
  obstacleControls = bindObstacleControls({ motion, view,
    isLocked: () => graspState.running || simulationFault,
    beforeMove: () => tcpDrag?.setEnabled(false),
    onSelectDemo: () => {
      selectedVisionTarget = 'red';
      graspDemo.setSelected('red');
      objectPositionController.setSelected('red');
    },
    onDemoReady: () => startGraspEl.click()
  });
  visionTargetsEl?.addEventListener('click', (event) => {
    const button = event.target.closest('[data-target]');
    if (!button || graspState.running) return;
    selectedVisionTarget = button.dataset.target;
    graspDemo.setSelected(selectedVisionTarget);
    objectPositionController.setSelected(selectedVisionTarget);
    renderGraspControls();
  });
  function reportObjectMove(result) {
    if (result.ok) {
      const position = result.position;
      setStatus(t('status.objectMoved', {
        name: t(`vision.${objectControlState.selectedId}`),
        x: Math.round(position.x * 1000),
        y: Math.round(position.y * 1000)
      }));
    } else if (result.reason === 'boundary') setStatus(t('status.objectBoundary'));
    else if (result.reason === 'overlap') setStatus(t('status.objectOverlap'));
  }

  function reportObjectRotation(result) {
    if (!result.ok) {
      reportObjectMove(result);
      return;
    }
    setStatus(t('status.objectRotated', {
      name: t(`vision.${objectControlState.selectedId}`)
    }));
  }

  const objectDirections = {
    forward: [OBJECT_MOVE_STEP, 0],
    backward: [-OBJECT_MOVE_STEP, 0],
    left: [0, OBJECT_MOVE_STEP],
    right: [0, -OBJECT_MOVE_STEP]
  };

  function moveObject(direction) {
    const delta = objectDirections[direction];
    if (!delta || !objectPositionController.isEnabled()) return;
    reportObjectMove(objectPositionController.move(delta[0], delta[1]));
  }

  window.addEventListener('keydown', (event) => {
    if (!objectPositionController.isEnabled() || graspState.running) return;
    if (event.target.closest?.('input, select, textarea, [contenteditable="true"]')) return;
    const direction = {
      KeyW: 'forward',
      KeyS: 'backward',
      KeyA: 'left',
      KeyD: 'right'
    }[event.code];
    if (!direction) return;
    event.preventDefault();
    moveObject(direction);
  });

  startGraspEl?.addEventListener('click', () => {
    if (graspState.running) return;
    objectPositionController.setEnabled(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.clearPartSelection();
    teachingControls.clearTrace();
    setSimulationPaused(false);
    graspDemo.start(selectedVisionTarget);
  });
  startStackEl?.addEventListener('click', () => {
    if (graspState.running) return;
    objectPositionController.setEnabled(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.clearPartSelection();
    teachingControls.clearTrace();
    setSimulationPaused(false);
    graspDemo.startStack();
  });
  cancelGraspEl?.addEventListener('click', () => {
    if (graspDemo.cancel()) setStatus(t('status.graspCancelled'));
  });
  toggleContactVisualsEl?.addEventListener('click', () => {
    contactVisualsEnabled = !contactVisualsEnabled;
    if (!contactVisualsEnabled) view.setContactVisuals([], false);
    renderContactVisualsToggle();
  });
  renderGraspControls();
  renderObjectEditHud();
  renderContactVisualsToggle();
  partExplorer = createPartExplorer(partExplorerEl, view);
  Object.entries(homePose(joints)).forEach(([name, amount]) => {
    panel.setTarget(name, amount);
    panel.setActual(name, amount);
  });

  tcpDrag = createTcpDrag({
    view,
    ik,
    physics,
    panel,
    clusterEl: dragClusterEl,
    markerEl: dragMarkerEl,
    hostEl: viewportEl,
    toggleEl: toggleDragEl,
    openEl: gripperOpenEl,
    closeEl: gripperCloseEl,
    onStatus: setStatus
  });

  view.render();
  panel.layout(view.projectWorld, data);

  function setSimulationPaused(next) {
    simulationPaused = next;
    if (next) { tcpDrag.stop(); graspDemo.cancel(); }
    pauseSimulationEl.dataset.i18n = next ? 'workspace.resume' : 'workspace.pause';
    pauseSimulationEl.textContent = t(pauseSimulationEl.dataset.i18n);
    pauseSimulationEl.setAttribute('aria-pressed', String(next));
    pauseSimulationEl.disabled = simulationFault;
    obstacleControls.refresh();
    setStatus(t(next ? 'status.paused' : 'status.ready'));
    renderGraspControls();
  }
  pauseSimulationEl.addEventListener('click', () => setSimulationPaused(!simulationPaused));
  resetEl.addEventListener('click', () => {
    simulationFault = false;
    setSimulationPaused(false);
    tcpDrag.stop();
    graspDemo.reset();
    telemetryPanel.reset();
    view.clearPartSelection();
    physics.reset();
    motion.restore();
    obstacleControls.reset();
    teachingControls.reset();
    objectPositionController.reset();
    objectPositionController.setEnabled(true);
    Object.entries(physics.targets).forEach(([name, amount]) => {
      panel.setTarget(name, amount);
      panel.setActual(name, amount);
    });
    setStatus(t('status.ready', { n: readyCount }));
    renderGraspControls();
  });

  setStatus(t('status.ready', { n: files.length }));
  readyCount = files.length;
  finishLoading();

  let lastTelemetryAt = -Infinity;
  let animationFrameId = 0;
  const loop = () => {
    animationFrameId = 0;
    if (document.hidden) return;
    const now = performance.now();
    if (!simulationPaused) {
      try {
        graspDemo.update();
        physics.step(graspDemo.isRunning() ? 12 : STEPS_PER_FRAME);
      } catch (error) {
        simulationFault = true;
        setSimulationPaused(true);
        setStatus(t('status.simulationStopped', { error: error.message }));
      }
    }
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([name, amount]) => panel.setActual(name, amount));
    if (graspDemo.isRunning()) {
      Object.entries(physics.targets).forEach(([name, amount]) => panel.setTarget(name, amount));
    }
    view.sync(data);
    tcpDrag.update(now);
    if (now - lastTelemetryAt >= 160) {
      objectControlState = objectPositionController.state();
      renderObjectEditHud();
      const contacts = contactTelemetry.sample(selectedVisionTarget, contactVisualsEnabled);
      if (contactVisualsEnabled) view.setContactVisuals(contacts.contacts, true);
      telemetryPanel.update(physics.telemetry(), contacts);
      teachingControls.updateProximity();
      lastTelemetryAt = now;
    }
    view.render();
    panel.layout(view.projectWorld, data);
    animationFrameId = requestAnimationFrame(loop);
  };
  const startLoop = () => {
    if (!document.hidden && !animationFrameId) animationFrameId = requestAnimationFrame(loop);
  };
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      cancelAnimationFrame(animationFrameId);
      animationFrameId = 0;
        return;
    }
    lastTelemetryAt = -Infinity;
    startLoop();
  });
  startLoop();
}

main().catch((error) => {
  console.error(error);
  const message = error && error.message ? error.message : error;
  loadingOverlayEl?.classList.add('is-error');
  setLoadProgress({ key: 'status.fail', vars: { error: message } });
});
