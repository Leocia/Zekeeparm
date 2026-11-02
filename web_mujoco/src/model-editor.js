import { t } from './i18n.js';
import { loadRsScene } from './load-model.js';
import { browserUrdfKey, readBrowserUrdf } from './browser-urdf.js';
import { diagnoseUrdf } from './urdf-diagnostics.js';

export function bindModelEditor(isBusy, mujoco, source, browserError) {
  const staticMode = import.meta.env.PROD;
  const form = document.getElementById('urdf-editor');
  const input = document.getElementById('urdf-text');
  const status = document.getElementById('urdf-status');
  const buttons = [...form.querySelectorAll('button')];
  const diagnostics = document.getElementById('urdf-diagnostics');
  const diagnosticSummary = document.getElementById('urdf-diagnostic-summary');
  let diagnosticTimer;
  function diagnose() {
    clearTimeout(diagnosticTimer);
    const errors = diagnoseUrdf(input.value, source?.urdf);
    input.setAttribute('aria-invalid', String(errors.length > 0));
    diagnosticSummary.textContent = errors.length ? `发现 ${errors.length} 项问题；点击跳转到对应行。` : 'XML 与六轴拓扑检查通过；同步时仍会执行转换与物理编译校验。';
    diagnostics.replaceChildren(...errors.map(error => {
      const li = document.createElement('li'), button = document.createElement('button');
      button.type = 'button'; button.textContent = `L${error.line} · ${error.message}`;
      button.addEventListener('click', () => {
        const start = input.value.split('\n').slice(0, error.line - 1).reduce((n, line) => n + line.length + 1, 0);
        input.focus(); input.setSelectionRange(start, input.value.indexOf('\n', start) < 0 ? input.value.length : input.value.indexOf('\n', start));
        input.scrollTop = Math.max(0, (error.line - 4) * parseFloat(getComputedStyle(input).lineHeight));
      });
      li.append(button); return li;
    }));
    return errors;
  }
  const draftKey = staticMode ? `${browserUrdfKey()}:draft` : 'zekeeparm.urdf.draft';
  let original = null;
  let saving = false;
  let dirty = false;
  const say = message => { status.textContent = message; };
  const lock = value => { buttons.forEach(button => { button.disabled = value; }); input.disabled = value; };
  function stash() {
    if (dirty && original) {
      try { sessionStorage.setItem(draftKey, JSON.stringify({ ...original, urdf: input.value })); }
      catch { say(t('model.draftUnavailable')); }
    }
  }
  async function load() {
    lock(true);
    try {
      if (staticMode) {
        let stored = null;
        try { stored = readBrowserUrdf(); } catch { /* Save will report unavailable storage. */ }
        original = { urdf: stored ?? source.urdf, stored };
        input.value = original.urdf; dirty = false;
        say(browserError ? t('model.browserFallback', { error: browserError }) : t(stored === null ? 'model.published' : 'model.browserCustom'));
        return;
      }
      const response = await fetch('./__model/urdf', { cache: 'no-store' });
      if (!response.ok) throw new Error(t('model.unavailable'));
      original = await response.json();
      input.value = original.urdf;
      dirty = false;
      say(t(original.custom ? 'model.custom' : 'model.bundled'));
    } catch (error) { say(error.message); }
    finally { lock(!original); if (original) diagnose(); }
  }
  input.addEventListener('input', () => {
    dirty = true; stash(); clearTimeout(diagnosticTimer); diagnosticTimer = setTimeout(diagnose, 300);
  });
  document.getElementById('urdf-read').addEventListener('click', async () => {
    await load();
    if (original) { try { sessionStorage.removeItem(draftKey); } catch { /* Draft storage is optional. */ } }
  });
  async function save(restore = false) {
    if (!original || saving || isBusy()) { say(t('model.idleOnly')); return; }
    if (!restore && diagnose().length) { say('URDF 未应用：请先修正上方诊断问题；当前仿真模型保持运行。'); return; }
    saving = true; lock(true); say(t('model.saving'));
    try {
      if (staticMode) {
        const ensureCurrent = () => {
          if (isBusy()) throw new Error(t('model.idleOnly'));
          let stored;
          try { stored = readBrowserUrdf(); } catch { throw new Error(t('model.storageUnavailable')); }
          if (stored !== original.stored) throw new Error(t('model.browserStale'));
        };
        ensureCurrent();
        const compiled = await loadRsScene(mujoco, null, { urdf: restore ? null : input.value });
        compiled.data.delete(); compiled.model.delete();
        ensureCurrent();
        try {
          if (restore) localStorage.removeItem(browserUrdfKey());
          else localStorage.setItem(browserUrdfKey(), input.value);
        } catch { throw new Error(t('model.storageUnavailable')); }
        dirty = false;
        try { sessionStorage.removeItem(draftKey); } catch { /* Draft storage is optional. */ }
        location.reload();
        return;
      }
      const response = await fetch('./__model/urdf', { method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Model-Token': original.token },
        body: JSON.stringify({ urdf: input.value, revision: original.revision,
          sourceRevision: original.sourceRevision, restore }) });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error);
      dirty = false;
      sessionStorage.removeItem(draftKey);
      location.reload();
    } catch (error) { say(error.message); lock(false); }
    finally { saving = false; }
  }
  form.addEventListener('submit', event => { event.preventDefault(); void save(); });
  document.getElementById('urdf-restore').addEventListener('click', () => void save(true));
  if (staticMode) {
    document.getElementById('model-hint').textContent = t('model.browserHint');
    document.getElementById('model-hint').dataset.i18n = 'model.browserHint';
    document.getElementById('urdf-restore').textContent = t('model.restorePublished');
    document.getElementById('urdf-restore').dataset.i18n = 'model.restorePublished';
  } else {
    import.meta.hot.on('model:updated', () => { if (!saving) { stash(); location.reload(); } });
    import.meta.hot.on('vite:beforeFullReload', stash);
    import.meta.hot.on('model:error', error => say(error.message));
  }
  void load().then(() => {
    let stored;
    try { stored = sessionStorage.getItem(draftKey); } catch { return; }
    if (!stored || !original) return;
    try {
      const draft = JSON.parse(stored);
      original = staticMode ? { ...original, stored: draft.stored } : { ...draft, token: original.token };
      input.value = draft.urdf;
      dirty = true; say(t('model.draft'));
      diagnose();
    } catch { sessionStorage.removeItem(draftKey); }
  });
}
