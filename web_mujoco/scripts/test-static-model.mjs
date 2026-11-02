import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createServer, preview } from 'vite';

// Chrome supplies the real XML DOM, storage, WASM and WebGL; no browser test dependency.
const root = fileURLToPath(new URL('..', import.meta.url));
const profile = await mkdtemp(join(tmpdir(), 'zekeeparm-browser-'));
const chrome = spawn(process.env.CHROME_BIN || 'google-chrome', ['--headless=new', '--no-sandbox',
  '--disable-dev-shm-usage', '--enable-unsafe-swiftshader', '--use-angle=swiftshader',
  '--remote-debugging-port=0', `--user-data-dir=${profile}`, 'about:blank'], { stdio: ['ignore', 'ignore', 'pipe'] });
let server, dev, socket;
const errors = [], requests = [], pending = new Map();
let sequence = 0;
function send(method, params = {}) {
  return new Promise((resolve, reject) => {
    const id = ++sequence; pending.set(id, { resolve, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
}
async function evaluate(expression) {
  const result = await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
  if (result.exceptionDetails) throw new Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
  return result.result.value;
}
async function until(expression, label, timeout = 60000) {
  const deadline = Date.now() + timeout;
  while (true) {
    try { if (await evaluate(expression)) return; }
    catch (error) { if (!/navigated or closed|context.*destroyed|Cannot find context/i.test(error.message)) throw error; }
    assert.ok(Date.now() < deadline, `Timeout: ${label}; errors: ${errors.join('\n')}`);
    await new Promise(resolve => setTimeout(resolve, 200));
  }
}
try {
  const endpoint = await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('Chrome startup timeout')), 15000);
    let output = '';
    chrome.stderr.on('data', bytes => {
      output += bytes;
      const match = output.match(/DevTools listening on (ws:\/\/[^\s]+)/);
      if (match) { clearTimeout(timer); resolve(match[1]); }
    });
    chrome.on('error', error => { clearTimeout(timer); reject(error); });
    chrome.on('exit', code => { clearTimeout(timer); reject(new Error(`Chrome exited: ${code}`)); });
  });
  const debugging = new URL(endpoint);
  const pages = await (await fetch(`http://${debugging.host}/json/list`)).json();
  socket = new WebSocket(pages.find(page => page.type === 'page').webSocketDebuggerUrl);
  await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
  socket.onmessage = event => {
    const message = JSON.parse(event.data);
    if (message.id) {
      const promise = pending.get(message.id); pending.delete(message.id);
      if (message.error) promise.reject(new Error(JSON.stringify(message.error))); else promise.resolve(message.result);
    } else if (message.method === 'Runtime.exceptionThrown') errors.push(message.params.exceptionDetails.exception?.description || message.params.exceptionDetails.text);
    else if (message.method === 'Network.requestWillBeSent') requests.push(message.params.request);
  };
  await send('Runtime.enable'); await send('Network.enable'); await send('Page.enable');
  await send('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1000, deviceScaleFactor: 1, mobile: false });

  // Conversion assertions use the same browser code and genuine WASM, with all requests local.
  dev = await createServer({ root, configFile: false, base: '/', optimizeDeps: { noDiscovery: true, include: [] },
    plugins: [{ name: 'browser-test-page', configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (req.url !== '/__browser_test') return next();
        res.setHeader('Content-Type', 'text/html'); res.end('<!doctype html><title>URDF checks</title>');
      });
    } }], server: { host: '127.0.0.1', port: 0 }, logLevel: 'error' });
  await dev.listen();
  await send('Page.navigate', { url: `http://127.0.0.1:${dev.httpServer.address().port}/__browser_test` });
  await until('document.title === "URDF checks"', 'test page');
  const checks = await evaluate(`(async () => {
    const {loadMujocoModule, loadRsScene} = await import('/src/load-model.js');
    const {convertBrowserUrdf, loadBrowserSource} = await import('/src/browser-urdf.js');
    const {diagnoseUrdf} = await import('/src/urdf-diagnostics.js');
    const {configureRobot, bindJoints} = await import('/src/kinematics.js');
    const source = await loadBrowserSource();
    const good = diagnoseUrdf(source.urdf, source.urdf);
    if (good.length) throw new Error('Bundled URDF diagnostics: '+JSON.stringify(good));
    const limitText = source.urdf.replace('upper="2.58"', 'upper="-3"');
    const limitLine = limitText.slice(0, limitText.indexOf('upper="-3"')).split('\\n').length;
    const limits = diagnoseUrdf(limitText, source.urdf);
    if (!limits.some(error => error.line === limitLine && error.message.includes('lower < upper'))) throw new Error('Limit line diagnostics failed: '+JSON.stringify(limits));
    const invalidXml = '<robot>\\n<link name="a">\\n</robot>';
    const syntax = diagnoseUrdf(invalidXml, source.urdf);
    if (!syntax.some(error => error.line === 3 && error.message.includes('XML'))) throw new Error('XML line diagnostics failed: '+JSON.stringify(syntax));
    if (!diagnoseUrdf(source.urdf.replace('name="joint1"', 'name="missing"'), source.urdf).some(error => error.message.includes('joint1'))) throw new Error('Required six-axis check failed');
    if (!diagnoseUrdf(source.urdf.replace('xyz="0 0 -1"', 'xyz="0 0 0"'), source.urdf).some(error => error.message.includes('axis/xyz'))) throw new Error('Axis diagnostics failed');
    const xml = await (await fetch('/models/rs_grasp_scene.xml')).text();
    const mj = await loadMujocoModule();
    const base = await loadRsScene(mj, null, {urdf: null});
    const same = await loadRsScene(mj, null, {urdf: source.urdf});
    const near = (a,b) => a.length === b.length && Array.from(a).every((v,i) => Math.abs(v-b[i]) < 2e-6);
    const invariant = ['body_pos','body_quat','body_mass','body_inertia','jnt_range','actuator_gainprm','actuator_biasprm','site_pos'];
    const parity = Object.fromEntries(invariant.map(key => [key, near(base.model[key], same.model[key])]));
    base.data.delete(); base.model.delete(); same.data.delete(); same.model.delete();
    const doc = new DOMParser().parseFromString(source.urdf, 'application/xml');
    for (const material of [...doc.documentElement.children].filter(node => node.tagName === 'material')) doc.documentElement.appendChild(material);
    doc.querySelector('joint[name="joint3"] > origin').setAttribute('xyz', '.265 0 0');
    doc.querySelector('joint[name="joint2"] > limit').setAttribute('velocity', '.5');
    doc.querySelector('joint[name="joint1"] > limit').setAttribute('upper', '2.4');
    doc.querySelector('joint[name="joint1"] > limit').setAttribute('effort', '20');
    doc.querySelector('joint[name="joint1"] > axis').setAttribute('xyz', '0 1 -1');
    doc.querySelector('link[name="link6"] > collision mesh').setAttribute('filename', 'package://zekeep_bringup/description/collision/link6.STL');
    doc.querySelector('link[name="link2"] > inertial > origin').setAttribute('rpy', '0 0 1.5707963267948966');
    doc.querySelector('link[name="link1"] > inertial > mass').setAttribute('value', '.8');
    doc.querySelector('material[name="light_grey"] > color').setAttribute('rgba', '.2 .4 .6 1');
    const modified = new XMLSerializer().serializeToString(doc);
    const result = await loadRsScene(mj, null, {urdf: modified});
    const id = (type,name) => mj.mj_name2id(result.model,mj.mjtObj['mjOBJ_'+type].value,name);
    const model = result.model, body = id('BODY','link2');
    const rotation = new mj.DoubleBuffer(9); mj.mju_quat2Mat(rotation, Array.from(model.body_iquat.slice(body*4,body*4+4)));
    const r = rotation.GetView(), inertia = model.body_inertia.slice(body*3,body*3+3);
    const tensorXX = inertia.reduce((sum,v,i) => sum + v*r[i]*r[i],0); rotation.delete();
    configureRobot(result.config);
    const controls = bindJoints(mj,model);
    const actual = {position:model.body_pos[id('BODY','link3')*3], mass:model.body_mass[id('BODY','link1')],
      velocity:result.config.velocityLimits.joint2, max:controls.byName.joint1.max,
      effort:model.actuator_forcerange[id('ACTUATOR','joint1')*2+1], tensorXX,
      distinctMesh:model.geom_dataid[id('GEOM','link6_collision')] !== model.geom_dataid[id('GEOM','link6_visual')],
      axis:Array.from(model.jnt_axis.slice(id('JOINT','joint1')*3,id('JOINT','joint1')*3+3)),
      color:Array.from(model.geom_rgba.slice(id('GEOM','link1_visual')*4,id('GEOM','link1_visual')*4+4))};
    result.data.delete(); result.model.delete();
    let rejected = 0;
    for (const invalid of [source.urdf.replace('name="joint1"','name="missing"'),
      source.urdf.replace('lower="-2.58"','lower="nan"'), source.urdf.replace('upper="2.58"','upper="-2"'),
      source.urdf.replace('multiplier="1"','multiplier="2"'),
      source.urdf.replace('package://zekeep_bringup/description/meshes/base_link.STL','https://example.org/a.STL'),
      source.urdf.replace('<axis xyz="0 0 -1"', '<dynamics damping="2"/><axis xyz="0 0 -1"'),
      '<!DOCTYPE robot><robot/>']) {
      try {convertBrowserUrdf(invalid,xml,source);} catch {rejected++;}
    }
    return {parity,actual,rejected};
  })()`);
  assert.ok(Object.values(checks.parity).every(Boolean), JSON.stringify(checks.parity));
  assert.ok(Math.abs(checks.actual.position - .265) < 1e-8);
  assert.ok(Math.abs(checks.actual.mass - .8) < 1e-8);
  assert.equal(checks.actual.velocity, .5); assert.equal(checks.actual.max, 2.4); assert.equal(checks.actual.effort, 20);
  assert.ok(Math.abs(checks.actual.tensorXX - .005877765) < 1e-7);
  assert.equal(checks.actual.distinctMesh, true);
  assert.ok(Math.abs(checks.actual.axis[1] - Math.SQRT1_2) < 1e-8);
  assert.ok(Math.abs(checks.actual.color[0] - .2) < 1e-6);
  assert.equal(checks.rejected, 7);
  console.log('PASS: real-browser URDF conversion and numeric model parity');
  await dev.close(); dev = null;

  // Production preview has no editor API; exercise save, reload, error recovery and restore in a subdirectory.
  server = await preview({ root, base: '/static-test/', preview: { host: '127.0.0.1', port: 0, open: false }, logLevel: 'error' });
  const url = `http://127.0.0.1:${server.httpServer.address().port}/static-test/`;
  await send('Page.navigate', { url });
  const ready = 'document.querySelector("#loading-overlay")?.classList.contains("is-hidden") && !document.querySelector("#urdf-text")?.disabled';
  await until(ready, 'static app ready');
  console.log('PASS: production static app loads in a subdirectory');
  assert.equal(await evaluate(`document.querySelector('#urdf-diagnostics').children.length`), 0);
  const counts = await evaluate(`(() => {
    const frames=document.querySelector('#study-frames'), hulls=document.querySelector('#study-hulls');
    frames.click(); hulls.click(); return [+frames.dataset.count, +hulls.dataset.count];
  })()`);
  assert.ok(counts[0] >= 10 && counts[1] >= 10, 'teaching frame and hull geometry must exist');
  await evaluate(`document.querySelector('#study-title').scrollIntoView({block:'start'})`);
  const geometryScreenshot = await send('Page.captureScreenshot', { format: 'png' });
  await writeFile('/tmp/zekeeparm-teaching-geometry.png', Buffer.from(geometryScreenshot.data, 'base64'));
  requests.length = 0;
  const original = await evaluate('document.querySelector("#urdf-text").value');
  const edited = original.replace('upper="2.58"', 'upper="2.4"');
  assert.notEqual(edited, original);
  async function submit(text) {
    await evaluate(`(() => {const input = document.querySelector('#urdf-text'); input.value=${JSON.stringify(text)}; input.dispatchEvent(new Event('input',{bubbles:true})); document.querySelector('#urdf-editor').requestSubmit();})()`);
  }
  await submit(edited);
  await until(`localStorage.getItem('zekeeparm.urdf:/static-test/') === ${JSON.stringify(edited)}`, 'URDF saved');
  await until(`${ready} && document.querySelector('#urdf-text').value === ${JSON.stringify(edited)} && document.querySelector('#urdf-status').textContent.includes('副本')`, 'saved model reloaded');
  assert.equal(await evaluate(`document.querySelector('[data-joint="joint1"] input').max`), '2.4', 'saved limits must reach the live control panel');
  console.log('PASS: static URDF save and scene reload');
  await submit('<robot/>');
  await until(`!document.querySelector('#urdf-text').disabled && document.querySelector('#urdf-diagnostics').children.length > 0 && document.querySelector('#urdf-status').textContent.includes('诊断')`, 'invalid edit rejected');
  assert.equal(await evaluate(`localStorage.getItem('zekeeparm.urdf:/static-test/')`), edited);
  await evaluate(`localStorage.setItem('zekeeparm.urdf:/static-test/', ${JSON.stringify(original)})`);
  await submit(edited);
  await until(`document.querySelector('#urdf-status').textContent.includes('其他标签页')`, 'cross-tab conflict rejected');
  assert.equal(await evaluate(`localStorage.getItem('zekeeparm.urdf:/static-test/')`), original);
  await evaluate(`document.querySelector('#urdf-read').click()`);
  await until(`!document.querySelector('#urdf-text').disabled && document.querySelector('#urdf-text').value === ${JSON.stringify(original)}`, 'editor reread');
  await evaluate(`window.savedSetItem = Storage.prototype.setItem; Storage.prototype.setItem = function(){throw new DOMException('blocked','SecurityError')}`);
  await submit(edited);
  await until(`!document.querySelector('#urdf-text').disabled && document.querySelector('#urdf-status').textContent.includes('不允许保存')`, 'blocked storage handled');
  assert.equal(await evaluate(`localStorage.getItem('zekeeparm.urdf:/static-test/')`), original);
  await evaluate(`Storage.prototype.setItem = window.savedSetItem`);
  await evaluate(`document.querySelector('#urdf-restore').click()`);
  await until(`localStorage.getItem('zekeeparm.urdf:/static-test/') === null`, 'published model restored');
  await until(`${ready} && document.querySelector('#urdf-text').value === ${JSON.stringify(original)}`, 'restored scene loaded');
  await evaluate(`localStorage.setItem('zekeeparm.urdf:/static-test/', '<robot/>')`);
  await send('Page.reload');
  await until(`${ready} && document.querySelector('#urdf-status').textContent.includes('暂未应用')`, 'invalid saved edit fallback');
  assert.equal(await evaluate(`document.querySelector('#urdf-text').value`), '<robot/>');
  await evaluate(`document.querySelector('#urdf-restore').click()`);
  await until(`localStorage.getItem('zekeeparm.urdf:/static-test/') === null`, 'fallback recoverable');
  await until(`${ready} && document.querySelector('#urdf-text').value === ${JSON.stringify(original)}`, 'final scene ready');
  await evaluate(`(() => {
    if(document.querySelector('#toggle-camera-views').getAttribute('aria-checked')==='true') document.querySelector('#toggle-camera-views').click();
    document.querySelector('#obstacle-demo').click();
  })()`);
  await until(`document.querySelector('#study-log').textContent.includes('通过；')`, 'live planner diagnostics', 120000);
  assert.equal(await evaluate(`document.querySelector('#study-distance').textContent.includes('mm')`), true);
  assert.ok(await evaluate(`document.querySelector('#study-log').textContent.includes('采样器')`));
  assert.ok(await evaluate(`document.querySelector('#study-log').textContent.includes('候选路径')`));
  await evaluate(`document.querySelector('#cancel-grasp').click(); document.querySelector('#study-title').scrollIntoView({block:'start'});`);
  const planningScreenshot = await send('Page.captureScreenshot', { format: 'png' });
  await writeFile('/tmp/zekeeparm-teaching-planning.png', Buffer.from(planningScreenshot.data, 'base64'));
  await evaluate(`document.querySelector('#study-clear').click()`);
  assert.equal(await evaluate(`document.querySelector('#study-log').children.length`), 0);
  assert.equal(requests.some(request => request.url.includes('/__model/') || request.method === 'POST'), false, 'static editing must never use a backend API');
  assert.deepEqual(errors, [], 'no uncaught browser errors');
  const screenshot = await send('Page.captureScreenshot', { format: 'png' });
  await writeFile('/tmp/zekeeparm-static-model.png', Buffer.from(screenshot.data, 'base64'));
  console.log('PASS: browser/Python model parity, URDF origins/axes/inertia/colors/limits/effort/velocity/mesh edits, validation, static save/reload/restore/fallback, no backend API');
} finally {
  socket?.close();
  chrome.kill('SIGTERM');
  await new Promise(resolve => chrome.exitCode !== null ? resolve() : chrome.once('exit', resolve));
  await dev?.close();
  if (server) {
    server.httpServer.closeAllConnections();
    await new Promise(resolve => server.httpServer.close(resolve));
  }
  await rm(profile, {recursive:true, force:true, maxRetries:5, retryDelay:200});
}
