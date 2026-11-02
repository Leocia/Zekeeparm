import assert from 'node:assert/strict';
import { mkdtemp, mkdir, cp, symlink, readFile, writeFile, access, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { createServer } from 'vite';
import { modelSyncPlugin } from './model-sync-plugin.js';

const real = fileURLToPath(new URL('..', import.meta.url));
const temporary = await mkdtemp(resolve(tmpdir(), 'zekeeparm-sync-test-'));
const root = resolve(temporary, 'web_mujoco');
await mkdir(root);
for (const name of ['scripts', 'reference_models', 'src', 'robot', 'public', 'index.html', 'package.json', 'vite.config.js']) {
  await cp(resolve(real, name), resolve(root, name), {
    recursive: true, filter: file => !file.startsWith(resolve(real, 'public/models')) && !file.includes('__pycache__') });
}
// Reuse installed npm dependencies only; the copied project has no ROS workspace beside it.
await symlink(resolve(real, 'node_modules'), resolve(root, 'node_modules'));
await assert.rejects(access(resolve(temporary, 'src')));
const execute = promisify(execFile);
await execute('npm', ['run', 'build'], { cwd: root, timeout: 60000, maxBuffer: 1024 * 1024 });
const server = await createServer({ root, configFile: false, plugins: [modelSyncPlugin()],
  optimizeDeps: { noDiscovery: true, include: [] },
  server: { host: '127.0.0.1', port: 0, watch: { ignored: ['**/node_modules/**'] } }, logLevel: 'error' });
try {
  await server.listen();
  const url = `http://127.0.0.1:${server.httpServer.address().port}/__model/urdf`;
  const get = async () => (await fetch(url)).json();
  let state = await get();
  const source = resolve(root, 'robot/description/urdf/sixaxis.urdf');
  const sourceBefore = await readFile(source);
  const post = (body, headers = {}) => fetch(url, { method: 'POST',
    headers: { 'Content-Type': 'application/json', 'X-Model-Token': state.token, ...headers },
    body: JSON.stringify({ ...state, ...body }) });
  assert.equal((await post({}, { 'X-Model-Token': 'bad' })).status, 403);
  assert.equal((await post({}, { Origin: 'https://example.org' })).status, 403);
  assert.equal((await post({ revision: 'stale' })).status, 409);
  const config = await readFile(resolve(root, 'src/robot-config.json'));
  assert.equal((await post({ urdf: '<robot/>' })).status, 400);
  assert.deepEqual(await readFile(resolve(root, 'src/robot-config.json')), config, 'invalid edits must preserve the last model');
  await assert.rejects(access(resolve(root, 'simulation.urdf')));
  const urdf = state.urdf.replace('0.89804 0.91765 0.92941 1', '0.8 0.8 0.8 1');
  let response = await post({ urdf });
  assert.equal(response.status, 200, JSON.stringify(await response.json()));
  state = await get();
  assert.equal(state.custom, true);
  assert.equal(state.urdf, urdf);
  assert.notDeepEqual(await readFile(resolve(root, 'src/robot-config.json')), config, 'successful edit must rebuild model version');
  response = await post({ restore: true });
  assert.equal(response.status, 200);
  state = await get();
  assert.equal(state.custom, false);
  assert.deepEqual(await readFile(source), sourceBefore);
  const baseline = await readFile(resolve(root, 'src/robot-config.json'), 'utf8');
  await writeFile(resolve(root, 'simulation.urdf'), urdf);
  const deadline = Date.now() + 15000;
  while (await readFile(resolve(root, 'src/robot-config.json'), 'utf8') === baseline) {
    assert.ok(Date.now() < deadline, 'external URDF save must trigger rebuild');
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  state = await get();
  await rm(source);
  state = await get();
  assert.equal(state.sourceRevision, null, 'an existing simulation copy must work without the default URDF');
  response = await post({ urdf: sourceBefore.toString() });
  assert.equal(response.status, 200, JSON.stringify(await response.json()));
  assert.equal((await get()).custom, true);
  await server.close();
  await execute('npm', ['run', 'build'], { cwd: root, timeout: 60000, maxBuffer: 1024 * 1024 });
  console.log('PASS: standalone build, editor sync, restore, file watcher, missing default URDF, stale edits, origin and token guards');
} finally {
  await server.close();
  await rm(temporary, { recursive: true, force: true });
}
