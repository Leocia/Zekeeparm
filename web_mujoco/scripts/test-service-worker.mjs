import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import vm from 'node:vm';

const source = await readFile(new URL('../public/sw.js', import.meta.url), 'utf8');
const origin = 'https://leocia.github.io';
function worker(path) {
  const events = {}, deleted = [];
  const context = vm.createContext({ URL, Promise,
    self: { registration: { scope: origin + path }, clients: { claim() {} },
      addEventListener(name, callback) { events[name] = callback; } },
    caches: { keys: async () => ['zekeeparm-mujoco-runtime-v3',
      'zekeeparm-site-%2Fother%2F-runtime-v3',
      `zekeeparm-site-${encodeURIComponent(path)}-runtime-v3`,
      `zekeeparm-site-${encodeURIComponent(path)}-runtime-v4`],
    delete: async key => deleted.push(key) } });
  vm.runInContext(source, context);
  return { context, events, deleted };
}
const a = worker('/zekeeparm-urdf-lab/'), b = worker('/zekeeparm-web-mujoco/');
assert.notEqual(vm.runInContext('CACHE_NAME', a.context), vm.runInContext('CACHE_NAME', b.context));
assert.equal(vm.runInContext('CACHE_NAME', a.context).startsWith('zekeeparm-mujoco-'), false,
  'the legacy site must not recognize the new cache as one it can delete');
let activation;
a.events.activate({ waitUntil(promise) { activation = promise; } });
await activation;
assert.deepEqual(a.deleted, ['zekeeparm-site-%2Fzekeeparm-urdf-lab%2F-runtime-v3'],
  'activating this site must preserve other sites and the legacy shared cache');
const removed = [];
a.context.cache = { keys: async () => [
  { url: origin + '/zekeeparm-urdf-lab/models/scene.xml?v=old' },
  { url: origin + '/zekeeparm-web-mujoco/models/scene.xml?v=other' },
  { url: origin + '/zekeeparm-urdf-lab/models/scene.xml?v=new' }],
delete: async request => removed.push(request.url) };
await vm.runInContext("pruneOldModels(cache, 'new')", a.context);
assert.deepEqual(removed, [origin + '/zekeeparm-urdf-lab/models/scene.xml?v=old']);
console.log('PASS: same-domain sites keep separate caches; activation and model pruning preserve other sites');
