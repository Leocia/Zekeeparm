import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { createHash, randomBytes } from 'node:crypto';
import { readFile, writeFile, mkdtemp, rename, rm, cp, access } from 'node:fs/promises';
import { resolve, sep } from 'node:path';
import { tmpdir } from 'node:os';

const execute = promisify(execFile);
const digest = text => createHash('sha256').update(text).digest('hex');

export function modelSyncPlugin() {
  let busy = false;
  let timer;
  let pendingFile = null;
  let lastUrdfRevision = null;
  let closed = false;
  const token = randomBytes(24).toString('hex');
  return {
    name: 'sync-urdf-scene',
    handleHotUpdate({ file, server }) {
      if (file === resolve(server.config.root, 'src/robot-config.json')) return [];
    },
    configureServer(server) {
      const root = server.config.root;
      const source = resolve(root, 'robot/description/urdf/sixaxis.urdf');
      const override = resolve(root, 'simulation.urdf');
      const watched = [resolve(root, 'robot'), resolve(root, 'scripts/urdf_model.py'),
        resolve(root, 'reference_models'), resolve(root, 'scripts/export_model.py'),
        resolve(root, 'scripts/model_source.py'), override];
      const isWatched = file => watched.some(path => file === path || file.startsWith(path + sep));
      const current = async () => {
        let custom = true;
        try { await access(override); } catch { custom = false; }
        const urdf = await readFile(custom ? override : source, 'utf8');
        const original = await readFile(source).catch(error => {
          if (error.code === 'ENOENT') return null;
          throw error;
        });
        return { urdf, custom, revision: digest(urdf), sourceRevision: original === null ? null : digest(original) };
      };
      async function rebuild(input, mode = null) {
        if (busy) throw new Error('模型正在同步，请稍后重试');
        busy = true;
        const folder = await mkdtemp(resolve(tmpdir(), 'zekeeparm-model-'));
        try {
          const before = await current();
          const candidate = resolve(folder, 'candidate.urdf');
          await writeFile(candidate, input);
          await execute('python3', ['scripts/export_model.py', '--input', candidate, '--output', folder],
            { cwd: root, timeout: 60000, maxBuffer: 1024 * 1024 });
          const after = await current();
          if (before.revision !== after.revision || before.sourceRevision !== after.sourceRevision) {
            throw new Error('文件在同步期间被修改，请重新读取');
          }
          await cp(resolve(folder, 'public/models'), resolve(root, 'public/models'), { recursive: true });
          const publishing = await current();
          if (before.revision !== publishing.revision || before.sourceRevision !== publishing.sourceRevision) {
            throw new Error('文件在同步期间被修改，请重新读取');
          }
          if (mode === 'save') {
            await writeFile(override + '.tmp', input);
            await rename(override + '.tmp', override);
          }
          if (mode === 'restore') await rm(override, { force: true });
          await cp(resolve(folder, 'src/robot-config.json'), resolve(root, 'src/robot-config.json.tmp'));
          await rename(resolve(root, 'src/robot-config.json.tmp'), resolve(root, 'src/robot-config.json'));
          lastUrdfRevision = digest(input);
          server.ws.send({ type: 'custom', event: 'model:updated', data: {} });
        } finally {
          await rm(folder, { recursive: true, force: true });
          busy = false;
          if (pendingFile && !closed) { const file = pendingFile; pendingFile = null; changed(file); }
        }
      }
      server.watcher.add(watched);
      const changed = file => {
        if (!isWatched(file) || closed) return;
        if (busy) { if (!pendingFile || file !== override) pendingFile = file; return; }
        clearTimeout(timer);
        timer = setTimeout(async () => {
          if (busy) { if (!pendingFile || file !== override) pendingFile = file; return; }
          try {
            const active = await current();
            if (file === override && active.revision === lastUrdfRevision) return;
            await rebuild(active.urdf);
          }
          catch (error) {
            server.config.logger.error(error.stderr || error.message);
            server.ws.send({ type: 'custom', event: 'model:error', data: { message: 'URDF 同步失败，请检查终端；保留上次有效场景。' } });
          }
        }, 350);
      };
      server.watcher.on('change', changed).on('add', changed).on('unlink', changed);
      server.httpServer?.once('close', () => { closed = true; clearTimeout(timer); });
      server.middlewares.use('/__model/urdf', async (req, res) => {
        res.setHeader('Content-Type', 'application/json; charset=utf-8');
        res.setHeader('Cache-Control', 'no-store');
        const reply = (status, body) => { res.statusCode = status; res.end(JSON.stringify(body)); };
        const host = req.headers.host;
        const loopback = ['127.0.0.1', '::1', '::ffff:127.0.0.1'].includes(req.socket.remoteAddress);
        if (!loopback || !/^(localhost|127\.0\.0\.1|\[::1\])(:\d+)?$/.test(host || '') ||
            (req.headers.origin && req.headers.origin !== `http://${host}`)) {
          return reply(403, { error: '仅允许本机同源访问' });
        }
        try {
          if (req.method === 'GET') return reply(200, { ...await current(), token });
          if (req.method !== 'POST') return reply(405, { error: '仅支持 GET/POST' });
          if (req.headers['x-model-token'] !== token || req.headers['content-type'] !== 'application/json') {
            return reply(403, { error: '无效编辑请求' });
          }
          if (busy) return reply(409, { error: '模型正在同步，请稍后重试' });
          const chunks = [];
          let size = 0;
          for await (const chunk of req) {
            size += chunk.length;
            if (size > 600 * 1024) return reply(413, { error: 'URDF 过大' });
            chunks.push(chunk);
          }
          const body = JSON.parse(Buffer.concat(chunks).toString());
          const active = await current();
          if (body.revision !== active.revision || body.sourceRevision !== active.sourceRevision) {
            return reply(409, { error: '文件已被修改，请重新读取后编辑' });
          }
          if (typeof body.urdf !== 'string') return reply(400, { error: '需要 URDF 文本' });
          const input = body.restore ? await readFile(source, 'utf8') : body.urdf;
          await rebuild(input, body.restore ? 'restore' : 'save');
          reply(200, { ok: true });
        } catch (error) {
          reply(400, { error: error.stderr?.trim().split('\n').at(-1) || error.message });
        }
      });
    },
  };
}
