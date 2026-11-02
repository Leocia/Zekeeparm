const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');

test('diagnostics keep topic/service state without simulation or playback messages', async () => {
  const source = fs.readFileSync(path.join(__dirname, '../public/js/ros/rebot-ros-ui.js'), 'utf8');
  const body = source.slice(source.indexOf('async function runDiagnostics()'), source.indexOf('async function sendTrajectory('));
  const logs = [];
  const topics = ['/zekeep/mujoco/object_states', '/zekeep/sim/animation_event'];
  const gravityStatus = '/zekeep/gravity_compensation/status';
  const context = vm.createContext({
    client: {connected: true, getRosTopics: async () => ({topics}),
      getRosServices: async () => ({services: [gravityStatus]})},
    listedTopics: new Set(), listedServices: new Set(),
    els: {}, REQUIRED_SERVICES: {gravityStatus},
    updateDiagnostics() {}, writeLog: (...args) => logs.push(args), t: key => key,
  });
  vm.runInContext(body, context);
  await context.runDiagnostics();
  assert.deepEqual([...context.listedTopics], topics);
  assert.ok(context.listedServices.has(gravityStatus));
  assert.deepEqual(logs, []);
  assert.doesNotMatch(source, /markSimulationDriverDetected|simulationDriverDetected|log\.lowLevelFallbackInfo/);
  context.client.getRosTopics = async () => { throw new Error('Service unavailable'); };
  await context.runDiagnostics();
  assert.equal(logs.at(-1)[0], 'log.rosapiFallback');
  assert.equal(logs.at(-1)[1], 'warn');
});
