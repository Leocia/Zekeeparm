const fs = require('node:fs');
const path = require('node:path');
const source = path.dirname(require.resolve('blockly'));
const target = path.join(__dirname, 'public', 'vendor', 'blockly');
fs.mkdirSync(target, { recursive: true });
for (const name of ['blockly_compressed.js', 'LICENSE', 'media', 'msg/zh-hans.js']) {
  fs.cpSync(path.join(source, name), path.join(target, name), { recursive: true });
}
console.log('Blockly assets ready for offline use.');
