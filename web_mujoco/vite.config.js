import { defineConfig } from 'vite';
import { modelSyncPlugin } from './scripts/model-sync-plugin.js';

export default defineConfig({
  base: './',
  plugins: [modelSyncPlugin()],
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },
  assetsInclude: ['**/*.wasm'],
});
