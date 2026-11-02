const {defineConfig}=require('@playwright/test');
module.exports=defineConfig({
  testDir:'./tests',testMatch:'*.spec.cjs',workers:1,
  use:{baseURL:'http://127.0.0.1:18765',viewport:{width:1440,height:1000},
    launchOptions:process.env.STUDIO_CHROMIUM?{executablePath:process.env.STUDIO_CHROMIUM}:{},
    screenshot:'only-on-failure'},
  webServer:{command:`${process.env.STUDIO_PYTHON || (process.platform==='win32'?'python':'python3')} studio.py --port 18765 --no-browser --data-dir "${require('node:os').tmpdir()}/zekeeparm-studio-e2e"`,url:'http://127.0.0.1:18765',reuseExistingServer:false}
});
