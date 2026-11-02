const {test,expect}=require('@playwright/test');

async function resetSimulation({request}){
  const headers={'X-Studio-Request':'1'};
  for(const [route,data] of [['stop',{}],['enable',{enabled:false}],['disconnect',{}]])
    await request.post(`/api/${route}`,{headers,data});
}
test.beforeEach(resetSimulation);
test.afterEach(resetSimulation);

test('device enable state follows refreshed feedback without a page reload',async({page})=>{
  let enabled=false;
  await page.route('**/api/state',async route=>{
    const response=await route.fetch(),state=await response.json();
    await route.fulfill({json:{...state,connected:true,mode:'sdk',enabled}});
  });
  await page.goto('/');
  await expect(page.locator('#device-status')).toHaveText('真机 · 未使能');
  enabled=true;
  await expect(page.locator('#device-status')).toHaveText('真机 · 已使能');
  await expect(page.locator('#disable-device')).toBeEnabled();
  await expect(page.locator('#enable-device')).toBeDisabled();
  enabled=false;
  await expect(page.locator('#device-status')).toHaveText('真机 · 未使能');
  await expect(page.locator('#disable-device')).toBeDisabled();
  await expect(page.locator('#enable-device')).toBeEnabled();
});

test('separate disable button and block release simulated motors; preview does not',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await expect(page.locator('#disable-device')).toBeDisabled();
  await page.locator('#connect-device').click();
  await expect(page.locator('#disable-device')).toBeDisabled();
  await page.locator('#enable-device').click();
  await expect(page.locator('#enable-device')).toHaveText('使能');
  await expect(page.locator('#enable-device')).toBeDisabled();
  await page.locator('#disable-device').click();
  await expect(page.locator('#message')).toHaveText('设备已失能');
  await expect(page.locator('#disable-device')).toBeDisabled();
  await page.locator('#enable-device').click();
  const program=await page.evaluate(()=>{
    StudioBlocks.newProgram();StudioBlocks.append('rebot_disable',{});
    const saved=Blockly.serialization.workspaces.save(StudioBlocks.workspace);
    Blockly.serialization.workspaces.load(saved,StudioBlocks.workspace);
    return StudioBlocks.compile();
  });
  expect(program).toEqual([{id:expect.any(String),type:'disable'}]);
  await page.locator('.blocklyToolboxCategory').filter({hasText:'系统与模式'}).click();
  await expect(page.locator('.blocklyToolboxFlyout')).toContainText('失能电机');
  await page.locator('#preview-program').click();
  await expect(page.locator('#message')).toContainText('预览完成');
  await expect(page.locator('#disable-device')).toBeEnabled();
  await page.locator('#run-program').click();
  await expect(page.locator('#progress')).toContainText('运行完成');
  await expect(page.locator('#disable-device')).toBeDisabled();
  await expect(page.locator('#enable-device')).toBeEnabled();
  for(const width of [320,768,1024,1440]){
    await page.setViewportSize({width,height:1000});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  }
  await page.screenshot({path:testInfo.outputPath('disable-ui.png'),fullPage:true});
  expect(errors).toEqual([]);
});

test('real mode requires confirmation for disable button and program, without hardware',async({page})=>{
  await page.route('**/api/state',async route=>{
    const response=await route.fetch(),state=await response.json();
    await route.fulfill({json:{...state,mode:'sdk'}});
  });
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await page.locator('#connect-device').click();
  await page.locator('#enable-device').click();
  await page.evaluate(()=>{StudioBlocks.newProgram();StudioBlocks.append('rebot_disable',{});});
  for(const selector of ['#disable-device','#run-program']){
    let prompt;
    page.once('dialog',async dialog=>{prompt=dialog.message();await dialog.dismiss();});
    await page.locator(selector).click();
    expect(prompt).toContain('释放电机力矩');
    await expect(page.locator('#disable-device')).toBeEnabled();
  }
  page.once('dialog',dialog=>dialog.accept());
  await page.locator('#run-program').click();
  await expect(page.locator('#progress')).toContainText('运行完成');
  await expect(page.locator('#disable-device')).toBeDisabled();
});
