const {test,expect}=require('@playwright/test');

test('slider previews target before motion, then viewport follows live joints',async({page},testInfo)=>{
  const errors=[],runs=[];
  page.on('pageerror',error=>errors.push(error.message));
  page.on('request',request=>{if(request.url().endsWith('/api/run'))runs.push(JSON.parse(request.postData()).program[0].joints);});
  let joints=[0,.7,.8,-.75,0,0];
  await page.route('**/api/state',async route=>{
    const response=await route.fetch(),state=await response.json();
    await route.fulfill({json:{...state,joints,connected:true,enabled:true,busy:false,mode:'simulation',phase:'idle',opening_mm:40}});
  });
  await page.route('**/api/run',async route=>{
    joints=[.2,.3,.4,-.5,.6,.7];
    await route.fulfill({json:{}});
  });
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await expect(page.locator('#model-status')).toBeHidden();
  await expect(page.locator('.robot-scene canvas')).toHaveCount(1);
  await expect(page.locator('#scene canvas')).toHaveCount(1);
  await expect(page.locator('#scene')).toHaveAttribute('aria-label','机械臂姿态预览');
  await expect(page.locator('#target-scene')).toHaveCount(0);
  await expect(page.locator('#joint-state > div')).toHaveCount(6);
  await expect(page.locator('#view-mode')).toHaveText('滑块目标姿态');
  await page.evaluate(()=>{
    const original=window.StudioScene.set;
    window.previewCalls=[];
    window.StudioScene.set=(joints,width)=>{window.previewCalls.push({joints:[...joints],width});original(joints,width);};
  });
  await page.locator('#joint-1').fill('30');
  await expect(page.locator('#view-mode')).toHaveText('滑块目标姿态');
  let call=await page.evaluate(()=>window.previewCalls.at(-1));
  expect(call.joints[0]).toBeCloseTo(Math.PI/6);
  expect(call.width).toBe(40);
  await expect(page.locator('#joint-state-1')).toHaveText('0.0°');
  await expect(page.locator('#joint-1')).toHaveValue('30');
  await expect(page.locator('#move-joints')).toBeEnabled();
  await page.locator('#move-joints').click();
  await expect(page.locator('#view-mode')).toHaveText('机械臂实时状态');
  await expect(page.locator('#joint-state-1')).toHaveText('11.5°');
  expect(runs).toHaveLength(1);
  expect(runs[0][0]).toBeCloseTo(Math.PI/6);
  expect((await page.evaluate(()=>window.previewCalls.at(-1).joints))).toEqual(joints);
  joints=[.4,.2,.3,-.4,.5,.6];
  await expect(page.locator('#joint-state-1')).toHaveText('22.9°');
  expect(await page.evaluate(()=>window.previewCalls.at(-1).joints)).toEqual(joints);
  await page.locator('#range-1').focus();
  await page.locator('#range-1').press('ArrowRight');
  await expect(page.locator('#view-mode')).toHaveText('滑块目标姿态');
  expect(await page.evaluate(()=>window.previewCalls.at(-1).joints[0])).toBeCloseTo(Number(await page.locator('#range-1').inputValue())*Math.PI/180);
  await page.screenshot({path:testInfo.outputPath('target-preview.png'),fullPage:true});
  expect(errors).toEqual([]);
});
