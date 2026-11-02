const {test,expect}=require('@playwright/test');

test('fullscreen preview exposes joint controls, draggable live angles, and program run',async({page},testInfo)=>{
  const runs=[];
  await page.route('**/api/state',async route=>{
    const response=await route.fetch(),state=await response.json();
    await route.fulfill({json:{...state,joints:[0,.7,.8,-.75,0,0],connected:true,enabled:true,busy:false,mode:'simulation',phase:'idle',opening_mm:40}});
  });
  await page.route('**/api/run',async route=>{runs.push(JSON.parse(route.request().postData()));await route.fulfill({json:{}});});
  await page.goto('/');
  await page.locator('#expand-preview').click();
  await expect.poll(()=>page.evaluate(()=>document.fullscreenElement?.classList.contains('preview-panel'))).toBe(true);
  await expect(page.locator('#fullscreen-sliders #range-1')).toBeVisible();
  await expect(page.locator('#fullscreen-sliders .joint-row')).toHaveCount(6);
  await expect(page.locator('#fullscreen-run')).toBeEnabled();
  const handle=page.locator('#joint-state-label'),before=await page.locator('#joint-state-strip').boundingBox();
  await handle.focus();await handle.press('ArrowRight');
  const after=await page.locator('#joint-state-strip').boundingBox();
  expect(after.x).toBeGreaterThan(before.x);
  await page.screenshot({path:testInfo.outputPath('fullscreen-preview.png')});
  await page.locator('#fullscreen-run').click();
  await expect.poll(()=>runs.length).toBe(1);
  expect(runs[0].program.length).toBeGreaterThan(0);
  await page.keyboard.press('Escape');
  await expect(page.locator('#joint-controls')).toBeVisible();
});
