const {test,expect}=require('@playwright/test');

test('readable library stays full size while the program zooms; blocks still drag',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await page.evaluate(async()=>{StudioBlocks.newProgram();await new Promise(requestAnimationFrame);StudioBlocks.workspace.setScale(.4);});
  await page.locator('.blocklyToolboxCategory').filter({hasText:'流程'}).click();
  const sizes=await page.evaluate(()=>{
    const w=StudioBlocks.workspace;
    return {canvas:w.scale,library:w.getFlyout().getWorkspace().scale,
      category:document.querySelector('.blocklyToolbox').getBoundingClientRect().width};
  });
  expect(sizes.canvas).toBe(.4);expect(sizes.library).toBe(1);expect(sizes.category).toBeGreaterThanOrEqual(136);
  const block=page.locator('.blocklyFlyout .blocklyBlockCanvas > .blocklyDraggable').first();
  const source=await block.boundingBox(),canvas=await page.locator('#blockly').boundingBox();
  await page.mouse.move(source.x+35,source.y+15);await page.mouse.down();
  await page.mouse.move(canvas.x+canvas.width-180,canvas.y+180,{steps:25});await page.mouse.up();
  await expect.poll(()=>page.evaluate(()=>StudioBlocks.workspace.getBlocksByType('studio_wait').length)).toBe(1);
  await page.locator('.blocklyToolboxCategory').filter({hasText:'运动学计算'}).click();
  for(const width of [1920,1440,1024,768,320]) {
    await page.setViewportSize({width,height:1000});
    await expect.poll(()=>page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
    if(width>=1440){
      const editor=await page.locator('.editor-panel').boundingBox(),side=await page.locator('.inspector').boundingBox();
      expect(editor.width/side.width).toBeGreaterThan(2.7);
    }else{
      const editor=await page.locator('.editor-panel').boundingBox(),side=await page.locator('.inspector').boundingBox();
      expect(side.y).toBeGreaterThanOrEqual(editor.y+editor.height);
    }
    await page.screenshot({path:testInfo.outputPath(`layout-${width}.png`),fullPage:true});
  }
  expect(errors).toEqual([]);
});
