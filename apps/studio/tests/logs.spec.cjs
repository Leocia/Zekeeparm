const {test,expect}=require('@playwright/test');

test('sidebar log starts collapsed, wraps output, and preserves reading position',async({page},testInfo)=>{
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('/');await expect(page.locator('#service-state')).toHaveText('本机服务在线');
 const panel=page.locator('#log-panel'),summary=panel.locator('summary'),content=page.locator('#log-content');
 await expect(summary).toBeVisible();await expect(content).toBeHidden();
 expect(await panel.evaluate(el=>el.closest('.inspector')!==null)).toBe(true);
 const editor=await page.locator('.editor-panel').boundingBox();
 const setMessage=async text=>page.evaluate(async text=>{
  StudioBlocks.load({blocks:{languageVersion:0,blocks:[{type:'studio_start',x:30,y:30,next:{block:{type:'studio_show',inputs:{VALUE:{block:{type:'studio_text',fields:{TEXT:text}}}}}}}]}});
  await new Promise(requestAnimationFrame);
 },text);
 await setMessage('日志输出测试 '+ 'abcdefghijklmnopqrstuvwxyz'.repeat(30));
 await page.getByRole('button',{name:'连接',exact:true}).click();
 await page.getByRole('button',{name:'使能',exact:true}).click();
 await page.locator('#run-program').click();
 await expect(content).toContainText('日志输出测试');await expect(content).toBeHidden();
 await summary.click();await expect(content).toBeVisible();
 expect(await page.locator('.editor-panel').boundingBox()).toEqual(editor);
 const appearance=await content.evaluate(el=>({font:getComputedStyle(el).fontSize,color:getComputedStyle(el).color,background:getComputedStyle(el.parentElement).backgroundColor,
  overflowing:el.scrollHeight>el.clientHeight,atBottom:el.scrollHeight-el.scrollTop-el.clientHeight<3,wraps:el.scrollWidth<=el.clientWidth}));
 expect(appearance).toEqual({font:'14px',color:'rgb(255, 255, 255)',background:'rgb(16, 21, 18)',overflowing:true,atBottom:true,wraps:true});
 await content.evaluate(el=>el.scrollTop=0);await setMessage('第二条日志');
 await expect(page.locator('#run-program')).toBeEnabled();await page.locator('#run-program').click();
 await expect(content).toContainText('第二条日志');expect(await content.evaluate(el=>el.scrollTop)).toBe(0);
 await summary.focus();await page.keyboard.press('Enter');await expect(content).toBeHidden();
 await page.keyboard.press('Enter');await expect(content).toBeVisible();
 await page.getByRole('button',{name:'清空日志',exact:true}).click();
 await expect(content).toHaveText('暂无运行记录');await expect(content).toBeVisible();
 // Wait for later polls, then reload to ensure old events do not return.
 await expect.poll(()=>page.evaluate(async()=> (await (await fetch('/api/state')).json()).events)).toEqual([]);
 await page.reload();await expect(page.locator('#service-state')).toHaveText('本机服务在线');
 await expect(content).toBeHidden();
 await page.getByRole('button',{name:'清空日志',exact:true}).click();await expect(content).toBeHidden();
 await setMessage('清空后的新日志');await page.locator('#run-program').click();
 await expect(content).toContainText('清空后的新日志');await expect(content).not.toContainText('第二条日志');
 await summary.click();await expect(content).toBeVisible();
 for(const width of [1440,1024,768,320]){
  await page.setViewportSize({width,height:1000});
  await expect.poll(()=>page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  const log=await panel.boundingBox(),side=await page.locator('.inspector').boundingBox();
  expect(log.x).toBeGreaterThanOrEqual(side.x);expect(log.x+log.width).toBeLessThanOrEqual(side.x+side.width);
  if(width===1440)expect(log.x).toBeGreaterThan(editor.x+editor.width);
  await page.screenshot({path:testInfo.outputPath(`sidebar-log-${width}.png`),fullPage:true});
 }
 for(const height of [600,400]){
  await page.setViewportSize({width:1440,height});
  await expect.poll(async()=>{const log=await panel.boundingBox(),side=await page.locator('.inspector').boundingBox();return log.y+log.height<=side.y+side.height+1&&log.y+log.height<=height;}).toBe(true);
  await expect(summary).toBeInViewport();
  await page.screenshot({path:testInfo.outputPath(`sidebar-log-height-${height}.png`)});
 }
 await page.setViewportSize({width:1440,height:1000});
 await page.getByRole('button',{name:'断开',exact:true}).click();
 await expect(page.getByRole('button',{name:'连接',exact:true})).toBeVisible();
 expect(errors).toEqual([]);
});
