const {test,expect}=require('@playwright/test');

test('joint angle list snaps into a motion input by dragging',async({page},testInfo)=>{
 await page.goto('/');await expect(page.locator('#service-state')).toHaveText('本机服务在线');
 for(const scale of [1,.55,1.4]){
  const points=await page.evaluate(async scale=>{
   StudioBlocks.newProgram();await new Promise(requestAnimationFrame);
   const w=StudioBlocks.workspace;w.setScale(scale);
   const motion=StudioBlocks.append('studio_move_value');
   const list=w.newBlock('studio_joint_values');list.initSvg();list.render();list.moveBy(340,240);w.scroll(0,0);
   const pt=(b,c)=>{const pos=b.getRelativeToSurfaceXY(),offset=c.getOffsetInBlock();return new DOMPoint(pos.x+offset.x,pos.y+offset.y).matrixTransform(w.getCanvas().getScreenCTM());};
   const target=pt(motion,motion.getInput('VALUE').connection),source=pt(list,list.outputConnection),rect=list.getSvgRoot().getBoundingClientRect();
   // Drop 40 workspace units off target, outside Blockly's original 28-unit radius.
   return {motion:motion.id,x:rect.x+65*scale,y:rect.y+18*scale,dx:target.x-source.x+40*scale,dy:target.y-source.y};
  },scale);
  const canvas=await page.locator('#blockly').boundingBox();
  expect(points.x).toBeGreaterThan(canvas.x);expect(points.x).toBeLessThan(canvas.x+canvas.width);
  expect(points.y).toBeGreaterThan(canvas.y);expect(points.y).toBeLessThan(canvas.y+canvas.height);
  await page.mouse.move(points.x,points.y);await page.mouse.down();await page.mouse.move(points.x+points.dx,points.y+points.dy,{steps:25});await page.mouse.up();
  await expect.poll(()=>page.evaluate(id=>StudioBlocks.workspace.getBlockById(id)?.getInputTargetBlock('VALUE')?.type,points.motion),{message:`Value input should snap at scale ${scale}`}).toBe('studio_joint_values');
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
 }
 await page.screenshot({path:testInfo.outputPath('joint-input.png')});
});
