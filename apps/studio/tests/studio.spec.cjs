const {test,expect}=require('@playwright/test');
test('offline editor, model, simulation, points, files, IK and cancellation',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await expect(page.locator('#model-status')).toBeHidden();
  await expect(page.locator('#run-program')).toBeDisabled();
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
  await page.getByRole('button',{name:'连接',exact:true}).click();
  await page.getByRole('button',{name:'使能',exact:true}).click();
  await expect(page.locator('#run-program')).toBeEnabled();
  await page.locator('#point-name').fill('准备位置');
  await page.getByRole('button',{name:'记录当前',exact:true}).click();
  await expect(page.locator('#point-count')).toHaveText('1 个位置');
  await page.locator('#project-name').fill('浏览器回归测试');
  await page.getByRole('button',{name:'保存',exact:true}).click();
  await expect(page.locator('#message')).toHaveText('项目已保存到本机');
  const downloadPromise=page.waitForEvent('download');
  await page.getByRole('button',{name:'导出',exact:true}).click();
  const download=await downloadPromise;
  expect(download.suggestedFilename()).toBe('浏览器回归测试.json');
  await page.getByRole('button',{name:'新建',exact:true}).click();
  await page.getByRole('button',{name:'打开',exact:true}).click();
  await page.locator('#project-list').selectOption('浏览器回归测试');
  await page.getByRole('button',{name:'打开项目',exact:true}).click();
  await expect(page.locator('#project-name')).toHaveValue('浏览器回归测试');
  await page.locator('.pose-panel summary').click();
  await page.getByRole('button',{name:'读取目标位姿',exact:true}).click();
  await page.getByRole('button',{name:'求解关节角',exact:true}).click();
  await expect(page.locator('#message')).toContainText('IK 求解成功');
  await page.getByRole('button',{name:'▶ 运行',exact:true}).click();
  await expect(page.locator('#progress')).toContainText('正在运行');
  await page.getByRole('button',{name:'■ 停止',exact:true}).click();
  await expect(page.locator('#progress')).toContainText('已停止');
  await expect(page.locator('#run-program')).toBeEnabled();
  await page.getByRole('button',{name:'▶ 预览',exact:true}).click();
  await expect(page.locator('#view-mode')).toHaveText('程序预览 · 2×');
  await page.getByRole('button',{name:'■ 停止',exact:true}).click();
  await expect(page.locator('#view-mode')).toHaveText('实际姿态');
  await page.screenshot({path:testInfo.outputPath('studio.png')});
  for(const width of [320,768,1024,1440]){
    await page.setViewportSize({width,height:1000});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  }
  expect(errors).toEqual([]);
});

test('extended blocks compile, roundtrip, preview and execute in simulation',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await expect(page.locator('#model-status')).toBeHidden();
  for(const category of ['系统与模式','关节与示教','运动学计算','流程']) {
    await page.locator('.blocklyToolboxCategory').filter({hasText:category}).click();
    expect(await page.locator('.blocklyFlyout .blocklyBlockCanvas > .blocklyDraggable').count()).toBeGreaterThan(0);
  }
  await page.evaluate(async()=>{
    const state=await (await fetch('/api/state')).json();
    if(state.connected) await fetch('/api/disconnect',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Request':'1'},body:'{}'});
  });
  await expect(page.getByRole('button',{name:'连接',exact:true})).toBeVisible();
  const compiled=await page.evaluate(async()=>{
    const api=window.StudioBlocks,ws=api.workspace;
    api.newProgram();
    const make=(type,fields={})=>{const b=ws.newBlock(type);Object.entries(fields).forEach(([k,v])=>b.setFieldValue(String(v),k));b.initSvg();b.render();return b;};
    const connect=(parent,input,child)=>parent.getInput(input).connection.connect(child.outputConnection);
    const ready=(await (await fetch('/api/state')).json()).joints;
    const response=await fetch('/api/fk',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Request':'1'},body:JSON.stringify({joints:ready})});
    const pose=await response.json(),deg=180/Math.PI;
    const poseFields={};['X','Y','Z'].forEach((n,i)=>poseFields[n]=pose.position_m[i]*1000);['R','P','YAW'].forEach((n,i)=>poseFields[n]=pose.rpy_rad[i]*deg);
    api.append('rebot_controller_lifecycle');
    api.append('rebot_set_traj_profile');
    api.append('rebot_move_single_joint_delta',{DELTA:.1,DURATION:.2});
    api.append('rebot_save_current_teach_point',{NAME:'积木示教点'});
    api.append('studio_point',{POINT:'积木示教点',DURATION:.2});
    const show=type=>{const parent=api.append('studio_show');const child=make(type);connect(parent,'VALUE',child);return child;};
    show('rebot_compute_fk');
    show('rebot_model_info');
    const stats=show('rebot_check_traj_stats');stats.setFieldValue('.2','DURATION');
    connect(stats,'TARGET',make('studio_current_joints'));
    const branch=api.append('studio_if');
    connect(branch,'CONDITION',make('rebot_is_pose_reachable',poseFields));
    const wait=make('studio_wait',{SECONDS:0});branch.getInput('THEN').connection.connect(wait.previousConnection);
    const move=api.append('studio_move_value',{DURATION:.2});
    connect(move,'VALUE',make('rebot_compute_ik',poseFields));
    const literal=show('studio_joint_values');ready.forEach((q,i)=>literal.setFieldValue(String(q*deg),`J${i+1}`));
    const before=api.compile(),saved=Blockly.serialization.workspaces.save(ws);
    const legacy=saved.blocks.blocks.find(b=>b.type==='studio_start').next.block;
    legacy.fields.GRAVITY='on';
    api.load(saved);
    return {before,after:api.compile(),removed:ws.getAllBlocks(false).find(b=>b.type==='rebot_controller_lifecycle').getField('GRAVITY')==null,legacyUnchanged:legacy.fields.GRAVITY==='on',cleanSave:!JSON.stringify(Blockly.serialization.workspaces.save(ws)).includes('"GRAVITY"')};
  });
  expect(compiled.after).toEqual(compiled.before);
  expect(compiled.removed).toBe(true);expect(compiled.legacyUnchanged).toBe(true);expect(compiled.cleanSave).toBe(true);
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
  await expect(page.locator('#point-count')).toHaveText('0 个位置');
  await page.getByRole('button',{name:'▶ 预览',exact:true}).click();
  await expect(page.locator('#message')).toContainText('预览完成',{timeout:15000});
  await expect(page.locator('#point-count')).toHaveText('0 个位置');
  await page.getByRole('button',{name:'连接',exact:true}).click();
  // Lifecycle block must be able to enable a connected, initially disabled session.
  await page.getByRole('button',{name:'▶ 运行',exact:true}).click();
  await expect(page.locator('#progress')).toContainText('运行完成',{timeout:15000});
  await expect(page.locator('#point-count')).toHaveText('1 个位置');
  await page.locator('#log-panel > summary').click();
  await expect(page.locator('#log-content')).toContainText('success_rate');
  await expect(page.locator('#log-content')).toContainText('frame_names');
  await expect(page.locator('#log-content')).toContainText('已记录示教点 积木示教点');
  await page.locator('#log-panel > summary').click();
  await page.locator('.blocklyToolboxCategory').filter({hasText:'运动学计算'}).click();
  await page.screenshot({path:testInfo.outputPath('extended-blocks.png')});
  expect(errors).toEqual([]);
});

test('common logic blocks, functions and variable driven joint motion',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await page.evaluate(async()=>{
    const s=await (await fetch('/api/state')).json();
    if(s.connected)await fetch('/api/disconnect',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Request':'1'},body:'{}'});
  });
  await expect(page.getByRole('button',{name:'连接',exact:true})).toBeVisible();
  for(const category of ['变量','数学','逻辑','数据','函数']){
    await page.locator('.blocklyToolboxCategory').filter({hasText:category}).click();
    expect(await page.locator('.blocklyFlyout .blocklyBlockCanvas > .blocklyDraggable').count()).toBeGreaterThan(0);
  }
  const result=await page.evaluate(()=>{
    const api=window.StudioBlocks,ws=api.workspace;api.newProgram();
    const make=(type,fields={})=>{const b=ws.newBlock(type);Object.entries(fields).forEach(([k,v])=>b.setFieldValue(String(v),k));b.initSvg();b.render();return b;};
    const connect=(parent,input,child)=>{parent.getInput(input).connection.connect(child.outputConnection);return parent;};
    const num=v=>make('studio_number',{NUMBER:v}),variable=n=>make('studio_variable',{NAME:n});
    const set=(name,value)=>connect(api.append('studio_set',{NAME:name}),'VALUE',value);
    const binary=(type,op,a,b)=>{const block=make(type,{OP:op});connect(block,'A',a);connect(block,'B',b);return block;};
    api.append('rebot_controller_lifecycle');
    set('次数',num(0));set('目标',make('studio_current_joints'));
    const list=make('studio_list');list.setFieldValue('6','COUNT');
    for(let i=0;i<6;i++)connect(list,`ITEM${i}`,num(i+1));
    set('测试列表',list);
    const fn=make('studio_function',{NAME:'微调一次'});fn.moveBy(500,40);
    const change=connect(make('studio_change',{NAME:'次数'}),'VALUE',num(1));
    const replace=make('studio_list_set',{NAME:'目标'});connect(replace,'INDEX',num(1));
    const get=make('studio_list_get');connect(get,'LIST',variable('目标'));connect(get,'INDEX',num(1));
    const radians=connect(make('studio_unary',{OP:'to_rad'}),'VALUE',num(.1));
    connect(replace,'VALUE',binary('studio_arithmetic','add',get,radians));
    fn.getInput('BODY').connection.connect(change.previousConnection);change.nextConnection.connect(replace.previousConnection);
    const repeat=api.append('studio_repeat',{COUNT:2});
    repeat.getInput('BODY').connection.connect(make('studio_call',{NAME:'微调一次'}).previousConnection);
    const stats=make('rebot_check_traj_stats',{DURATION:.2});set('统计',stats);
    const rate=connect(make('studio_dict_get',{KEY:'success_rate'}),'DICT',variable('统计'));
    const comparison=binary('studio_compare','ge',rate,num(.99));
    const count=binary('studio_compare','eq',variable('次数'),num(2));
    const not=connect(make('studio_not'),'VALUE',make('studio_boolean',{BOOL:'false'}));
    const condition=binary('studio_logic','and',comparison,binary('studio_logic','and',count,not));
    const branch=connect(api.append('studio_if'),'CONDITION',condition);
    const move=connect(make('studio_move_value',{DURATION:.2}),'VALUE',variable('目标'));
    branch.getInput('THEN').connection.connect(move.previousConnection);
    const show=value=>connect(api.append('studio_show'),'VALUE',value);
    show(connect(make('studio_length'),'VALUE',variable('测试列表')));
    show(make('studio_text',{TEXT:'逻辑积木测试成功'}));
    const before=api.compile(),saved=Blockly.serialization.workspaces.save(ws);api.load(saved);
    const restored=ws.getAllBlocks(false).find(b=>b.type==='studio_list');
    return {before,after:api.compile(),items:restored.itemCount};
  });
  expect(result.items).toBe(6);expect(result.after).toEqual(result.before);
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
  await page.getByRole('button',{name:'▶ 预览',exact:true}).click();
  await expect(page.locator('#message')).toContainText('预览完成',{timeout:10000});
  await page.getByRole('button',{name:'连接',exact:true}).click();
  await page.getByRole('button',{name:'▶ 运行',exact:true}).click();
  await expect(page.locator('#progress')).toContainText('运行完成',{timeout:10000});
  await page.locator('#log-panel > summary').click();
  await expect(page.locator('#log-content')).toContainText('逻辑积木测试成功');
  await expect(page.locator('#log-content')).toContainText('"variable": "次数", "value": 2');
  await page.locator('#log-panel > summary').click();
  await page.locator('.blocklyToolboxCategory').filter({hasText:'数据'}).click();
  await page.screenshot({path:testInfo.outputPath('common-logic.png')});
  expect(errors).toEqual([]);
});

test('Python editor roundtrip, calculation and simulation',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  await page.evaluate(async()=>{const s=await(await fetch('/api/state')).json();if(s.connected)await fetch('/api/disconnect',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Request':'1'},body:'{}'});});
  await expect(page.getByRole('button',{name:'连接',exact:true})).toBeVisible();
  await page.getByRole('button',{name:'新建',exact:true}).click();
  await page.locator('#edit-python').click();
  const code='import math\nresult = list(joints)\nresult[0] += math.radians(0.1)\nvariables["说明"] = "Python已执行"\nprint("Python计算完成", len(result))';
  await page.locator('#python-code').fill(code);await page.locator('#python-result').fill('Python目标');
  await page.getByRole('button',{name:'保存代码',exact:true}).click();
  await expect(page.locator('#python-dialog')).not.toBeVisible();
  const result=await page.evaluate(()=>{
    const api=window.StudioBlocks,ws=api.workspace;
    const variable=n=>{const b=ws.newBlock('studio_variable');b.setFieldValue(n,'NAME');b.initSvg();b.render();return b;};
    const move=api.append('studio_move_value',{DURATION:.2});move.getInput('VALUE').connection.connect(variable('Python目标').outputConnection);
    const show=api.append('studio_show');show.getInput('VALUE').connection.connect(variable('说明').outputConnection);
    const before=api.compile(),saved=Blockly.serialization.workspaces.save(ws);api.load(saved);
    return {before,after:api.compile(),code:ws.getAllBlocks(false).find(b=>b.type==='studio_python').getFieldValue('CODE')};
  });
  expect(result.code).toBe(code);expect(result.before).toEqual(result.after);
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
  await page.getByRole('button',{name:'▶ 预览',exact:true}).click();
  await expect(page.locator('#message')).toContainText('预览完成',{timeout:10000});
  await page.getByRole('button',{name:'连接',exact:true}).click();
  await page.getByRole('button',{name:'使能',exact:true}).click();
  await page.getByRole('button',{name:'▶ 运行',exact:true}).click();
  await expect(page.locator('#progress')).toContainText('运行完成',{timeout:10000});
  await page.locator('#log-panel > summary').click();
  await expect(page.locator('#log-content')).toContainText('Python计算完成 6');
  await expect(page.locator('#log-content')).toContainText('Python已执行');
  await page.locator('#log-panel > summary').click();
  await page.locator('#edit-python').click();
  await expect(page.locator('#python-code')).toHaveValue(code);
  await page.screenshot({path:testInfo.outputPath('python-editor.png')});
  expect(errors).toEqual([]);
});

test('manual checks identify logical and Python errors without editing alerts',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');await expect(page.locator('#service-state')).toHaveText('本机服务在线');
  for(const [kind,expected] of [['variable','未赋值'],['condition','布尔值'],['index','列表序号'],['call','函数不存在'],['division','除数不能'],['python','第 2 行']]){
    const id=await page.evaluate(kind=>{
      const api=window.StudioBlocks,ws=api.workspace;api.newProgram();
      const make=(type,fields={})=>{const b=ws.newBlock(type);Object.entries(fields).forEach(([k,v])=>b.setFieldValue(String(v),k));b.initSvg();b.render();return b;};
      const connect=(parent,input,child)=>{parent.getInput(input).connection.connect(child.outputConnection);return parent;};
      const num=v=>make('studio_number',{NUMBER:v});
      if(kind==='call')return api.append('studio_call',{NAME:'缺少的函数'}).id;
      if(kind==='python')return api.append('studio_python',{CODE:'result = 1\nresult = 1 / 0'}).id;
      if(kind==='condition'){
        connect(api.append('studio_set',{NAME:'数值'}),'VALUE',num(1));
        return connect(api.append('studio_if'),'CONDITION',make('studio_variable',{NAME:'数值'})).id;
      }
      const show=api.append('studio_show');
      if(kind==='variable')connect(show,'VALUE',make('studio_variable',{NAME:'未设置'}));
      if(kind==='index'){
        const list=make('studio_list');list.setFieldValue('0','COUNT');
        const get=connect(make('studio_list_get'),'LIST',list);connect(get,'INDEX',num(1));connect(show,'VALUE',get);
      }
      if(kind==='division'){
        const division=connect(make('studio_arithmetic',{OP:'div'}),'A',num(1));connect(division,'B',num(0));connect(show,'VALUE',division);
      }
      return show.id;
    },kind);
    await page.getByRole('button',{name:'检查程序',exact:true}).click();
    await expect(page.locator('#program-error-text')).toContainText(expected);
    await expect(page.locator('#program-error')).toHaveAttribute('data-block-id',id);
    await page.getByRole('button',{name:'定位积木',exact:true}).click();
    expect(await page.evaluate(id=>!!window.StudioBlocks.workspace.getBlockById(id).getIcon(Blockly.icons.IconType.WARNING)?.getText(),id)).toBe(true);
  }
  const id=await page.evaluate(()=>{const a=window.StudioBlocks;a.newProgram();return a.append('studio_show').id;});
  // Wait for the editor's debounce to finish: unfinished blocks stay quiet until checked.
  await expect(page.locator('#program-info')).toContainText('2 块积木 · 草稿已保存在浏览器');
  await expect(page.locator('#program-error')).toBeHidden();
  expect(await page.evaluate(id=>!window.StudioBlocks.workspace.getBlockById(id).getIcon(Blockly.icons.IconType.WARNING),id)).toBe(true);
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#program-error-text')).toContainText('连接值积木');
  await expect(page.locator('#program-error')).toHaveAttribute('data-block-id',id);
  await page.screenshot({path:testInfo.outputPath('program-error.png')});
  for(const width of [320,768,1440]){await page.setViewportSize({width,height:1000});expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);}
  await page.evaluate(id=>{const ws=window.StudioBlocks.workspace,b=ws.newBlock('studio_number');b.initSvg();b.render();ws.getBlockById(id).getInput('VALUE').connection.connect(b.outputConnection);},id);
  await expect(page.locator('#program-error')).toBeHidden();
  await page.getByRole('button',{name:'检查程序',exact:true}).click();
  await expect(page.locator('#message')).toContainText('自碰撞检查通过');
  expect(errors).toEqual([]);
});
