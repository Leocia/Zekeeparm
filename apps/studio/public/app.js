(() => {
  const $=id=>document.getElementById(id), blocks=window.StudioBlocks, ws=blocks.workspace;
  const degrees=180/Math.PI;
  let config, state, points=Object.create(null), polling=false, preview=0, previewActual=false, lastPhase='', pointRevision=0, lastIssue='', pythonBlock=null, collisionTimer, collisionToken=0;
  function message(text,error=false) { $('message').textContent=text;$('message').classList.toggle('error',error); }
  function clearErrors() {
    $('program-error').hidden=true;
    for(const block of ws.getAllBlocks(false))block.setWarningText(null,'program');
    window.StudioScene.setCollisionLinks([]);
  }
  function showError(error) {
    clearErrors();
    window.StudioScene.setCollisionLinks(error.collisionLinks||[]);
    const block=ws.getBlockById(error.blockId);
    if(block)block.setWarningText(error.message,'program');
    $('program-error-text').textContent=(block?`${block.toString(40)}：\n`:'')+error.message;
    const hints=[[/未赋值|变量未定义/,'先用「设置变量」积木赋值，并核对变量名称。'],[/函数不存在/,'添加同名函数定义，或更正调用名称。'],[/递归/,'去掉直接或间接调用自身的函数。'],[/列表.*越界|列表序号/,'检查列表长度：普通列表积木从 1 开始，Python 下标从 0 开始。'],[/除数不能|division by zero/,'检查除数，避免使用 0。'],[/布尔值|类型|需要六个关节/,'检查输入类型；关节运动需要六个 rad 数值，条件需要真或假。'],[/连接值积木|动作须连接/,'把动作接到程序入口，补齐值输入；函数定义独立放置。'],[/Python/,'按提示行号修改代码；给 result 赋值，使用编辑器说明中的语法。']];
    $('program-error-hint').textContent=hints.find(([pattern])=>pattern.test(error.message))?.[1]||'修改对应积木后，重新点击「检查程序」。';
    $('program-error').hidden=false;$('program-error').dataset.blockId=block?.id||'';
    $('locate-error').hidden=!block;
    message(error.message,true);
  }
  async function api(route,body) {
    const response=await fetch(`/api/${route}`,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Request':'1'},body:JSON.stringify(body)});
    const value=await response.json();
    if(!response.ok) throw Object.assign(Error(value.error || '本机服务请求失败'),{blockId:value.block_id,collisionLinks:value.collision_links||[]});
    return value;
  }
  function action(id,fn) { $(id).onclick=async()=>{const program=['check-program','preview-program','run-program'].includes(id);try{if(program){clearErrors();lastIssue='';}await fn();}catch(error){if(program)showError(error);else message(error.message,true);}}; }
  function targets() {return [1,2,3,4,5,6].map(i=>Math.max(i===3?0:config.limits[i-1][0],Math.min(config.limits[i-1][1],Number($(`joint-${i}`).value)/degrees)));}
  function updateTargetPreview() {
    const inputs=[1,2,3,4,5,6].map(i=>$(`joint-${i}`)),openingInput=$('opening');
    clearTimeout(collisionTimer);const token=++collisionToken;window.StudioScene.setCollisionLinks([]);
    if(inputs.some(input=>input.value===''||!Number.isFinite(input.valueAsNumber))||openingInput.value===''||!Number.isFinite(openingInput.valueAsNumber)||openingInput.validity.rangeUnderflow||openingInput.validity.rangeOverflow)return;
    previewActual=false;$('view-mode').textContent='滑块目标姿态';
    const q=targets(),opening=openingInput.valueAsNumber;window.StudioScene.set(q,opening);
    collisionTimer=setTimeout(async()=>{try{const result=await api('collision',{joints:q,opening});if(token===collisionToken)window.StudioScene.setCollisionLinks(result.links);}catch{}},180);
  }
  function setTargets(q) { q.forEach((v,i)=>{v=Math.max(i===2?0:config.limits[i][0],Math.min(config.limits[i][1],v));$(`joint-${i+1}`).value=(v*degrees).toFixed(2);$(`range-${i+1}`).value=(v*degrees).toFixed(2);});updateTargetPreview(); }
  function resizeInspector() {
    const handle=$('resize-handle'),workbench=document.querySelector('.workbench');
    const setWidth=width=>{width=Math.max(320,Math.min(680,innerWidth-640,width));workbench.style.setProperty('--inspector-width',`${width}px`);handle.setAttribute('aria-valuenow',Math.round(width));try{localStorage.setItem('studio-inspector-width',Math.round(width));}catch{}};
    const saved=Number(localStorage.getItem('studio-inspector-width'));if(saved>=320)setWidth(saved);
    handle.onpointerdown=event=>{handle.setPointerCapture(event.pointerId);setWidth(innerWidth-event.clientX-14);};
    handle.onpointermove=event=>{if(handle.hasPointerCapture(event.pointerId))setWidth(innerWidth-event.clientX-14);};
    handle.onkeydown=event=>{if(event.key==='ArrowLeft'||event.key==='ArrowRight'){event.preventDefault();setWidth(parseFloat(getComputedStyle(workbench).gridTemplateColumns.split(' ').at(-1))+(event.key==='ArrowRight'?24:-24));}};
  }
  function payload(program=blocks.compile()) {return {program,points};}
  function motion() {return {id:'manual',type:'move',joints:targets(),duration:Number($('duration').value)};}
  function project() {return {format:'zekeeparm_studio_v1',name:$('project-name').value.trim(),workspace:Blockly.serialization.workspaces.save(ws),points};}
  function cache() {try{localStorage.setItem('zekeeparm-studio-draft',JSON.stringify(project()));}catch(error){message(`草稿未保存：${error.message}`,true);}}
  function renderPoints() {
    blocks.points=points;$('point-count').textContent=`${Object.keys(points).length} 个位置`;
    $('point-list').replaceChildren();
    if(!Object.keys(points).length) {const p=document.createElement('p');p.className='empty';p.textContent='移动机械臂后记录位置，再把位置插入程序。';$('point-list').append(p);}
    for(const name of Object.keys(points)) {
      const row=document.createElement('div');row.className='point-row';
      const dot=document.createElement('i');dot.className='point-dot';
      const label=document.createElement('span');label.textContent=name;label.title=points[name].map(v=>(v*degrees).toFixed(2)+'°').join(' / ');
      const add=document.createElement('button');add.textContent='＋ 积木';add.onclick=()=>{try{blocks.append('studio_point',{POINT:name});}catch(e){message(e.message,true);}};
      const remove=document.createElement('button');remove.textContent='×';remove.setAttribute('aria-label',`删除示教点 ${name}`);remove.onclick=()=>{delete points[name];renderPoints();cache();};
      row.append(dot,label,add,remove);$('point-list').append(row);
    }
  }
  function validateProject(value) {
    if(value?.format!=='zekeeparm_studio_v1' || typeof value.name!=='string' || !value.name.trim() || value.name.length>80 || !value.points || typeof value.points!=='object' || Array.isArray(value.points) || Object.keys(value.points).length>200) throw Error('项目格式无效');
    for(const [name,q] of Object.entries(value.points)) {
      if(!name || name.length>60 || !Array.isArray(q) || q.length!==6 || q.some((v,i)=>typeof v!=='number'||!Number.isFinite(v)||v<config.limits[i][0]||v>config.limits[i][1])) throw Error('示教点数据无效');
    }
  }
  function loadProject(value) {
    validateProject(value);
    const oldPoints=points, oldState=Blockly.serialization.workspaces.save(ws);
    try {points=Object.assign(Object.create(null),value.points);blocks.points=points;blocks.load(value.workspace);}
    catch(error){points=oldPoints;blocks.points=points;Blockly.serialization.workspaces.load(oldState,ws);throw error;}
    $('project-name').value=value.name;renderPoints();cache();
  }
  async function save() {const p=project();await api('projects/save',{name:p.name,workspace:p.workspace,points:p.points});cache();message('项目已保存到本机');}
  function drawState(next) {
    state=next;
    if(state.point_revision>pointRevision) {
      pointRevision=state.point_revision;
      Object.assign(points,state.points);renderPoints();cache();
    }
    $('connect-device').textContent=state.connected?'断开':'连接';
    $('device-mode').disabled=state.connected;
    $('device-channel').disabled=state.connected;
    $('enable-device').disabled=!state.connected||state.enabled||state.busy;
    $('disable-device').disabled=!state.connected||!state.enabled||state.busy;
    $('device-status').textContent=!state.connected?'未连接':state.mode==='simulation'?'模拟设备':state.enabled?'真机 · 已使能':'真机 · 未使能';
    $('run-program').disabled=!state.connected||state.busy||!!preview;
    $('fullscreen-run').disabled=$('run-program').disabled;
    $('check-program').disabled=state.busy;
    $('preview-program').disabled=state.busy||!!preview;
    $('connect-device').disabled=state.busy;
    for(const id of ['move-joints','move-gripper']) $(id).disabled=!state.connected||!state.enabled||state.busy||!!preview;
    $('fullscreen-move').disabled=$('move-joints').disabled;
    $('view-mode').textContent=previewActual?'机械臂实时状态':'滑块目标姿态';
    if(!preview) window.StudioScene.set(previewActual?state.joints:targets(),previewActual?state.opening_mm:Number($('opening').value));
    state.joints.forEach((value,i)=>{$(`joint-state-${i+1}`).textContent=`${(value*degrees).toFixed(2)}°`;});
    $('joint-state-label').textContent=state.connected?'关节实时状态 · °':'关节状态 · 未连接';
    const phases={idle:'等待执行',checking:'检查路径',running:'正在运行',completed:'运行完成',stopped:'已停止',error:'运行失败'};
    $('progress').textContent=`${phases[state.phase]||state.phase}${state.total?` · ${state.completed} / ${state.total}`:''}`;
    ws.highlightBlock(state.busy?state.current:null);
    if(state.error && `${state.error_block}:${state.error}`!==lastIssue){lastIssue=`${state.error_block}:${state.error}`;showError({message:state.error,blockId:state.error_block,collisionLinks:state.error_links});}
    if(!state.error)lastIssue='';
    if(state.phase!==lastPhase) {if(!state.error&&['completed','stopped'].includes(state.phase))message(phases[state.phase]);lastPhase=state.phase;}
    const log=$('log-content'),text=state.events.length?state.events.map(e=>`${e.time}  ${e.message}`).join('\n'):'暂无运行记录';
    if(log.textContent!==text){
      const atBottom=log.scrollHeight-log.scrollTop-log.clientHeight<24;
      log.textContent=text;
      if(atBottom)log.scrollTop=log.scrollHeight;
    }
  }
  async function refresh() {
    if(polling)return;polling=true;
    try {drawState(await api('state'));$('service-state').textContent='本机服务在线';$('service-state').classList.add('online');}
    catch(error){$('service-state').textContent='本机服务已断开';$('service-state').classList.remove('online');$('run-program').disabled=true;message(error.message,true);}
    finally{polling=false;}
  }
  async function run(program) {
    const releasesTorque=steps=>steps.some(step=>step.type==='disable'||['body','then','else'].some(key=>Array.isArray(step[key])&&releasesTorque(step[key])));
    if(state.mode==='sdk'&&releasesTorque(program)&&!confirm('程序包含失能动作，会释放电机力矩。请确保执行到该步骤时机械臂已被可靠支撑，确认运行？'))return;
    cancelPreview();previewActual=true;$('view-mode').textContent='机械臂实时状态';window.StudioScene.set(state.joints,state.opening_mm);await api('run',payload(program));await refresh();
  }
  function cancelPreview() {preview=0;$('view-mode').textContent=previewActual?'机械臂实时状态':'滑块目标姿态';ws.highlightBlock(null);if(state)drawState(state);}
  async function previewProgram() {
    const program=blocks.compile();const checked=await api('check',payload(program));
    const token=Date.now();preview=token;$('view-mode').textContent='程序预览 · 2×';drawState(state);
    message(`预览 ${checked.steps} 个动作，不发送设备命令`);
    let q=[...state.joints],opening=state.opening_mm;
    const flat=checked.prepared;
    for(const step of flat) {
      if(preview!==token)return;ws.highlightBlock(step.id);
      if(step.type==='show') {message(JSON.stringify(step.result));continue;}
      if(['save_point','lifecycle','disable'].includes(step.type)) continue;
      const target=step.type==='move'?step.joints:q;
      const span=step.type==='move'?step.duration:step.type==='wait'?step.seconds:.2;
      const start=performance.now(),from=[...q],oldOpening=opening;
      await new Promise(resolve=>{
        function tick(now){if(preview!==token)return resolve();const t=Math.min(1,(now-start)/(Math.max(span,.01)*500));let s=t*t*(3-2*t);
          if(step.profile==='min_jerk')s=10*t**3-15*t**4+6*t**5;
          else if(step.profile==='linear')s=t;
          else if(step.profile==='trapezoid'){const a=step.accel_ratio,v=1/(1-a);s=t<a?v*t*t/(2*a):t<=1-a?v*(t-a/2):1-v*(1-t)**2/(2*a);}
          q=from.map((v,i)=>v+(target[i]-v)*s);opening=step.type==='gripper'?oldOpening+(step.opening-oldOpening)*s:oldOpening;window.StudioScene.set(q,opening);if(t<1)requestAnimationFrame(tick);else resolve();}requestAnimationFrame(tick);
      });
    }
    if(preview===token){cancelPreview();message('预览完成，已恢复实际姿态');}
  }
  async function updatePose() {
    const pose=await api('fk',{joints:targets()});
    pose.position_m.forEach((v,i)=>$(`pose-${i}`).value=(v*1000).toFixed(2));
    pose.rpy_rad.forEach((v,i)=>$(`pose-${i+3}`).value=(v*degrees).toFixed(2));
  }
  async function init() {
    resizeInspector();
    config=await api('config');
    const previewPanel=document.querySelector('.preview-panel'),jointControls=$('joint-controls'),jointControlsHome=jointControls.parentElement;
    document.addEventListener('fullscreenchange',()=>{
      if(document.fullscreenElement===previewPanel)$('fullscreen-sliders').append(jointControls);
      else jointControlsHome.insertBefore(jointControls,$('opening').parentElement);
    });
    $('fullscreen-run').onclick=()=>$('run-program').click();
    $('fullscreen-move').onclick=()=>$('move-joints').click();
    const dragHandle=$('joint-state-label'),statePanel=$('joint-state-strip');let drag=null;
    dragHandle.tabIndex=0;dragHandle.title='拖动此处移动关节状态窗，也可用方向键微调';
    dragHandle.addEventListener('pointerdown',event=>{
      if(document.fullscreenElement!==previewPanel||event.button!==0)return;
      const rect=statePanel.getBoundingClientRect(),bounds=previewPanel.getBoundingClientRect();statePanel.style.left=`${rect.left-bounds.left}px`;statePanel.style.top=`${rect.top-bounds.top}px`;statePanel.style.right='auto';
      drag={x:event.clientX-rect.left,y:event.clientY-rect.top};dragHandle.setPointerCapture(event.pointerId);event.preventDefault();
    });
    dragHandle.addEventListener('pointermove',event=>{
      if(!drag)return;const bounds=previewPanel.getBoundingClientRect(),rect=statePanel.getBoundingClientRect();
      statePanel.style.left=`${Math.max(0,Math.min(bounds.width-rect.width,event.clientX-bounds.left-drag.x))}px`;
      statePanel.style.top=`${Math.max(48,Math.min(bounds.height-rect.height-150,event.clientY-bounds.top-drag.y))}px`;
    });
    for(const type of ['pointerup','pointercancel','lostpointercapture'])dragHandle.addEventListener(type,()=>{drag=null;});
    dragHandle.addEventListener('keydown',event=>{
      if(document.fullscreenElement!==previewPanel||!event.key.startsWith('Arrow'))return;
      const bounds=previewPanel.getBoundingClientRect(),rect=statePanel.getBoundingClientRect(),step=event.shiftKey?20:8;
      const left=rect.left-bounds.left+(event.key==='ArrowLeft'?-step:event.key==='ArrowRight'?step:0);
      const top=rect.top-bounds.top+(event.key==='ArrowUp'?-step:event.key==='ArrowDown'?step:0);
      statePanel.style.left=`${Math.max(0,Math.min(bounds.width-rect.width,left))}px`;statePanel.style.top=`${Math.max(48,Math.min(bounds.height-rect.height-150,top))}px`;statePanel.style.right='auto';event.preventDefault();
    });
    config.limits.forEach(([lo,hi],i)=>{
      if(i===2)lo=0;
      const row=document.createElement('div');row.className='joint-row';
      const label=document.createElement('label');label.htmlFor=`joint-${i+1}`;label.textContent=`J${i+1}`;
      const slider=document.createElement('input');slider.type='range';slider.id=`range-${i+1}`;slider.min=lo*degrees;slider.max=hi*degrees;slider.step=.01;slider.setAttribute('aria-label',`J${i+1} 目标角度`);
      const input=document.createElement('input');input.type='number';input.id=`joint-${i+1}`;input.min=lo*degrees;input.max=hi*degrees;input.step=.01;
      slider.oninput=()=>{input.value=Number(slider.value).toFixed(2);updateTargetPreview();};input.oninput=()=>{slider.value=input.value;updateTargetPreview();};input.onchange=()=>{if(input.value==='')return;input.value=Math.max(lo*degrees,Math.min(hi*degrees,input.valueAsNumber)).toFixed(2);slider.value=input.value;updateTargetPreview();};
      const unit=document.createElement('span');unit.textContent='°';row.append(label,slider,input,unit);$('joint-controls').append(row);
      const live=document.createElement('div'),name=document.createElement('span'),angle=document.createElement('span');
      name.textContent=`J${i+1}`;angle.id=`joint-state-${i+1}`;angle.textContent='—';live.append(name,angle);$('joint-state').append(live);
    });
    $('opening').oninput=updateTargetPreview;
    ['X / mm','Y / mm','Z / mm','Roll / °','Pitch / °','Yaw / °'].forEach((name,i)=>{const label=document.createElement('label');label.textContent=name;const input=document.createElement('input');input.type='number';input.step='.1';input.id=`pose-${i}`;label.append(input);$('pose-fields').append(label);});
    await refresh();setTargets(state.joints);
    const saved=localStorage.getItem('zekeeparm-studio-draft');
    if(saved){try{loadProject(JSON.parse(saved));message('已恢复上次编辑的草稿');}catch(error){blocks.example();message(`草稿无法恢复：${error.message}`,true);}}
    else blocks.example();
    let timer;
    ws.addChangeListener(event=>{if(event.isUiEvent)return;clearErrors();clearTimeout(timer);timer=setTimeout(()=>{cache();$('program-info').textContent=`${ws.getAllBlocks(false).length} 块积木 · 草稿已保存在浏览器`;},700);});
    $('locate-error').onclick=()=>{const id=$('program-error').dataset.blockId;if(ws.getBlockById(id)){ws.centerOnBlock(id);ws.highlightBlock(id);}};
    blocks.editPython=block=>{
      pythonBlock=block;$('python-code').value=block.getFieldValue('CODE');$('python-result').value=block.getFieldValue('RESULT');$('python-editor-error').textContent='';$('python-dialog').showModal();$('python-code').focus();
    };
    action('edit-python',()=>{const selected=Blockly.getSelected();const block=selected?.type==='studio_python'?selected:ws.getAllBlocks(false).find(b=>b.type==='studio_python')||blocks.append('studio_python');blocks.editPython(block);});
    $('save-python').onclick=event=>{
      event.preventDefault();
      const code=$('python-code').value,name=$('python-result').value.trim();
      if(!code.trim()||code.length>8192||!name||name.length>60){$('python-editor-error').textContent='请输入代码和 1–60 个字符的结果变量名';return;}
      if(!pythonBlock||!ws.getBlockById(pythonBlock.id)){$('python-editor-error').textContent='该积木已被删除';return;}
      Blockly.Events.setGroup(true);
      try{pythonBlock.setFieldValue(code,'CODE');pythonBlock.setFieldValue(name,'RESULT');}finally{Blockly.Events.setGroup(false);}
      $('python-dialog').close();cache();message('Python 代码已保存，点击检查程序查看错误');
    };
    $('python-code').onkeydown=event=>{if(event.key==='Tab'&&!event.shiftKey){event.preventDefault();const editor=$('python-code');editor.setRangeText('    ',editor.selectionStart,editor.selectionEnd,'end');}else if((event.ctrlKey||event.metaKey)&&event.key==='Enter'){event.preventDefault();$('save-python').click();}};
    action('save-project',save);
    action('open-project',async()=>{const data=await api('projects');$('project-list').replaceChildren();data.projects.forEach(name=>{const o=document.createElement('option');o.value=o.textContent=name;$('project-list').append(o);});$('load-project').disabled=!data.projects.length;$('project-dialog').showModal();});
    action('load-project',async()=>{loadProject(await api('projects/load',{name:$('project-list').value}));$('project-dialog').close();message('项目已打开');});
    action('export-project',()=>{const blob=new Blob([JSON.stringify(project(),null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`${project().name.replace(/[<>:"/\\|?*]/g,'_')}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);message('项目文件已导出');});
    action('import-project',()=>$('import-file').click());
    $('import-file').onchange=async e=>{try{const file=e.target.files[0];if(!file)return;if(file.size>1024*1024)throw Error('项目文件不能超过 1 MiB');loadProject(JSON.parse(await file.text()));message('项目已导入');}catch(error){message(error.message,true);}finally{e.target.value='';}};
    action('new-project',()=>{cache();blocks.newProgram();$('project-name').value='未命名程序';message('已新建程序，示教点继续保留');});
    action('example',()=>{blocks.example();message('已载入往复运动示例，可先点击预览');});
    $('device-mode').onchange=()=>{$('device-channel').hidden=$('device-mode').value!=='sdk';$('device-hint').textContent=$('device-mode').value==='sdk'?'连接后显式使能；使用现有设备标定':'模拟设备可体验完整编程流程';};
    action('connect-device',async()=>{if(state.connected)await api('disconnect',{});else await api('connect',{mode:$('device-mode').value,channel:$('device-channel').value});await refresh();setTargets(state.joints);message(state.connected?'设备已连接，点击使能后可运行':'设备已断开');});
    action('enable-device',async()=>{await api('enable',{enabled:true});await refresh();message('设备已使能');});
    action('disable-device',async()=>{if(state.mode==='sdk'&&!confirm('失能会释放电机力矩。请先支撑机械臂，确认失能？'))return;await api('enable',{enabled:false});await refresh();message('设备已失能');});
    action('read-joints',async()=>{await refresh();$('opening').value=state.opening_mm.toFixed(1);setTargets(state.joints);message('已读取当前位置');});
    action('add-move',()=>{const fields={DURATION:$('duration').value};targets().forEach((q,i)=>fields[`J${i+1}`]=(q*degrees).toFixed(2));blocks.append('studio_move',fields);message('已插入关节运动积木');});
    action('move-joints',()=>run([motion()]));
    action('move-gripper',()=>run([{id:'manual-gripper',type:'gripper',opening:Number($('opening').value)}]));
    action('capture-point',async()=>{const name=$('point-name').value.trim();if(!name)throw Error('请输入示教点名称');await refresh();if(!state.connected)throw Error('请先连接设备，再记录当前位置');if(state.busy)throw Error('请等待运动结束再记录');if(Object.hasOwn(points,name))throw Error('名称已存在，请使用其他名称');points[name]=[...state.joints];renderPoints();cache();$('point-name').value='';message(`已记录「${name}」`);});
    action('check-program',async()=>{const r=await api('check',payload());message(`${r.message} · ${r.steps} 个动作，约 ${r.duration.toFixed(1)} 秒`);});
    action('run-program',()=>run(blocks.compile()));
    action('preview-program',previewProgram);
    action('stop-program',async()=>{cancelPreview();await api('stop',{});await refresh();message('任务已停止');});
    action('read-pose',updatePose);
    action('solve-ik',async()=>{const position_m=[0,1,2].map(i=>Number($(`pose-${i}`).value)/1000),rpy_rad=[3,4,5].map(i=>Number($(`pose-${i}`).value)/degrees);const r=await api('ik',{position_m,rpy_rad,seed:targets()});setTargets(r.joints);message('IK 求解成功，关节目标已更新，尚未发送运动');});
    action('clear-log',async()=>{await api('logs/clear',{});await refresh();message('运行日志已清空');});
    $('clear-log').addEventListener('click',event=>event.stopPropagation());
    $('log-panel').ontoggle=()=>{if($('log-panel').open)$('log-content').scrollTop=$('log-content').scrollHeight;};
    $('project-name').onchange=cache;
    document.addEventListener('keydown',e=>{if($('python-dialog').open)return;if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='s'){e.preventDefault();save().catch(error=>message(error.message,true));}});
    window.addEventListener('beforeunload',()=>cache());
    setInterval(refresh,1000/30);
    setInterval(()=>api('heartbeat',{}).catch(()=>{}),1000);
    await updatePose();renderPoints();
  }
  init().catch(error=>{message(`初始化失败：${error.message}`,true);$('service-state').textContent='初始化失败';});
})();
