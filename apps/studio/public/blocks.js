(() => {
  const rad = Math.PI / 180;
  const limits = [[-2.58,2.58],[0,3.7],[-.01,3.7],[-1.57,1.57],[-1.57,1.57],[-1.57,1.57]];
  const definitions = [
    {type:'studio_start',message0:'当点击运行',nextStatement:null,colour:'#53684d',tooltip:'程序从这里开始，只执行连接在下方的积木。'},
    {type:'studio_wait',message0:'等待 %1 秒',args0:[{type:'field_number',name:'SECONDS',value:1,min:0,max:60,precision:.1}],previousStatement:null,nextStatement:null,colour:'#896633'},
    {type:'studio_gripper',message0:'夹爪开口 %1 mm',args0:[{type:'field_number',name:'OPENING',value:40,min:0,max:65.17241379310346,precision:.1}],previousStatement:null,nextStatement:null,colour:'#397c78'},
    {type:'studio_repeat',message0:'重复 %1 次',args0:[{type:'field_number',name:'COUNT',value:3,min:1,max:100,precision:1}],message1:'%1',args1:[{type:'input_statement',name:'BODY'}],previousStatement:null,nextStatement:null,colour:'#896633'},
    {type:'studio_move',message0:'关节运动 %1 秒',args0:[{type:'field_number',name:'DURATION',value:2,min:.2,max:60,precision:.1}],message1:'J1 %1 °  J2 %2 °  J3 %3 °',args1:[0,1,2].map(i=>({type:'field_number',name:`J${i+1}`,value:[0,40,47][i],min:limits[i][0]/rad,max:limits[i][1]/rad,precision:.1})),message2:'J4 %1 °  J5 %2 °  J6 %3 °',args2:[3,4,5].map(i=>({type:'field_number',name:`J${i+1}`,value:i===3?-43:0,min:limits[i][0]/rad,max:limits[i][1]/rad,precision:.1})),previousStatement:null,nextStatement:null,colour:'#a94e21',tooltip:'同步关节运动，末端不保证走直线。'}
  ];
  const numeric=(name,value,min,max,precision=.1)=>({type:'field_number',name,value,min,max,precision});
  const dropdown=(name,options)=>({type:'field_dropdown',name,options});
  const input=(name,check)=>({type:'input_value',name,...(check?{check}:{})});
  const statement=(type,message0,args0,extra={})=>({type,message0,args0,inputsInline:false,previousStatement:null,nextStatement:null,colour:'#53684d',...extra});
  const value=(type,message0,args0,output,extra={})=>({type,message0,args0,inputsInline:false,output,colour:'#576e96',...extra});
  const profiles=[['最小冲击','min_jerk'],['梯形速度','trapezoid'],['线性匀速','linear']];
  const poseFields=()=>['X','Y','Z','R','P','YAW'].map((name,i)=>numeric(name,[250,0,200,0,0,0][i],i<3?-2000:-180,i<3?2000:180));
  const poseRows={message1:'X %1  Y %2  Z %3 mm',args1:poseFields().slice(0,3),message2:'Roll %1  Pitch %2  Yaw %3 °',args2:poseFields().slice(3)};
  definitions.push(
    statement('rebot_disable','失能电机',[],{tooltip:'释放机械臂和夹爪的电机力矩，请先可靠支撑机械臂；再次运动前须启动并使能。'}),
    statement('rebot_controller_lifecycle','控制器 %1',[dropdown('ACTION',[['启动并使能','start'],['安全回零并保持','end']])],{message1:'模式 %1',args1:[dropdown('MODE',[['位置速度 posvel','posvel'],['阻抗 MIT','mit']])],tooltip:'真机模式须匹配硬件配置；结束时回零并保持力矩，失能须显式操作。'}),
    statement('rebot_safe_home','安全回零  最大角速度 %1 °/s',[numeric('MAX_VEL',8.6,.6,17.18)],{message1:'到位阈值 %1 °  超时 %2 秒',args1:[numeric('SETTLE',.57,.06,2.86,.01),numeric('TIMEOUT',30,1,120,1)],tooltip:'回到硬件配置 safe_home 位置；最小冲击曲线，检查自碰撞，超时保持。'}),
    statement('rebot_set_traj_profile','轨迹曲线 %1',[dropdown('PROFILE',profiles)],{message1:'周期 %1 秒  加速占比 %2',args1:[numeric('DT',.02,.02,.1,.01),numeric('ACCEL',.25,.01,.49,.01)]}),
    statement('rebot_move_single_joint_delta','单关节微调 %1  增量 %2 °  用时 %3 秒',[dropdown('JOINT',[1,2,3,4,5,6].map(i=>[`J${i}`,String(i-1)])),numeric('DELTA',5,-180,180),numeric('DURATION',2,.2,60)],{colour:'#a94e21',tooltip:'运行到此处时基于实际关节位置微调，自动钳制到关节限位。'}),
    statement('rebot_save_current_teach_point','记录当前姿态为示教点 %1',[{type:'field_input',name:'NAME',text:'P1'}],{colour:'#397c78',tooltip:'运行到此处读取实际关节及位姿；同名点拒绝覆盖，运行后随项目保存。'}),
    value('studio_current_joints','当前六轴关节角 rad',[],'Array'),
    value('studio_joint_values','关节角列表',[],'Array',{message1:definitions[4].message1,args1:definitions[4].args1,message2:definitions[4].message2,args2:definitions[4].args2}),
    value('rebot_compute_fk','正运动学 FK  输出 %1  坐标系 %2',[
      dropdown('OUTPUT',[['位置 m','position_m'],['旋转矩阵 3×3','rotation'],['变换矩阵 4×4','transform'],['完整位姿','pose']]),{type:'field_input',name:'FRAME',text:'official_tcp'}],null,
      {message1:'关节角 %1',args1:[input('JOINTS','Array')],tooltip:'未接关节角时，使用程序当前关节角。坐标系默认沿用本机模型 official_tcp。'}),
    value('rebot_compute_ik','逆运动学 IK  随机重试 %1 次',[numeric('RETRIES',0,0,8,1)],'Array',poseRows),
    value('rebot_is_pose_reachable','目标位姿可达？ 随机重试 %1 次',[numeric('RETRIES',0,0,8,1)],'Boolean',{...poseRows,tooltip:'仅判断限位内 IK 收敛，不代表路径无碰撞；运动前仍检查路径。'}),
    value('rebot_check_traj_stats','预检笛卡尔轨迹质量  用时 %1 秒',[numeric('DURATION',2,.2,60)],null,
      {message1:'目标关节角 %1',args1:[input('TARGET','Array')],message2:'曲线 %1  周期 %2 秒  加速占比 %3',args2:[dropdown('PROFILE',profiles),numeric('DT',.05,.02,1,.01),numeric('ACCEL',.25,.01,.49,.01)],tooltip:'不发送运动。输出成功率、最大/平均 IK 误差及总采样数，最多 501 点。'}),
    value('rebot_model_info','机器人模型信息',[],null),
    statement('studio_show','显示结果到日志 %1',[input('VALUE')],{colour:'#576e96'}),
    statement('studio_move_value','运动到关节角 %1  用时 %2 秒',[input('VALUE','Array'),numeric('DURATION',2,.2,60)],{colour:'#a94e21'}),
    statement('studio_if','如果 %1',[input('CONDITION','Boolean')],{colour:'#896633',message1:'那么 %1',args1:[{type:'input_statement',name:'THEN'}],message2:'否则 %1',args2:[{type:'input_statement',name:'ELSE'}]})
  );
  const variableName=()=>({type:'field_input',name:'NAME',text:'变量1'});
  const logicColour='#896633', dataColour='#397c78';
  definitions.push(
    value('studio_number','数字 %1',[numeric('NUMBER',1,-1e12,1e12,0)],'Number'),
    value('studio_text','文本 %1',[{type:'field_input',name:'TEXT',text:'你好'}],'String',{colour:dataColour}),
    value('studio_boolean','%1',[dropdown('BOOL',[['真','true'],['假','false']])],'Boolean',{colour:logicColour}),
    value('studio_variable','变量 %1',[variableName()],null,{colour:'#8c5a78'}),
    statement('studio_set','设置变量 %1 为 %2',[variableName(),input('VALUE')],{colour:'#8c5a78'}),
    statement('studio_change','变量 %1 增加 %2',[variableName(),input('VALUE','Number')],{colour:'#8c5a78'}),
    value('studio_arithmetic','%1 %2 %3',[input('A','Number'),dropdown('OP',[['＋','add'],['－','sub'],['×','mul'],['÷','div'],['取余','mod']]),input('B','Number')],'Number',{inputsInline:true}),
    value('studio_unary','%1 %2',[dropdown('OP',[['取整 round','round'],['向下取整','floor'],['向上取整','ceil'],['绝对值','abs'],['取负','neg'],['角度转弧度','to_rad'],['弧度转角度','to_deg']]),input('VALUE','Number')],'Number'),
    value('studio_compare','%1 %2 %3',[input('A'),dropdown('OP',[['＝','eq'],['≠','ne'],['＜','lt'],['≤','le'],['＞','gt'],['≥','ge']]),input('B')],'Boolean',{colour:logicColour,inputsInline:true}),
    value('studio_logic','%1 %2 %3',[input('A','Boolean'),dropdown('OP',[['并且','and'],['或者','or']]),input('B','Boolean')],'Boolean',{colour:logicColour,inputsInline:true}),
    value('studio_not','取反 %1',[input('VALUE','Boolean')],'Boolean',{colour:logicColour}),
    value('studio_list_get','取列表 %1 的第 %2 项',[input('LIST','Array'),input('INDEX','Number')],null,{colour:dataColour,tooltip:'序号从 1 开始，J1 对应第 1 项。'}),
    statement('studio_list_set','将列表变量 %1 的第 %2 项设为 %3',[variableName(),input('INDEX','Number'),input('VALUE')],{colour:dataColour}),
    value('studio_length','长度 %1',[input('VALUE')],'Number',{colour:dataColour,tooltip:'返回列表、字典或文本的长度。'}),
    value('studio_dict_get','取字典 %1 的字段 %2',[input('DICT'),{type:'field_input',name:'KEY',text:'success_rate'}],null,{colour:dataColour}),
    {type:'studio_function',message0:'定义函数 %1',args0:[{type:'field_input',name:'NAME',text:'动作组合'}],message1:'执行 %1',args1:[{type:'input_statement',name:'BODY'}],colour:'#8c5a78',tooltip:'独立放在工作区，调用时执行内部动作。无参数，共享程序变量，不支持递归。'},
    statement('studio_call','调用函数 %1',[{type:'field_input',name:'NAME',text:'动作组合'}],{colour:'#8c5a78'})
  );
  Blockly.common.defineBlocksWithJsonArray(definitions);
  const api = {points:Object.create(null), workspace:null};
  Blockly.Blocks.studio_point = {init() {
    this.appendDummyInput().appendField('移动到').appendField(new Blockly.FieldDropdown(function() {
      const recorded=api.workspace?.getAllBlocks(false).filter(b=>b.type==='rebot_save_current_teach_point').map(b=>b.getFieldValue('NAME').trim()).filter(Boolean) || [];
      const names=[...new Set([...Object.keys(api.points),...recorded])];
      const current=this.getValue();
      if(current && current!=='__empty__' && !names.includes(current)) names.push(current);
      return names.length ? names.map(n=>[n,n]) : [['请先记录示教点','__empty__']];
    }), 'POINT');
    this.appendDummyInput().appendField('用时').appendField(new Blockly.FieldNumber(2,.2,60,.1),'DURATION').appendField('秒');
    this.setPreviousStatement(true); this.setNextStatement(true); this.setColour('#397c78');
  }};
  Blockly.Blocks.studio_list = {
    init() {
      this.itemCount=0;
      this.appendDummyInput('HEADER').appendField('创建列表，项数').appendField(new Blockly.FieldNumber(3,0,32,1,function(count) {
        this.getSourceBlock()?.updateItems(count);return count;
      }),'COUNT');
      this.setOutput(true,'Array');this.setColour(dataColour);this.setInputsInline(false);
      this.updateItems(3);
    },
    updateItems(count) {
      if(!Number.isInteger(count)||count<0||count>32) throw Error('列表项数需为 0–32 的整数');
      for(let i=this.itemCount;i<count;i++)this.appendValueInput(`ITEM${i}`).appendField(`第 ${i+1} 项`);
      for(let i=count;i<this.itemCount;i++)this.removeInput(`ITEM${i}`);
      this.itemCount=count;
    },
    saveExtraState(){return {count:this.itemCount};},
    loadExtraState(state){this.updateItems(state.count);}
  };
  const numberShadow=(n=1)=>({shadow:{type:'studio_number',fields:{NUMBER:n}}});
  const boolShadow=(b='true')=>({shadow:{type:'studio_boolean',fields:{BOOL:b}}});
  const toolboxBlock=(type,inputs={})=>({kind:'block',type,inputs});
  class PythonCodeField extends Blockly.FieldLabelSerializable {
    getText() {
      const lines=String(this.getValue()||'').split('\n');
      return `${lines[0].slice(0,48)||'空代码'} · ${lines.length} 行`;
    }
  }
  Blockly.Blocks.studio_python={init() {
    const edit=()=>api.editPython?.(this);
    const icon='data:image/svg+xml,'+encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20"><text x="1" y="17" fill="white" font-size="19">✎</text></svg>');
    this.appendDummyInput().appendField('Python 计算').appendField(new Blockly.FieldImage(icon,20,20,'编辑 Python 代码',edit));
    this.appendDummyInput().appendField(new PythonCodeField('result = [round(q, 4) for q in joints]\nprint(result)'),'CODE');
    this.appendDummyInput().appendField('结果保存到变量').appendField(new Blockly.FieldTextInput('Python结果'),'RESULT');
    this.setPreviousStatement(true);this.setNextStatement(true);this.setColour('#576e96');
    this.setTooltip('点击铅笔或上方 Python 按钮编辑。读取 variables、joints，给 result 赋值；支持受限 Python 计算语法。');
    this.customContextMenu=items=>items.push({text:'编辑 Python 代码',enabled:true,callback:edit});
  }};
  const theme = Blockly.Theme.defineTheme('zekeeparm', {
    base:Blockly.Themes.Classic,
    componentStyles:{workspaceBackgroundColour:'#fcfcfa',toolboxBackgroundColour:'#f3f4ef',toolboxForegroundColour:'#26332f',flyoutBackgroundColour:'#f0f1ea',flyoutForegroundColour:'#26332f',flyoutOpacity:1,scrollbarColour:'#b7beb4',insertionMarkerColour:'#a94e21',insertionMarkerOpacity:.35},
    fontStyle:{family:'Microsoft YaHei, PingFang SC, sans-serif',size:13,weight:'normal'},
    startHats:true
  });
  // Keep the block library readable when the program is zoomed to fit.
  class LibraryFlyout extends Blockly.VerticalFlyout {
    getFlyoutScale() { return window.matchMedia('(max-width: 760px)').matches ? .85 : 1; }
  }
  // Give large value blocks more room to snap into an input.
  Blockly.config.snapRadius=Blockly.config.connectingSnapRadius=56;
  const workspace=Blockly.inject('blockly', {
    plugins:{[Blockly.registry.Type.FLYOUTS_VERTICAL_TOOLBOX]:LibraryFlyout},
    toolbox:{kind:'categoryToolbox',contents:[
      {kind:'category',name:'系统与模式',colour:'#53684d',contents:['rebot_controller_lifecycle','rebot_disable','rebot_safe_home','rebot_set_traj_profile'].map(type=>({kind:'block',type}))},
      {kind:'category',name:'关节与示教',colour:'#a94e21',contents:['studio_move','studio_point','rebot_move_single_joint_delta','rebot_save_current_teach_point','studio_move_value'].map(type=>({kind:'block',type}))},
      {kind:'category',name:'运动学计算',colour:'#576e96',contents:['rebot_compute_fk','rebot_compute_ik','rebot_is_pose_reachable','rebot_check_traj_stats','rebot_model_info','studio_current_joints','studio_joint_values','studio_show'].map(type=>({kind:'block',type}))},
      {kind:'category',name:'变量',colour:'#8c5a78',contents:[toolboxBlock('studio_set',{VALUE:numberShadow(0)}),toolboxBlock('studio_variable'),toolboxBlock('studio_change',{VALUE:numberShadow()})]},
      {kind:'category',name:'数学',colour:'#576e96',contents:[toolboxBlock('studio_number'),toolboxBlock('studio_arithmetic',{A:numberShadow(),B:numberShadow()}),toolboxBlock('studio_unary',{VALUE:numberShadow()})]},
      {kind:'category',name:'逻辑',colour:logicColour,contents:[toolboxBlock('studio_boolean'),toolboxBlock('studio_compare',{A:numberShadow(),B:numberShadow()}),toolboxBlock('studio_logic',{A:boolShadow(),B:boolShadow()}),toolboxBlock('studio_not',{VALUE:boolShadow('false')})]},
      {kind:'category',name:'数据',colour:dataColour,contents:[toolboxBlock('studio_text'),toolboxBlock('studio_list',{ITEM0:numberShadow(0),ITEM1:numberShadow(0),ITEM2:numberShadow(0)}),toolboxBlock('studio_list_get',{INDEX:numberShadow()}),toolboxBlock('studio_list_set',{INDEX:numberShadow(),VALUE:numberShadow(0)}),toolboxBlock('studio_length'),toolboxBlock('studio_dict_get')]},
      {kind:'category',name:'Python',colour:'#576e96',contents:[toolboxBlock('studio_python')]},
      {kind:'category',name:'函数',colour:'#8c5a78',contents:[toolboxBlock('studio_function'),toolboxBlock('studio_call')]},
      {kind:'category',name:'夹爪',colour:'#397c78',contents:[{kind:'block',type:'studio_gripper'}]},
      {kind:'category',name:'流程',colour:'#896633',contents:[{kind:'block',type:'studio_wait'},{kind:'block',type:'studio_repeat'},{kind:'block',type:'studio_if'}]}
    ]}, theme, renderer:'zelos', media:'/vendor/blockly/media/',
    grid:{spacing:24,length:1,colour:'#d4d9cf',snap:true},
    zoom:{controls:true,wheel:true,startScale:1,maxScale:1.6,minScale:.4},
    move:{scrollbars:true,drag:true,wheel:false},trashcan:true
  });
  api.workspace=workspace;
  new ResizeObserver(()=>Blockly.svgResize(workspace)).observe(document.getElementById('blockly'));
  function expression(block) {
    try {return expressionValue(block);}
    catch(error){if(!error.blockId)error.blockId=block?.id;throw error;}
  }
  function expressionValue(block) {
    if(!block || !block.isEnabled()) throw Error('请连接值积木');
    const get=n=>Number(block.getFieldValue(n));
    const pose=()=>['X','Y','Z','R','P','YAW'].map((n,i)=>get(n)*(i<3?.001:rad));
    const current={type:'current_joints'};
    const child=n=>expression(block.getInputTargetBlock(n));
    if(block.type==='studio_number')return {type:'literal',value:get('NUMBER')};
    if(block.type==='studio_text')return {type:'literal',value:block.getFieldValue('TEXT')};
    if(block.type==='studio_boolean')return {type:'literal',value:block.getFieldValue('BOOL')==='true'};
    if(block.type==='studio_variable')return {type:'variable',name:block.getFieldValue('NAME').trim()};
    const binary={studio_arithmetic:'arithmetic',studio_compare:'compare',studio_logic:'logic'};
    if(binary[block.type])return {type:binary[block.type],op:block.getFieldValue('OP'),a:child('A'),b:child('B')};
    if(block.type==='studio_unary')return {type:'unary',op:block.getFieldValue('OP'),value:child('VALUE')};
    if(block.type==='studio_not')return {type:'not',value:child('VALUE')};
    if(block.type==='studio_list')return {type:'list',items:Array.from({length:block.itemCount},(_,i)=>child(`ITEM${i}`))};
    if(block.type==='studio_list_get')return {type:'list_get',list:child('LIST'),index:child('INDEX')};
    if(block.type==='studio_length')return {type:'length',value:child('VALUE')};
    if(block.type==='studio_dict_get')return {type:'dict_get',dict:child('DICT'),key:block.getFieldValue('KEY').trim()};
    if(block.type==='studio_current_joints') return current;
    if(block.type==='studio_joint_values') return {type:'joint_values',joints:[1,2,3,4,5,6].map(i=>get(`J${i}`)*rad)};
    if(block.type==='rebot_compute_fk') return {type:'fk',joints:block.getInputTargetBlock('JOINTS')?expression(block.getInputTargetBlock('JOINTS')):current,frame:block.getFieldValue('FRAME').trim(),output:block.getFieldValue('OUTPUT')};
    if(['rebot_compute_ik','rebot_is_pose_reachable'].includes(block.type)) return {type:block.type==='rebot_compute_ik'?'ik':'reachable',pose:pose(),max_retries:get('RETRIES')};
    if(block.type==='rebot_model_info') return {type:'model_info'};
    if(block.type==='rebot_check_traj_stats') return {type:'traj_stats',target:block.getInputTargetBlock('TARGET')?expression(block.getInputTargetBlock('TARGET')):current,duration:get('DURATION'),profile:block.getFieldValue('PROFILE'),dt:get('DT'),accel_ratio:get('ACCEL')};
    throw Error('不支持的值积木');
  }
  function sequence(block) {
    const steps=[];
    while(block) {
      if(!block.isEnabled()) {block=block.getNextBlock();continue;}
      const id=block.id;
      try {
      const get=n=>Number(block.getFieldValue(n));
      if(block.type==='studio_move') steps.push({id,type:'move',joints:[1,2,3,4,5,6].map(i=>get(`J${i}`)*rad),duration:get('DURATION')});
      else if(block.type==='studio_point') steps.push({id,type:'point',name:block.getFieldValue('POINT'),duration:get('DURATION')});
      else if(block.type==='studio_gripper') steps.push({id,type:'gripper',opening:get('OPENING')});
      else if(block.type==='studio_wait') steps.push({id,type:'wait',seconds:get('SECONDS')});
      else if(block.type==='studio_repeat') steps.push({id,type:'repeat',count:get('COUNT'),body:sequence(block.getInputTargetBlock('BODY'))});
      else if(block.type==='rebot_controller_lifecycle') steps.push({id,type:'lifecycle',action:block.getFieldValue('ACTION'),mode:block.getFieldValue('MODE')});
      else if(block.type==='rebot_disable') steps.push({id,type:'disable'});
      else if(block.type==='rebot_safe_home') steps.push({id,type:'home',max_vel:get('MAX_VEL')*rad,settle_thresh:get('SETTLE')*rad,timeout:get('TIMEOUT')});
      else if(block.type==='rebot_set_traj_profile') steps.push({id,type:'profile',profile:block.getFieldValue('PROFILE'),dt:get('DT'),accel_ratio:get('ACCEL')});
      else if(block.type==='rebot_move_single_joint_delta') steps.push({id,type:'delta',joint:get('JOINT'),delta:get('DELTA')*rad,duration:get('DURATION')});
      else if(block.type==='rebot_save_current_teach_point') steps.push({id,type:'save_point',name:block.getFieldValue('NAME').trim()});
      else if(['studio_show','studio_move_value'].includes(block.type)) steps.push({id,type:block.type==='studio_show'?'show':'move_value',value:expression(block.getInputTargetBlock('VALUE')),...(block.type==='studio_move_value'?{duration:get('DURATION')}:{})});
      else if(block.type==='studio_if') steps.push({id,type:'if',condition:expression(block.getInputTargetBlock('CONDITION')),then:sequence(block.getInputTargetBlock('THEN')),else:sequence(block.getInputTargetBlock('ELSE'))});
      else if(['studio_set','studio_change','studio_list_set'].includes(block.type))steps.push({id,type:{studio_set:'set',studio_change:'change',studio_list_set:'list_set'}[block.type],name:block.getFieldValue('NAME').trim(),value:expression(block.getInputTargetBlock('VALUE')),...(block.type==='studio_list_set'?{index:expression(block.getInputTargetBlock('INDEX'))}:{})});
      else if(block.type==='studio_call')steps.push({id,type:'call',name:block.getFieldValue('NAME').trim()});
      else if(block.type==='studio_python')steps.push({id,type:'python',name:block.getFieldValue('RESULT').trim(),code:block.getFieldValue('CODE')});
      else throw Error('程序包含不支持的积木');
      }catch(error){if(!error.blockId)error.blockId=id;throw error;}
      block=block.getNextBlock();
    }
    return steps;
  }
  api.compile=()=> {
    const top=workspace.getTopBlocks(true).filter(b=>b.isEnabled());
    const starts=top.filter(b=>b.type==='studio_start'),functions=top.filter(b=>b.type==='studio_function');
    if(starts.length!==1 || top.length!==functions.length+1) throw Object.assign(Error('动作须连接到唯一的「当点击运行」下方，函数定义独立放置'),{blockId:top.find(b=>!['studio_start','studio_function'].includes(b.type))?.id||starts[1]?.id});
    return [...functions.map(b=>({id:b.id,type:'function',name:b.getFieldValue('NAME').trim(),body:sequence(b.getInputTargetBlock('BODY'))})),...sequence(starts[0].getNextBlock())];
  };
  api.append=(type,fields={})=> {
    const start=workspace.getTopBlocks(false).find(b=>b.type==='studio_start');
    if(!start) throw Error('请先新建程序');
    let tail=start; while(tail.getNextBlock()) tail=tail.getNextBlock();
    const block=workspace.newBlock(type);
    for(const [key,value] of Object.entries(fields)) block.setFieldValue(String(value),key);
    block.initSvg();block.render();tail.nextConnection.connect(block.previousConnection);
    workspace.centerOnBlock(block.id);
    return block;
  };
  api.load=state=> {
    const blocks=state?.blocks?.blocks;
    if(!Array.isArray(blocks) || JSON.stringify(state).length>1024*1024) throw Error('积木文件格式或大小无效');
    state=JSON.parse(JSON.stringify(state));
    const allowed=new Set([...definitions.map(d=>d.type),'studio_point','studio_list','studio_python']);
    let count=0;
    function inspect(value,depth=0) {
      if(depth>100 || ++count>30000) throw Error('积木文件过于复杂');
      if(value && typeof value==='object') {
        if(value.type==='rebot_controller_lifecycle'&&value.fields)delete value.fields.GRAVITY;
        if(value.type && !allowed.has(value.type)) throw Error(`不支持的积木：${value.type}`);
        Object.values(value).forEach(v=>inspect(v,depth+1));
      }
    }
    inspect(state);Blockly.serialization.workspaces.load(state,workspace);
    requestAnimationFrame(()=>workspace.zoomToFit());
  };
  api.newProgram=()=>api.load({blocks:{languageVersion:0,blocks:[{type:'studio_start',x:60,y:50,deletable:false}]}});
  api.example=()=> {
    api.newProgram();
    const loop=api.append('studio_repeat',{COUNT:2});
    const make=(type,fields)=>{const b=workspace.newBlock(type);Object.entries(fields).forEach(([k,v])=>b.setFieldValue(String(v),k));b.initSvg();b.render();return b;};
    const blocks=[make('studio_move',{J1:-12,J2:40,J3:47,J4:-43,J5:0,J6:0,DURATION:2}),make('studio_gripper',{OPENING:40}),make('studio_wait',{SECONDS:.5}),make('studio_move',{J1:12,J2:40,J3:47,J4:-43,J5:0,J6:0,DURATION:2}),make('studio_gripper',{OPENING:10})];
    loop.getInput('BODY').connection.connect(blocks[0].previousConnection);
    blocks.slice(1).forEach((b,i)=>blocks[i].nextConnection.connect(b.previousConnection));
    workspace.zoomToFit();
  };
  window.StudioBlocks=api;
})();
