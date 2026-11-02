import { t, onLangChange } from './i18n.js';

const strategy = { direct: '直线', over: '上方抬高', 'side-negative': 'Y 负向侧绕', 'side-positive': 'Y 正向侧绕', outside: 'X 外侧绕行' };
const reason = { ik: 'IK 不可达或姿态不满足', collision: '自碰撞/场景碰撞', 'obstacle-clearance': '障碍物安全间距不足', 'joint-limits': '超出关节限位' };
const mm = values => values.map(v => (v * 1000).toFixed(1)).join(', ');

export function bindTeachingControls({ motion, view }) {
  const log = document.getElementById('study-log');
  const distance = document.getElementById('study-distance');
  const planStatus = document.getElementById('study-plan-status');
  let hasDetour = false, previousStage = 'idle';
  function append(message, failure = false) {
    const pinned = log.scrollHeight - log.scrollTop - log.clientHeight < 35;
    const li = document.createElement('li');
    li.textContent = `[${(performance.now() / 1000).toFixed(2)}s] ${message}`;
    if (failure) li.className = 'is-error';
    log.append(li);
    while (log.children.length > 160) log.firstElementChild.remove();
    if (pinned) log.scrollTop = log.scrollHeight;
  }
  function clearTrace() { hasDetour = false; view.clearPlanningTrace(); planStatus.textContent = t('study.idle'); }
  for (const [id, setter] of [['frames', view.setTeachingFrames], ['hulls', view.setCollisionHulls],
    ['paths', view.setTrajectoriesVisible], ['proximity', view.setProximityVisible]]) {
    const checkbox = document.getElementById(`study-${id}`);
    const apply = () => {
      const count = setter(checkbox.checked);
      if (typeof count === 'number') checkbox.dataset.count = String(count);
    };
    checkbox.addEventListener('change', apply); apply();
  }
  motion.onDiagnostics(event => {
    switch (event.type) {
      case 'configured':
        clearTrace();
        append(`[场景] 障碍物${event.enabled ? '启用' : '关闭'}；中心 (${mm(event.position)}) mm；尺寸 (${mm(event.size)}) mm。`);
        break;
      case 'target':
        append(`[采样器/${event.stage}] 笛卡尔目标 (${mm(event.point)}) mm；直线采样 ≤${event.spacing * 1000} mm，关节边检测 ≤${event.jointSpacing} rad。`);
        break;
      case 'candidate': append(`[规划器] 尝试 ${strategy[event.strategy]} 候选路径。`); break;
      case 'rejected':
        append(`[碰撞检测/IK] ${strategy[event.strategy]}：第 ${event.sample} 个采样点失败，${reason[event.reason] || event.reason}${event.blocker ? `（${event.blocker}）` : ''}${event.distance !== undefined ? `，距离 ${(event.distance * 1000).toFixed(1)} mm` : ''}。`, true);
        break;
      case 'planned':
        append(`[IK/规划器] ${strategy[event.strategy]} 通过；${event.samples} 个 IK 采样点，规划耗时 ${event.elapsed.toFixed(1)} ms；线性关节采样 + PD 跟踪。`);
        // Keep the most recent rejected/direct comparison when later task stages are unobstructed.
        if (event.direct.length || !hasDetour) {
          view.setPlanningTrace(event); hasDetour = event.direct.length > 0;
          planStatus.textContent = `${event.stage} · ${strategy[event.strategy]} · ${event.samples} 个安全 IK 采样点${event.directFailure ? `；直线失败：${reason[event.directFailure.reason]}` : ''}。`;
        }
        break;
      case 'failed':
        view.setPlanningTrace(event); hasDetour = true;
        planStatus.textContent = `${event.stage}：有限候选路径无解，任务已停止；红线表示被拒目标路线，并非可执行路径。`;
        append(`[规划器] 无安全路径；耗时 ${event.elapsed.toFixed(1)} ms，停止执行。`, true);
        break;
    }
  });
  function updateProximity() {
    const value = motion.proximity(); view.setProximityVisual(value);
    distance.classList.toggle('is-danger', Boolean(value && value.distance < value.clearance));
    distance.textContent = value ? `${value.distance < value.clearance ? '⚠ 安全间距不足' : '最近表面距离'}：${(value.distance * 1000).toFixed(1)} mm · ${value.body} / 障碍物（阈值 ${value.clearance * 1000} mm）`
      : t('study.disabled');
  }
  onLangChange(updateProximity);
  function reset() { log.replaceChildren(); clearTrace(); updateProximity(); }
  document.getElementById('study-clear').addEventListener('click', reset);
  return { reset, clearTrace, updateProximity,
    updateTask(state) {
      if (state.stage === previousStage) return;
      previousStage = state.stage;
      append(`[执行器] 阶段 ${state.stage}${state.message ? `：${state.message}` : ''}。`, state.stage === 'failed');
    }
  };
}
