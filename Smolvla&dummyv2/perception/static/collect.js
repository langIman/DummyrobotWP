const $ = id => document.getElementById(id);
const movementKeys = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown'];
const turnKeys = ['8', '2', '4', '6'];
const gripperKeys = ['0', '.'];
const pressed = new Map(); // Physical codes keep numpad and main keyboard releases independent.
const input = {
  yaw:0, pitch:0, keys:new Set(), recording:false, recordPending:false, controlPending:false,
  controlArmed:false, lastRecorded:null, lastRecordedAt:0, gripperHeld:null,
  gripperDirection:null, gripperPending:false, holdTimer:null, readyTimer:null,
  sessionId:null, remoteArmed:false, controlDirty:false, stopMode:'immediate',
  gripperTask:null, gripperStopRequested:false, gripperBlocked:false,
  enablingCombined:false, enableGeneration:0
};

async function api(path, options = {}) {
  const response = await fetch(path, {cache:'no-store', ...options});
  let data = {};
  try { data = await response.json(); } catch (_) {}
  if (!response.ok) { const error = new Error(data.error?.message || data.error?.code || `HTTP ${response.status}`); error.data = data; throw error; }
  return data;
}

function jsonOptions(method, body = {}) {
  return {method, headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)};
}

function message(text, error = false) {
  $('collect-message').textContent = text;
  $('collect-message').classList.toggle('error', error);
}

function setView(id, meta, stateId, channel = 'rgb') {
  const part = meta[channel] || meta;
  const available = Boolean(part.frame_available && part.frame_age_ms < 2000);
  $(stateId).dataset.state = available ? 'streaming' : part.frame_available ? 'stale' : (meta.state === 'streaming' ? 'connecting' : meta.state || 'connecting');
  const dimensions = part.dimensions;
  $(id).textContent = dimensions ? `${dimensions[0]} × ${dimensions[1]} · ${Number(meta.measured_fps || 0).toFixed(1)} FPS` : (meta.reason || '等待图像');
}

function renderStatus(data) {
  $('collect-service').className = 'service-state ok';
  $('collect-service').innerHTML = '<i></i>服务正常';
  setView('collect-wrist-meta', data.cameras.wrist, 'collect-view-wrist');
  setView('collect-oak-rgb-meta', data.cameras.oak, 'collect-view-oak-rgb');
  setView('collect-oak-depth-meta', data.cameras.oak, 'collect-view-oak-depth', 'depth');
  const recording = data.recording || {};
  input.recording = Boolean(recording.active);
  const incomplete = recording.last?.status === 'incomplete';
  $('collect-record-state').textContent = recording.active ? '录制中' : recording.status === 'stopping' ? '正在收尾' : incomplete ? '上段未完整保存' : '空闲';
  $('collect-record-state').style.color = recording.active ? 'var(--red)' : incomplete ? 'var(--amber)' : '';
  const diskText = data.disk ? ` · 磁盘剩余 ${Number(data.disk.free_gb).toFixed(1)} GB` : '';
  $('collect-record-detail').textContent = (recording.active ? `${Math.floor((recording.duration_ms || 0) / 1000)} 秒 · 腕部 ${recording.counts?.wrist || 0} 帧 / 上方 ${recording.counts?.oak || 0} 帧` : (recording.last ? `最近：${recording.last.session_id}${incomplete ? ' · ' + (recording.last.reason || '保存异常') : ''}` : '')) + diskText;
  $('collect-record-start').disabled = recording.active;
  $('collect-record-stop').disabled = !recording.active;
  $('collect-label').disabled = recording.active;

  const control = data.control || {};
  const diagnostics = control.diagnostics || {};
  const servoLabels = {1:'接近奇异位形，减速', 2:'奇异位形，停止', 3:'远离奇异位形，减速', 4:'接近碰撞，减速', 5:'碰撞风险，停止', 6:'关节限位，停止'};
  const limitText = (diagnostics.joint_limit_axes || []).map(axis => `${axis.joint} ${axis.bound === 'lower' ? '下限' : '上限'} ${axis.limit_deg}°`).join('、');
  $('control-diagnostics').textContent = control.armed ? (limitText ? `${limitText}；请松键后向范围内操作。` : servoLabels[diagnostics.servo_status?.code] || '') : '';
  const robot = data.robot || {};
  const joints = robot.state?.positions;
  const age = robot.state?.sampled_at_ms ? Math.max(0, Date.now()-robot.state.sampled_at_ms) : null;
  $('collect-joints').innerHTML = Array.from({length:6}, (_, i) => `<div><span>J${i+1}</span><strong>${age !== null && age <= 1500 && Number.isFinite(joints?.[i]) ? joints[i].toFixed(2) + '°' : '—'}</strong></div>`).join('');
  $('collect-camera-warning').textContent = data.cameras.oak.selected_device?.usb3 === false ? '上方相机当前为 USB2，图像帧率可能低于设定值。' : '';
  const motion = robot.motion || {};
  const motionRunning = motion.status === 'running';
  const gripper = robot.gripper || {};
  const gripperReady = Boolean(gripper.control_ready);
  input.remoteArmed = Boolean(control.armed);
  input.controlArmed = Boolean(control.armed && input.sessionId);
  if (!control.armed) input.sessionId = null;
  const controlPaused = Boolean(input.controlArmed && control.paused_reason);
  const pauseLabels = {motion_no_progress:'目标存在但关节反馈连续 3 秒没有明显进展，已保持当前位置', browser_input_idle:'输入心跳超时', moveit_trajectory_stale:'MoveIt 轨迹已过期', joint_feedback_stale:'关节反馈已过期', joint_feedback_unavailable:'关节反馈不可用'};
  const pauseText = (String(control.paused_reason || '').startsWith('moveit_servo_2') ? '当前姿态不能执行这个方向的运动' : pauseLabels[control.paused_reason] || control.paused_reason) + (control.release_required ? '。松开按键后再操作' : '');
  const moveitLabels = {ready:'正常', checking:'检查中', unavailable:'未启动', incompatible:'版本不匹配'};
  $('moveit-state').textContent = moveitLabels[control.backend_status] || control.backend_status || '未知';
  $('arm-state').textContent = input.controlArmed ? (controlPaused ? '实机控制 · 已暂停' : '实机控制中') : robot.enabled_latch === 'confirmed' ? '已使能' : robot.enabled_latch === 'disabled' ? '已失能' : '状态未知';
  $('gripper-state').textContent = gripper.jog_direction === 'close' ? '正在闭合' : gripper.jog_direction === 'open' ? '正在张开' : gripperReady ? '已停止 · 可继续' : gripper.enabled_latch === 'confirmed' ? '已使能' : gripper.enabled_latch === 'disabled' ? '已失能' : '状态未知';
  $('collect-mode').textContent = input.controlArmed ? '实机控制' : '记录模式';
  let stopReason = friendlyStop(control.last_stop_reason);
  if (stopReason && control.diagnostics?.emergency_acknowledged === false) {
    stopReason += ' 停止／失能未获完整确认，硬件状态未知，请使用实体急停。';
  }
  $('collect-safety').textContent = input.remoteArmed && !input.controlArmed ? '另一个页面正在控制；本页只显示状态，可点击退出或急停。' : controlPaused ? `已暂停：${pauseText}` : input.controlArmed ? '键盘控制中 · Esc 停止输入' : stopReason ? `实机控制已退出：${stopReason}` : motionRunning ? '正在前往准备位' : control.backend_status === 'ready' ? '使能后，长按进入实机控制' : '控制服务未就绪';
  $('collect-safety').className = `collect-safety${controlPaused ? ' paused' : input.controlArmed ? ' live' : control.backend_status === 'incompatible' ? ' error' : ''}`;
  $('collect-arm-enable').disabled = input.enablingCombined || motionRunning || input.remoteArmed || gripper.busy || Boolean(gripper.jog_direction) || control.backend_status !== 'ready' || robot.bridge_status !== 'ready' || (robot.enabled_latch === 'confirmed' && gripperReady);
  $('collect-arm-hold').disabled = input.enablingCombined || motionRunning || input.remoteArmed || control.backend_status !== 'ready' || robot.enabled_latch !== 'confirmed';
  $('collect-ready-hold').disabled = input.enablingCombined || motionRunning || input.remoteArmed || robot.bridge_status !== 'ready' || robot.enabled_latch !== 'confirmed' || Boolean(gripper.jog_direction);
  $('collect-arm-disarm').disabled = !input.remoteArmed;
  $('collect-gripper-enable').disabled = Boolean(input.enablingCombined || (input.remoteArmed && !input.controlArmed) || robot.bridge_status !== 'ready' || gripper.busy || gripperReady || gripper.enabled_latch === 'confirmed');
  $('collect-gripper-stop').disabled = Boolean(robot.bridge_status !== 'ready' || gripper.busy || !gripper.jog_direction);
  $('gripper-key-state').textContent = input.gripperDirection === 'close' ? '0 按住 · 正在闭合' : input.gripperDirection === 'open' ? '. 按住 · 正在张开' : '0 闭合 · . 张开';
  $('gripper-key-state').classList.toggle('active', Boolean(input.gripperDirection));
}

function friendlyStop(reason) {
  if (!reason) return '';
  if (reason.includes('controller_firmware_exception') || reason.includes('terminate called after throwing an instance of')) return '主控报告固件异常，已停止发送运动目标；这不代表硬件已失能。';
  if (reason.includes('hold_feedback_unavailable')) return '停止输入时无法读到有效关节反馈，位置保持目标未发送；已退出控制并尝试急停。';
  if (reason.includes('stream_target_step_too_large')) return '目标与反馈差超过 8°，已停止。请检查关节反馈和机械臂实际运动。';
  if (reason.includes('stream_motion_not_accepted')) return '主控未确认运动命令，已停止发送新目标；原始回复保存在日志。';
  if (reason.includes('feedback')) return '关节反馈异常，已停止。';
  return ({operator:'已主动退出', emergency_stop:'已急停', page_hidden:'控制页面已关闭', service_shutdown:'服务已重启'})[reason] || reason;
}

function updateLook() {
  const keys = new Set(pressed.values());
  input.keys = new Set(movementKeys.filter(key => keys.has(key)));
  input.yaw = Number(keys.has('6')) - Number(keys.has('4'));
  input.pitch = Number(keys.has('8')) - Number(keys.has('2'));
  const length = Math.max(1, Math.hypot(input.yaw, input.pitch));
  input.yaw /= length; input.pitch /= length;
  for (const key of [...movementKeys, ...turnKeys, ...gripperKeys]) document.querySelector(`.key-state[data-key="${key}"]`).classList.toggle('active', keys.has(key));
}

function inputPayload() {
  return {yaw:input.yaw, pitch:input.pitch, keys:[...input.keys].sort(), at_client_ms:Date.now()};
}

async function sendInput(force = false) {
  const payload = inputPayload();
  const signature = JSON.stringify({yaw:payload.yaw, pitch:payload.pitch, keys:payload.keys});
  const active = payload.keys.length || payload.yaw || payload.pitch;
  if (input.recording && (!input.remoteArmed || input.controlArmed) && !input.recordPending && (force || signature !== input.lastRecorded || active || Date.now() - input.lastRecordedAt >= 500)) {
    input.recordPending = true;
    (async () => { try {
      await api('/api/recordings/control-event', jsonOptions('POST', {kind:payload.keys.length ? 'move' : 'look', ...payload}));
      input.lastRecorded = signature; input.lastRecordedAt = Date.now();
    } catch (error) {
      if (error.data?.error?.code === 'recording_not_active') input.recording = false;
      message(error.message, true);
    } finally { input.recordPending = false; } })();
  }
  if (input.controlArmed && input.controlPending) { input.controlDirty = true; return; }
  if (input.controlArmed && input.sessionId && !input.controlPending) {
    input.controlDirty = false;
    input.controlPending = true;
    try { await api('/api/control/input', jsonOptions('POST', {...inputPayload(), session_id:input.sessionId, stop_mode:input.stopMode})); }
    catch (error) {
      if (['live_control_not_armed','moveit_command_failed','control_session_mismatch'].includes(error.data?.error?.code)) { input.controlArmed = false; input.sessionId = null; }
      message(error.message, true);
    } finally { input.controlPending = false; if (input.controlDirty && input.controlArmed) sendInput(); }
  }
}

function resetMotionInput() {
  input.stopMode = 'immediate';
  pressed.clear(); updateLook(); sendInput(true);
}

function isTextEntry(target) { return target && target.matches?.('input,textarea,select,[contenteditable="true"]'); }

async function toggleRecording(start) {
  try {
    if (start) await api('/api/recordings/start', jsonOptions('POST', {label:$('collect-label').value}));
    else await api('/api/recordings/stop', jsonOptions('POST', {}));
    message(start ? '录制已开始，控制输入将写入片段事件日志。' : '录制已结束。');
    renderStatus(await api('/api/status'));
  } catch (error) { message(error.message, true); }
}

async function robotAction(action, body = {}) {
  const result = await api(`/api/robot/actions/${action}`, jsonOptions('POST', body));
  renderStatus(await api('/api/status'));
  return result;
}

async function enableArm() {
  if (input.enablingCombined) return;
  input.enablingCombined = true;
  const generation = ++input.enableGeneration;
  $('collect-arm-enable').disabled = true;
  $('collect-arm-hold').disabled = true;
  $('collect-ready-hold').disabled = true;
  $('collect-gripper-enable').disabled = true;
  try {
    const status = await api('/api/status');
    if (generation !== input.enableGeneration) return;
    if (status.control?.armed) throw new Error('请先退出已有实机控制。');
    let robot = status.robot || {};
    if (robot.enabled_latch !== 'confirmed') robot = await robotAction('enable');
    if (generation !== input.enableGeneration) return;
    if (!robot.gripper?.control_ready) await robotAction('gripper-enable');
    if (generation !== input.enableGeneration) return;
    input.gripperBlocked = false;
    message('六轴和夹爪已使能；可前往准备位，再长按进入实机控制。');
  } catch (error) { if (generation === input.enableGeneration) message(`使能未全部完成：${error.message} 请查看六轴和夹爪状态。`, true); }
  finally {
    input.enablingCombined = false;
    try { renderStatus(await api('/api/status')); } catch (_) {}
  }
}

function cancelArmHold() {
  clearTimeout(input.holdTimer); input.holdTimer = null; $('collect-arm-hold').classList.remove('holding');
}

function cancelReadyHold() {
  clearTimeout(input.readyTimer); input.readyTimer = null;
  $('collect-ready-hold').classList.remove('holding');
}

function beginReadyHold(event) {
  if ($('collect-ready-hold').disabled || input.readyTimer) return;
  event.preventDefault();
  $('collect-ready-hold').classList.add('holding');
  input.readyTimer = setTimeout(async () => {
    cancelReadyHold();
    if ($('collect-ready-hold').disabled) return;
    $('collect-ready-hold').disabled = true;
    try {
      const result = await robotAction('ready', {confirmation:'MOVE_READY'});
      message(`正在以速度 ${result?.motion?.speed ?? 12} 前往控制准备位，请等待到位。需要停止时点击急停。`);
    } catch (error) { message(error.message, true); }
  }, 1200);
}

function beginArmHold(event) {
  if ($('collect-arm-hold').disabled || input.holdTimer) return;
  event.preventDefault(); $('collect-arm-hold').classList.add('holding');
  input.holdTimer = setTimeout(async () => {
    input.holdTimer = null; $('collect-arm-hold').classList.remove('holding');
    try {
      const armed = await api('/api/control/arm', jsonOptions('POST', {confirmation:'ENABLE_LIVE_CONTROL'}));
      input.sessionId = armed.session_id;
      input.controlArmed = true;
      resetMotionInput();
      input.gripperBlocked = false;
      $('control-keyboard').focus({preventScroll:true});
      message('实机控制已启用，按下的控制键会亮起。');
      renderStatus(await api('/api/status'));
    } catch (error) { message(error.message, true); }
  }, 1200);
}

async function disarmControl() {
  input.enableGeneration += 1;
  input.gripperBlocked = true;
  input.controlArmed = false; input.sessionId = null;
  resetMotionInput();
  stopGripper(); // Unified server-side disarm must not wait for a browser status request.
  try {
    const result = await api('/api/control/disarm', jsonOptions('POST', {reason:'operator'}));
    message(result.overall_acknowledged === false ? '已退出控制，但六轴或夹爪失能未获确认，请检查状态或使用急停。' : '已退出实机控制，六轴与夹爪一起失能。', result.overall_acknowledged === false);
    renderStatus(await api('/api/status'));
  } catch (error) { message(error.message, true); }
}

async function enableGripper() {
  $('collect-gripper-enable').disabled = true;
  try { await robotAction('gripper-enable'); input.gripperBlocked = false; message('夹爪控制已就绪；按住 0 闭合或 . 张开。'); }
  catch (error) { message(error.message, true); }
}

function stopGripper(force = false) {
  for (const [code, key] of pressed) if (gripperKeys.includes(key)) pressed.delete(code);
  input.gripperHeld = null;
  input.gripperStopRequested ||= force;
  updateLook();
  return reconcileGripper();
}

function syncGripperKeys() {
  const keys = new Set(pressed.values());
  const close = keys.has('0'), open = keys.has('.');
  const desired = close === open ? null : close ? 'close' : 'open';
  if (desired && (input.enablingCombined || input.gripperBlocked || (input.remoteArmed && !input.controlArmed))) {
    message('请先在本页进入实机控制或使能夹爪。', true);
    return stopGripper();
  }
  input.gripperHeld = desired;
  return reconcileGripper();
}

function reconcileGripper() {
  if (input.gripperPending) return input.gripperTask;
  input.gripperPending = true;
  input.gripperTask = (async () => {
    try {
      // Serialize start/stop, always rechecking the latest keys after each reply.
      // A quick release or reversal must not be lost while start is in flight.
      while (input.gripperStopRequested || input.gripperDirection !== input.gripperHeld) {
        if (input.gripperStopRequested || input.gripperDirection) {
          input.gripperStopRequested = false;
          await robotAction('gripper-jog-stop');
          input.gripperDirection = null;
        } else {
          const direction = input.gripperHeld;
          await robotAction('gripper-jog-start', {direction});
          input.gripperDirection = direction;
        }
      }
    } catch (error) {
      input.gripperHeld = null;
      for (const [code, key] of pressed) if (gripperKeys.includes(key)) pressed.delete(code);
      try { await robotAction('gripper-jog-stop'); } catch (_) {}
      input.gripperDirection = null;
      input.gripperStopRequested = false;
      message(error.message, true);
    } finally { input.gripperPending = false; updateLook(); }
  })();
  return input.gripperTask;
}

async function gripperHeartbeat() {
  if (!input.gripperDirection || input.gripperPending || input.gripperHeld !== input.gripperDirection) return;
  try { await api('/api/robot/actions/gripper-jog-heartbeat', jsonOptions('POST', {direction:input.gripperDirection})); }
  catch (error) { await stopGripper(true); message(error.message, true); }
}

async function emergencyStop() {
  input.enableGeneration += 1;
  cancelReadyHold(); cancelArmHold();
  input.gripperBlocked = true;
  $('collect-emergency').disabled = true; resetMotionInput(); input.gripperHeld = null;
  try { await api('/api/robot/emergency-stop', jsonOptions('POST', {})); input.controlArmed = false; input.gripperDirection = null; message('急停完成，请确认机械臂和夹爪均已失能。'); }
  catch (error) { message(error.message, true); }
  finally { $('collect-emergency').disabled = false; }
}

window.addEventListener('keydown', event => {
  if (event.key === 'Escape') { event.preventDefault(); resetMotionInput(); stopGripper(); return; }
  if (isTextEntry(event.target) || event.ctrlKey || event.altKey || event.metaKey) return;
  const key = controlKey(event);
  if (![...movementKeys, ...turnKeys, ...gripperKeys].includes(key)) return;
  event.preventDefault();
  const code = event.code || event.key;
  if (event.repeat || pressed.has(code)) return;
  pressed.set(code, key); input.stopMode='release'; updateLook();
  if (gripperKeys.includes(key)) syncGripperKeys();
  else sendInput(true);
});
window.addEventListener('keyup', event => {
  const code = event.code || event.key;
  const key = pressed.get(code);
  if (!key) return;
  event.preventDefault(); pressed.delete(code); input.stopMode='release'; updateLook();
  if (gripperKeys.includes(key)) syncGripperKeys();
  else sendInput(true);
});
function controlKey(event) {
  return ({KeyW:'w', KeyA:'a', KeyS:'s', KeyD:'d', ArrowUp:'arrowup', ArrowDown:'arrowdown',
    Numpad8:'8', Numpad2:'2', Numpad4:'4', Numpad6:'6', Numpad0:'0', NumpadDecimal:'.',
    Digit8:'8', Digit2:'2', Digit4:'4', Digit6:'6', Digit0:'0', Period:'.'})[event.code] || (event.key || '').toLowerCase();
}
window.addEventListener('blur', () => { cancelReadyHold(); cancelArmHold(); resetMotionInput(); stopGripper(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { cancelReadyHold(); cancelArmHold(); resetMotionInput(); stopGripper(); } });
document.addEventListener('focusin', event => { if (isTextEntry(event.target)) { resetMotionInput(); stopGripper(); } });
$('collect-arm-enable').onclick = enableArm;
$('collect-ready-hold').addEventListener('pointerdown', beginReadyHold);
for (const event of ['pointerup','pointerleave','pointercancel']) $('collect-ready-hold').addEventListener(event, cancelReadyHold);
$('collect-arm-hold').addEventListener('pointerdown', beginArmHold);
$('collect-arm-hold').addEventListener('pointerup', cancelArmHold);
$('collect-arm-hold').addEventListener('pointerleave', cancelArmHold);
$('collect-arm-hold').addEventListener('pointercancel', cancelArmHold);
$('collect-arm-disarm').onclick = disarmControl;
$('collect-gripper-enable').onclick = enableGripper;
$('collect-gripper-stop').onclick = () => stopGripper(true);
$('collect-record-start').onclick = () => toggleRecording(true);
$('collect-record-stop').onclick = () => toggleRecording(false);
$('collect-emergency').onclick = emergencyStop;
setInterval(() => { $('collect-clock').textContent = new Date().toLocaleTimeString('zh-CN',{hour12:false}); sendInput(); }, 25);
setInterval(gripperHeartbeat, 500);
setInterval(async () => { try { renderStatus(await api('/api/status')); } catch (error) { $('collect-service').className = 'service-state error'; $('collect-service').innerHTML = '<i></i>服务断开'; message(error.message, true); } }, 1000);
window.addEventListener('pagehide', () => {
  input.enableGeneration += 1;
  resetMotionInput(); input.gripperHeld = null;
  if (input.recording && (!input.remoteArmed || input.controlArmed)) navigator.sendBeacon('/api/recordings/control-event', new Blob([JSON.stringify({kind:'move',yaw:0,pitch:0,keys:[],at_client_ms:Date.now()})], {type:'application/json'}));
  input.gripperBlocked = true;
  if (input.gripperDirection || input.gripperPending) navigator.sendBeacon('/api/robot/actions/gripper-jog-stop', new Blob(['{}'], {type:'application/json'}));
  if (input.controlArmed) navigator.sendBeacon('/api/control/disarm', new Blob([JSON.stringify({reason:'page_hidden'})], {type:'application/json'}));
});
updateLook();
api('/api/status').then(renderStatus).catch(error => message(error.message, true));
