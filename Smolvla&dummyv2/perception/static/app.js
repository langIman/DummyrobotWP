const $ = id => document.getElementById(id);
const state = { status: null, configs: {}, pendingPreset: null, holdTimer: null, recordingListAt: 0,
  jog: {direction:null, pending:false, stopRequested:false, heartbeatTimer:null, heartbeatBusy:false} };

async function api(path, options = {}) {
  const response = await fetch(path, { cache: 'no-store', ...options });
  let data = {};
  try { data = await response.json(); } catch (_) {}
  if (!response.ok) {
    const error = new Error(data.error?.message || data.error?.code || `HTTP ${response.status}`);
    error.data = data;
    throw error;
  }
  return data;
}

function jsonOptions(method, body = {}) {
  return { method, headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
}

let toastTimer;
function toast(message, error = false) {
  const element = $('toast');
  element.textContent = message;
  element.className = `toast visible${error ? ' error' : ''}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { element.className = 'toast'; }, 3500);
}

function statusText(camera) {
  const labels = { streaming: '正常', connecting: '连接中', unavailable: '不可用', stopped: '已停止', starting: '启动中' };
  return labels[camera?.state] || '未知';
}

function cameraMeta(camera, part) {
  const meta = part || camera;
  const dimensions = meta?.dimensions;
  const fps = Number(camera?.measured_fps || 0).toFixed(1);
  return dimensions ? `${dimensions[0]} × ${dimensions[1]} · ${fps} FPS` : (camera?.reason || '等待图像');
}

function setCameraView(id, camera, metaId, part) {
  const fresh = Boolean((part || camera)?.frame_available);
  $(id).dataset.state = fresh ? 'streaming' : (camera?.state || 'connecting');
  $(metaId).textContent = cameraMeta(camera, part);
}

function formatDuration(milliseconds) {
  const seconds = Math.max(0, Math.floor((milliseconds || 0) / 1000));
  return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

function formatBytes(bytes) {
  if (!Number.isFinite(bytes)) return '--';
  if (bytes >= 1073741824) return `${(bytes / 1073741824).toFixed(2)} GB`;
  return `${(bytes / 1048576).toFixed(1)} MB`;
}

function renderStatus(data) {
  state.status = data;
  $('service-state').className = 'service-state ok';
  $('service-state').innerHTML = '<i></i>服务正常';
  const wrist = data.cameras.wrist;
  const oak = data.cameras.oak;
  $('wrist-summary').textContent = statusText(wrist);
  const oakUsb = oak.selected_device?.usb_speed;
  $('oak-summary').textContent = `${statusText(oak)}${oakUsb && !oak.selected_device.usb3 ? ' · USB2' : ''}`;
  $('disk-summary').textContent = `${Number(data.disk.free_gb).toFixed(1)} GB`;
  $('wrist-config-state').textContent = wrist.reason || statusText(wrist);
  $('oak-config-state').textContent = oak.reason || statusText(oak);
  setCameraView('view-wrist', wrist, 'wrist-meta');
  setCameraView('view-oak-rgb', oak, 'oak-rgb-meta', oak.rgb);
  setCameraView('view-oak-depth', oak, 'oak-depth-meta', oak.depth);
  document.querySelectorAll('.camera-toggle').forEach(button => {
    const camera = button.dataset.camera === 'wrist' ? wrist : oak;
    button.textContent = camera.enabled ? '停止' : '启动';
    button.dataset.action = camera.enabled ? 'stop' : 'start';
  });

  const recording = data.recording;
  $('record-summary').textContent = recording.active ? '录制中' : '空闲';
  $('record-indicator').className = `record-indicator${recording.active ? ' active' : ''}`;
  $('record-indicator').querySelector('span').textContent = recording.active ? recording.session_id : '空闲';
  $('record-toggle').className = `button primary record-button${recording.active ? ' active' : ''}`;
  $('record-toggle').innerHTML = `<span class="record-dot"></span>${recording.active ? '停止录制' : '开始录制'}`;
  $('record-label').disabled = recording.active;
  $('record-duration').textContent = formatDuration(recording.duration_ms);
  $('record-wrist-count').textContent = recording.counts?.wrist || 0;
  $('record-oak-count').textContent = recording.counts?.oak || 0;
  $('record-robot-count').textContent = recording.counts?.robot || 0;
  const dropped = (recording.dropped?.wrist || 0) + (recording.dropped?.oak || 0);
  $('record-message').textContent = recording.active ? `剩余 ${formatDuration(recording.remaining_ms)} · 写入队列丢帧 ${dropped}` : (recording.last ? `最近结束：${recording.last.session_id} · ${recording.last.status}` : '');
  document.querySelectorAll('#apply-wrist,#apply-oak,#discover,.camera-toggle').forEach(button => { button.disabled = recording.active; });

  renderRobot(data.robot);
}

function renderRobot(robot) {
  if (state.status) state.status.robot = robot;
  const stateValue = robot.state || {};
  const bridgeLabels = { ready: '桥接正常', checking: '检查中', unavailable: '桥接不可用', incompatible: '桥接不兼容' };
  $('robot-summary').textContent = bridgeLabels[robot.bridge_status] || robot.bridge_status;
  $('robot-detail').textContent = stateValue.reason || `${bridgeLabels[robot.bridge_status] || robot.bridge_status} · 使能状态 ${robot.enabled_latch}`;
  for (let index = 0; index < 6; index++) {
    $(`joint-${index}`).textContent = stateValue.positions ? `${Number(stateValue.positions[index]).toFixed(2)}°` : '--';
    const stale = !stateValue.sampled_at_ms || Date.now() - stateValue.sampled_at_ms > 2500;
    $(`joint-${index}`).closest('.joint').classList.toggle('stale', stale);
    $(`joint-time-${index}`).textContent = stateValue.positions ? sampleTime(stateValue.sampled_at_ms, stale) : '未取得反馈';
  }
  renderGripperAngle(stateValue.gripper);
  const motion = robot.motion;
  const motionLabels = { running: '执行中', complete: '已完成', failed: '已失败', cancelled: '已取消' };
  $('motion-state').textContent = motion ? `${({home:'回零位', rest:'回休止位', ready:'控制准备位'})[motion.name] || motion.name} · ${motionLabels[motion.status] || motion.status}` : '无动作';
  const armed = robot.enabled_latch === 'confirmed';
  document.querySelectorAll('.button.motion').forEach(button => { button.disabled = !armed || motion?.status === 'running'; });
  $('enable').disabled = robot.bridge_status !== 'ready' || armed || robot.raw_console?.active;
  $('disable').disabled = robot.bridge_status !== 'ready';
  renderGripper(robot.gripper);
  renderRawConsole();
}

async function pollStatus() {
  try {
    renderStatus(await api('/api/status'));
    if (Date.now() - state.recordingListAt > 10000) await loadRecordings();
  } catch (error) {
    $('service-state').className = 'service-state error';
    $('service-state').innerHTML = '<i></i>服务断开';
    for (const id of ['gripper-enable', 'gripper-open', 'gripper-close', 'gripper-current-send', 'gripper-reference', 'gripper-jog-close', 'gripper-jog-open']) $(id).disabled = true;
    $('gripper-state').textContent = '服务断开 · 状态未知';
    state.status = null;
    $('raw-send').disabled = true;
    $('raw-mode').textContent = '服务断开 · 状态未知';
    document.querySelectorAll('.joint').forEach(cell => cell.classList.add('stale'));
    $('joint-time-6').textContent = '服务断开';
  } finally {
    setTimeout(pollStatus, 1000);
  }
}

async function loadConfigs() {
  const [wrist, oak] = await Promise.all([api('/api/cameras/wrist/config'), api('/api/cameras/oak/config')]);
  state.configs = { wrist: wrist.configuration, oak: oak.configuration };
  showConfigs();
}

function showConfigs() {
  const wrist = state.configs.wrist;
  const oak = state.configs.oak;
  if (wrist) {
    $('wrist-device').value = wrist.device_id || '';
    $('wrist-resolution').value = `${wrist.width}x${wrist.height}`;
    $('wrist-fps').value = String(wrist.fps);
    $('wrist-quality').value = wrist.jpeg_quality;
  }
  if (oak) {
    $('oak-device').value = oak.mx_id || '';
    $('oak-resolution').value = `${oak.rgb_width}x${oak.rgb_height}`;
    $('oak-fps').value = String(oak.fps);
    $('oak-quality').value = oak.jpeg_quality;
    $('depth-min').value = oak.depth_min_mm;
    $('depth-max').value = oak.depth_max_mm;
  }
}

function fillOptions(select, devices, valueField, labeler, configured) {
  select.replaceChildren(new Option('自动识别', ''));
  for (const device of devices) select.add(new Option(labeler(device), device[valueField]));
  select.value = configured || '';
}

async function discoverDevices(announce = false) {
  try {
    const devices = await api('/api/cameras/discover');
    fillOptions($('wrist-device'), devices.wrist, 'device_id', d => `${d.name}${d.is_virtual ? ' · 虚拟' : ''}${d.vid ? ` · ${Number(d.vid).toString(16).padStart(4, '0')}:${Number(d.pid).toString(16).padStart(4, '0')}` : ''}`, state.configs.wrist?.device_id);
    fillOptions($('oak-device'), devices.oak, 'mx_id', d => `${d.mx_id} · ${d.state}`, state.configs.oak?.mx_id);
    if (announce) toast(`发现腕部相机 ${devices.wrist.length} 台，OAK ${devices.oak.length} 台`);
  } catch (error) { toast(error.message, true); }
}

async function applyWrist() {
  const [width, height] = $('wrist-resolution').value.split('x').map(Number);
  const body = { device_id: $('wrist-device').value || null, width, height, fps: Number($('wrist-fps').value), jpeg_quality: Number($('wrist-quality').value) };
  await applyCamera('wrist', body, $('apply-wrist'));
}

async function applyOak() {
  const [rgb_width, rgb_height] = $('oak-resolution').value.split('x').map(Number);
  const body = { mx_id: $('oak-device').value || null, rgb_width, rgb_height, fps: Number($('oak-fps').value), jpeg_quality: Number($('oak-quality').value), depth_min_mm: Number($('depth-min').value), depth_max_mm: Number($('depth-max').value) };
  await applyCamera('oak', body, $('apply-oak'));
}

async function applyCamera(camera, body, button) {
  button.disabled = true;
  $('config-message').className = 'inline-status';
  $('config-message').textContent = '正在验证设备输出…';
  try {
    const result = await api(`/api/cameras/${camera}/config`, jsonOptions('PUT', body));
    state.configs[camera] = result.configuration;
    showConfigs();
    $('config-message').textContent = '配置已验证并保存';
    toast('相机配置已应用');
  } catch (error) {
    $('config-message').className = 'inline-status error';
    $('config-message').textContent = error.message;
    toast(error.message, true);
    await loadConfigs();
  } finally { button.disabled = false; }
}

async function toggleRecording() {
  const active = Boolean(state.status?.recording.active);
  const button = $('record-toggle');
  button.disabled = true;
  try {
    if (active) {
      const result = await api('/api/recordings/stop', jsonOptions('POST'));
      toast(`片段已保存：${result.session_id}`);
      await loadRecordings();
    } else {
      const result = await api('/api/recordings/start', jsonOptions('POST', { label: $('record-label').value }));
      toast(`开始录制：${result.session_id}`);
    }
  } catch (error) { toast(error.message, true); }
  finally { button.disabled = false; }
}

async function loadRecordings() {
  try {
    const data = await api('/api/recordings');
    const body = $('recordings');
    body.replaceChildren();
    for (const item of data.recordings) {
      const row = document.createElement('tr');
      const label = item.label ? `${item.label} · ${item.session_id}` : item.session_id;
      const frames = `${item.counts?.wrist || 0} / ${item.counts?.oak || 0}`;
      for (const [value, className] of [[label, ''], [item.status || '--', `status-${item.status}`], [formatDuration(item.duration_ms), ''], [frames, ''], [formatBytes(item.size_bytes), '']]) {
        const cell = document.createElement('td'); cell.textContent = value; cell.className = className; row.append(cell);
      }
      body.append(row);
    }
    if (!data.recordings.length) body.innerHTML = '<tr><td colspan="5">暂无片段</td></tr>';
    state.recordingListAt = Date.now();
  } catch (_) {}
}

async function robotAction(action, body = {}) {
  try {
    await api(`/api/robot/actions/${action}`, jsonOptions('POST', body));
    toast(action === 'enable' ? '机械臂已确认使能' : action === 'disable' ? '机械臂已确认失能' : '预设动作已开始');
  } catch (error) { toast(error.message, true); }
}

async function emergencyStop() {
  const button = $('emergency'); button.disabled = true;
  try {
    const result = await api('/api/robot/emergency-stop', jsonOptions('POST'));
    const okay = result.overall_acknowledged;
    toast(okay ? '夹爪与六轴停止命令均已确认' : '停止命令未全部确认，请检查设备', !okay);
  } catch (error) { toast(error.message, true); }
  finally { button.disabled = false; }
}

function renderGripper(gripper) {
  if (!gripper) return;
  state.gripper = gripper;
  const labels = { unknown: '状态未知', confirmed: '夹爪已使能', disabled: '夹爪已失能' };
  const controlReady = Boolean(gripper.control_ready);
  $('gripper-state').textContent = gripper.faulted ? '停止未确认 · 请重试失能' : controlReady && !gripper.jog_direction ? '夹爪已停止 · 可继续' : (labels[gripper.enabled_latch] || '状态未知');
  $('gripper-countdown').textContent = gripper.jog_direction
    ? `点动${gripper.jog_direction === 'close' ? '闭合' : '张开'} · 看门狗 ${Math.max(0, Math.round(gripper.jog_watchdog_ms || 0))} ms`
    : (gripper.remaining_ms == null ? `开合电流设定 ${gripper.current_a} A` : `约 ${(gripper.remaining_ms / 1000).toFixed(1)} 秒后发送失能`);
  // The unified robot panel owns angle display; action replies remain in history.
  const ready = state.status?.robot.bridge_status === 'ready';
  const busy = Boolean(state.gripperPending || state.jog.pending || gripper.busy || state.status?.robot.raw_console?.active || state.status?.robot.motion?.status === 'running');
  const enabled = gripper.enabled_latch === 'confirmed';
  const current = Number($('gripper-current').value);
  const validCurrent = $('gripper-current').value.trim() !== '' && Number.isFinite(current) && current >= 0.05 && current <= 1.0;
  $('gripper-enable').disabled = !ready || busy || enabled || controlReady || gripper.faulted || !validCurrent || Math.abs(current - gripper.current_a) > 0.00001;
  $('gripper-current-send').disabled = !ready || busy || enabled || controlReady || gripper.faulted || !validCurrent;
  $('gripper-current').disabled = busy || enabled || controlReady;
  $('gripper-disable').disabled = !ready || busy || (!enabled && !controlReady);
  const pulseActive = ['open', 'close'].includes(gripper.last_transaction?.action) && enabled;
  $('gripper-open').disabled = $('gripper-close').disabled = !ready || busy || !enabled || pulseActive || gripper.faulted;
  $('gripper-reference').disabled = !ready || busy || enabled || gripper.faulted;
  const jogDirection = state.jog.direction || gripper.jog_direction;
  $('gripper-jog-close').disabled = !ready || busy && jogDirection !== 'close' || !controlReady || gripper.faulted || jogDirection === 'open';
  $('gripper-jog-open').disabled = !ready || busy && jogDirection !== 'open' || !controlReady || gripper.faulted || jogDirection === 'close';
  $('gripper-jog-close').dataset.holding = jogDirection === 'close' && !state.jog.stopRequested ? 'true' : 'false';
  $('gripper-jog-open').dataset.holding = jogDirection === 'open' && !state.jog.stopRequested ? 'true' : 'false';
  // Stop stays available even during a pending request or a connection fault.
  const history = gripper.history || [];
  if (history.length) {
    $('gripper-replies').textContent = [...history].reverse().map(item => {
      const when = new Date(item.at_ms).toLocaleTimeString('zh-CN', {hour12:false});
      const source = {watchdog:'看门狗停止',command_failure:'失败收尾',jog_hold_failure:'停止失败收尾',operator:'操作',service_shutdown:'服务关闭'}[item.source] || item.source;
      return `${when} · ${source} · ${item.command} · ${item.acknowledged ? '主控确认' : '未确认'}\n${item.raw_response || item.error || '无回复'}`;
    }).join('\n\n');
  }
}

async function gripperAction(action, body = {}) {
  if (state.gripperPending && action !== 'disable') return;
  const requestId = Symbol(action);
  state.gripperPending = requestId;
  renderGripper(state.gripper);
  const message = $('gripper-message');
  message.classList.remove('error');
  message.textContent = '正在等待夹爪命令回复…';
  try {
    const result = await api(`/api/robot/actions/gripper-${action}`, jsonOptions('POST', body));
    renderGripper(result.gripper);
    const labels = {enable:'夹爪控制已就绪；点动松键后可直接再次按住继续。',disable:'主控确认夹爪失能。',open:'主控收到打开命令；计时结束会自动失能，请观察实际开口。',close:'主控收到闭合命令；计时结束会自动失能，请观察实际开口。',current:'主控确认开合电流设定；这不是位置或校准动作的通用限流。'};
    message.textContent = labels[action];
  } catch (error) {
    if (error.data?.gripper) renderGripper(error.data.gripper);
    message.textContent = error.message;
    message.classList.add('error');
  } finally {
    if (state.gripperPending === requestId) state.gripperPending = null;
    try { renderStatus(await api('/api/status')); } catch (_) { renderGripper(state.gripper); }
  }
}

function jogButton(direction) {
  return direction === 'close' ? $('gripper-jog-close') : $('gripper-jog-open');
}

function clearJogHeartbeat() {
  if (state.jog.heartbeatTimer) clearInterval(state.jog.heartbeatTimer);
  state.jog.heartbeatTimer = null;
  state.jog.heartbeatBusy = false;
}

async function jogHeartbeat(direction) {
  if (state.jog.direction !== direction || state.jog.stopRequested || state.jog.heartbeatBusy) return;
  state.jog.heartbeatBusy = true;
  try {
    const result = await api('/api/robot/actions/gripper-jog-heartbeat', jsonOptions('POST', {direction}));
    if (state.jog.direction === direction && !state.jog.stopRequested) renderGripper(result.gripper);
  } catch (error) {
    toast(`夹爪点动已停止：${error.message}`, true);
    await stopJog(direction);
  } finally {
    state.jog.heartbeatBusy = false;
  }
}

async function startJog(direction) {
  if (state.jog.direction === direction || state.jog.pending) return;
  if (state.jog.direction && state.jog.direction !== direction) return;
  const button = jogButton(direction);
  if (button.disabled) return;
  state.jog = {direction, pending:true, stopRequested:false, heartbeatTimer:null, heartbeatBusy:false};
  renderGripper(state.gripper);
  try {
    const result = await api('/api/robot/actions/gripper-jog-start', jsonOptions('POST', {direction}));
    if (state.jog.direction !== direction || state.jog.stopRequested) {
      await stopJog(direction);
      return;
    }
    state.jog.pending = false;
    state.jog.heartbeatTimer = setInterval(() => jogHeartbeat(direction), 500);
    renderGripper(result.gripper);
    $('gripper-message').textContent = direction === 'close' ? '正在点动闭合，松开后停止；可再次按住继续。' : '正在点动张开，松开后停止；可再次按住继续。';
  } catch (error) {
    clearJogHeartbeat();
    if (state.jog.direction === direction) state.jog = {direction:null, pending:false, stopRequested:false, heartbeatTimer:null, heartbeatBusy:false};
    renderGripper(state.gripper);
    $('gripper-message').textContent = error.message;
    $('gripper-message').classList.add('error');
  }
}

async function stopJog(direction = state.jog.direction) {
  if (!direction || state.jog.direction !== direction) return;
  state.jog.stopRequested = true;
  clearJogHeartbeat();
  try {
    const result = await api('/api/robot/actions/gripper-jog-stop', jsonOptions('POST'));
    if (state.jog.direction === direction) {
      state.jog = {direction:null, pending:false, stopRequested:false, heartbeatTimer:null, heartbeatBusy:false};
    }
    renderGripper(result.gripper);
    $('gripper-message').textContent = '夹爪运动已停止；再次按住会自动恢复使能并继续。';
  } catch (error) {
    toast(`夹爪停止未确认：${error.message}`, true);
  } finally {
    if (state.jog.direction === direction && !state.jog.pending) {
      state.jog = {direction:null, pending:false, stopRequested:false, heartbeatTimer:null, heartbeatBusy:false};
    }
  }
}

function jogPointerDown(event, direction) {
  event.preventDefault();
  jogButton(direction).setPointerCapture?.(event.pointerId);
  startJog(direction);
}

function jogPointerStop(event, direction) {
  event.preventDefault();
  stopJog(direction);
}

function isTextEntry(target) {
  return target && (target.matches?.('input, textarea, select, [contenteditable="true"]'));
}

function sampleTime(stamp, stale) {
  return `${stale ? '上次' : '读取'} ${new Date(stamp).toLocaleTimeString('zh-CN', {hour12:false})}`;
}

function renderGripperAngle(feedback) {
  const available = feedback?.status === 'available' && Number.isFinite(feedback.angle_deg);
  const stale = !available || feedback.stale || Date.now() - feedback.sampled_at_ms > 2500;
  const logical = Number.isFinite(feedback?.logical_angle_deg) ? feedback.logical_angle_deg : feedback?.angle_deg;
  $('joint-6').textContent = available ? `${logical.toFixed(2)}°` : '--';
  $('joint-6').closest('.joint').classList.toggle('stale', Boolean(stale));
  $('joint-time-6').textContent = available ? sampleTime(feedback.sampled_at_ms, stale) : '读取不可用';
  $('joint-6').closest('.joint').title = available ? '夹爪软件逻辑角：首次有效读数按全闭 115° 计算；原始角度见状态。' : (feedback?.message || '需要连接 ST-LINK 才能读取夹爪角度。');
  $('gripper-angle').textContent = available
    ? `逻辑角：${logical.toFixed(2)}° · 原始：${feedback.angle_deg.toFixed(2)}° · ${sampleTime(feedback.sampled_at_ms, stale)}`
    : (feedback?.message || '夹爪角度尚未读取');
  const source = feedback?.reference_source === 'operator_confirmed_closed' ? '已人工确认闭合参考' : '首次读数假定为闭合参考';
  $('gripper-reference-state').textContent = available ? `${source}（115°）` : '尚未建立软件参考';
}

async function captureGripperReference() {
  const button = $('gripper-reference');
  button.disabled = true;
  try {
    const result = await api('/api/robot/gripper/reference', jsonOptions('POST'));
    renderGripperAngle(result);
    toast('已记录当前夹爪为软件闭合参考 115°');
    try { renderStatus(await api('/api/status')); } catch (_) {}
  } catch (error) {
    toast(error.message, true);
  } finally {
    button.disabled = false;
  }
}

async function readAllAngles() {
  const button = $('read-joints');
  button.disabled = true;
  button.textContent = '读取中…';
  try {
    const robot = await api('/api/robot/state');
    renderRobot(robot);
    const okay = robot.state?.gripper?.status === 'available' && !robot.state.gripper.stale;
    toast(okay ? '六轴与夹爪角度已更新' : `六轴角度已更新；${robot.state?.gripper?.message || '夹爪读取不可用'}`, !okay);
  } catch (error) {
    if (error.data?.state) renderRobot(error.data);
    toast(error.message, true);
  } finally {
    button.disabled = false;
    button.textContent = '读取角度';
  }
}

function renderRawConsole() {
  const robot = state.status?.robot;
  const raw = robot?.raw_console;
  const command = $('raw-command').value;
  const wait = Number($('raw-timeout').value);
  const valid = command.trim().length > 0 && command.length <= 62 && /^[\x20-\x7e]+$/.test(command)
    && $('raw-timeout').value !== '' && Number.isInteger(wait) && wait >= 0 && wait <= 2000;
  const busy = Boolean(state.rawPending || raw?.busy);
  $('raw-send').disabled = !valid || !robot || busy || robot.motion?.status === 'running'
    || robot.gripper?.busy || robot.gripper?.enabled_latch === 'confirmed' || robot.gripper?.control_ready || robot.gripper?.faulted;
  $('raw-command').disabled = $('raw-timeout').disabled = busy;
  $('raw-mode').textContent = busy ? '等待通信返回…' : raw?.active ? '原始模式 · 自动角度采样暂停' : '按钮控制模式';
  const history = raw?.history || [];
  if (history.length) $('raw-replies').textContent = [...history].reverse().map(item => {
    const sent = item.sent === true ? '串口已写入 · 执行结果未知' : '发送未确认 · 执行结果未知';
    return `${new Date(item.at_ms).toLocaleTimeString('zh-CN', {hour12:false})} > ${item.command}\n${sent}\n原始回复：\n${item.raw_response || '（无）'}\n发送前残留：\n${item.pending_response || '（无）'}\n桥接详情：\n${JSON.stringify(item.bridge || {error:item.transport_error || item.error}, null, 2)}`;
  }).join('\n\n');
}

async function sendRawCommand() {
  if ($('raw-send').disabled || state.rawPending) return;
  const body = {command:$('raw-command').value, read_timeout_ms:Number($('raw-timeout').value)};
  state.rawPending = true;
  renderRawConsole();
  const message = $('raw-message');
  message.classList.remove('error');
  message.textContent = '正在发送并等待原始回复…';
  try {
    const result = await api('/api/robot/command', jsonOptions('POST', body));
    message.textContent = result.sent === true ? '串口已写入，请查看回复并观察机器人；动作是否完成未知。' : '发送状态未知，请查看回复。';
  } catch (error) {
    message.textContent = error.message;
    message.classList.add('error');
    if (error.data?.command) $('raw-replies').textContent = JSON.stringify(error.data, null, 2);
  } finally {
    state.rawPending = false;
    try { renderStatus(await api('/api/status')); } catch (_) {
      state.status = null;
      renderRawConsole();
      $('raw-mode').textContent = '服务断开 · 状态未知';
    }
  }
}

function stopGripperOnLeave() {
  if (state.jog.direction || state.jog.pending) {
    fetch('/api/robot/actions/gripper-jog-stop', {...jsonOptions('POST'), keepalive:true}).catch(() => {});
  }
}

function openMotionDialog(name) {
  state.pendingPreset = name;
  const target = state.status?.robot.presets[name];
  $('dialog-title').textContent = name === 'home' ? '确认回零位' : '确认回休止位';
  $('dialog-target').textContent = `[${(target || []).join(', ')}]°  ·  speed 4`;
  $('motion-dialog').showModal();
}

function cancelHold() {
  clearTimeout(state.holdTimer); state.holdTimer = null; $('hold-confirm').classList.remove('holding');
}

function startHold(event) {
  event.preventDefault(); cancelHold();
  $('hold-confirm').classList.add('holding');
  state.holdTimer = setTimeout(async () => {
    const name = state.pendingPreset;
    cancelHold(); $('motion-dialog').close();
    await robotAction(name, { confirmation: name === 'home' ? 'MOVE_HOME' : 'MOVE_REST' });
  }, 1500);
}

function installStreamRetry(id, stream) {
  const image = $(id);
  image.onerror = () => setTimeout(() => { image.src = `/api/streams/${stream}.mjpeg?t=${Date.now()}`; }, 1000);
}

for (let index = 0; index < 7; index++) {
  const cell = document.createElement('div'); cell.className = 'joint';
  const label = document.createElement('span'); label.textContent = index === 6 ? '夹爪 · 电机角' : `J${index + 1}`;
  const value = document.createElement('strong'); value.id = `joint-${index}`; value.textContent = '--';
  const time = document.createElement('small'); time.id = `joint-time-${index}`; time.textContent = '尚未读取';
  cell.append(label, value, time); $('joints').append(cell);
}

$('discover').onclick = () => discoverDevices(true);
$('apply-wrist').onclick = applyWrist;
$('apply-oak').onclick = applyOak;
$('record-toggle').onclick = toggleRecording;
$('enable').onclick = () => robotAction('enable');
$('disable').onclick = () => robotAction('disable');
$('gripper-enable').onclick = () => gripperAction('enable');
$('gripper-disable').onclick = () => gripperAction('disable');
$('gripper-open').onclick = () => gripperAction('open', {duration_ms:Number($('gripper-duration').value)});
$('gripper-close').onclick = () => gripperAction('close', {duration_ms:Number($('gripper-duration').value)});
$('gripper-current-send').onclick = () => gripperAction('current', {current:Number($('gripper-current').value)});
$('gripper-reference').onclick = captureGripperReference;
$('gripper-jog-close').addEventListener('pointerdown', event => jogPointerDown(event, 'close'));
$('gripper-jog-open').addEventListener('pointerdown', event => jogPointerDown(event, 'open'));
for (const [id, direction] of [['gripper-jog-close','close'], ['gripper-jog-open','open']]) {
  const button = $(id);
  for (const name of ['pointerup', 'pointercancel', 'pointerleave', 'lostpointercapture']) {
    button.addEventListener(name, event => jogPointerStop(event, direction));
  }
  button.addEventListener('contextmenu', event => event.preventDefault());
}
$('gripper-current').addEventListener('input', () => renderGripper(state.gripper));
$('raw-command').addEventListener('input', renderRawConsole);
$('raw-command').addEventListener('paste', event => {
  const text = event.clipboardData.getData('text');
  if (/[^\x20-\x7e]/.test(text)) {
    event.preventDefault();
    $('raw-message').textContent = '已拒绝粘贴：只能输入单行可打印 ASCII，不含换行。';
    $('raw-message').classList.add('error');
  }
});
$('raw-timeout').addEventListener('input', renderRawConsole);
$('raw-send').onclick = sendRawCommand;
window.addEventListener('pagehide', stopGripperOnLeave);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopGripperOnLeave(); });
$('emergency').onclick = emergencyStop;
$('read-joints').onclick = readAllAngles;
window.addEventListener('keydown', event => {
  if (isTextEntry(event.target) || event.repeat) return;
  const direction = event.key === 'ArrowLeft' ? 'close' : event.key === 'ArrowRight' ? 'open' : null;
  if (!direction) return;
  event.preventDefault();
  startJog(direction);
});
window.addEventListener('keyup', event => {
  const direction = event.key === 'ArrowLeft' ? 'close' : event.key === 'ArrowRight' ? 'open' : null;
  if (direction) { event.preventDefault(); stopJog(direction); }
});
document.querySelectorAll('.camera-toggle').forEach(button => button.onclick = async () => { try { await api(`/api/cameras/${button.dataset.camera}/${button.dataset.action}`, jsonOptions('POST')); } catch (error) { toast(error.message, true); } });
document.querySelectorAll('.button.motion').forEach(button => button.onclick = () => openMotionDialog(button.dataset.preset));
for (const eventName of ['pointerdown']) $('hold-confirm').addEventListener(eventName, startHold);
for (const eventName of ['pointerup', 'pointercancel', 'pointerleave']) $('hold-confirm').addEventListener(eventName, cancelHold);
$('motion-dialog').addEventListener('close', cancelHold);
installStreamRetry('stream-wrist', 'wrist');
installStreamRetry('stream-oak-rgb', 'oak-rgb');
installStreamRetry('stream-oak-depth', 'oak-depth');
setInterval(() => { $('capture-clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false }); }, 1000);

Promise.all([loadConfigs(), discoverDevices(), loadRecordings()]).catch(error => toast(error.message, true));
pollStatus();
