// No network or robot access: exercise the real page event handlers in a DOM stub.
const vm = require('node:vm');
const fs = require('node:fs');
const assert = require('node:assert/strict');
const path = require('node:path');
const handlers = {};
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    style:{}, dataset:{}, classList:{toggle(){},add(){},remove(){}},
    addEventListener(){}, matches(){return false;}, focus(){},
    getBoundingClientRect(){return {left:0,top:0,width:400,height:400};},
  });
  return elements.get(id);
}
const sent = [];
const timers = new Map();
let nextTimer = 0;
const actions = [];
const context = vm.createContext({
  document:{getElementById:element,querySelector:element,addEventListener(name,fn){handlers[name]=fn;}},
  window:{addEventListener(name,fn){handlers[name]=fn;}},
  setInterval(){}, setTimeout(fn){const id=++nextTimer; timers.set(id,fn); return id;}, clearTimeout(id){timers.delete(id);},
  actions,
  fetch(url,options){
    if (url === '/api/status') return new Promise(()=>{});
    sent.push(JSON.parse(options.body));
    return Promise.resolve({ok:true,json:async()=>({})});
  },
});
vm.runInContext(fs.readFileSync(path.join(__dirname,'../static/collect.js'),'utf8'), context);
vm.runInContext('input.controlArmed = true; input.sessionId = "test-owner"', context);
async function main() {
  vm.runInContext("setView('depth-meta', {rgb:{dimensions:[1280,720]},depth:{dimensions:[640,360],frame_available:true,frame_age_ms:2500},state:'streaming'}, 'depth-view', 'depth')", context);
  assert.equal(element('depth-view').dataset.state,'stale');
  assert.match(element('depth-meta').textContent,/640 × 360/);
  assert.equal(vm.runInContext('input.yaw',context),0);
  assert.equal(vm.runInContext('input.pitch',context),0);
  for (const key of ['W','A','S','D']) {
    const event = {code:`Key${key}`,key:'Process',target:element('plane'),preventDefault(){}};
    handlers.keydown(event);
    await new Promise(setImmediate);
    assert.deepEqual(sent.at(-1).keys,[key.toLowerCase()]);
    handlers.keyup(event);
    await new Promise(setImmediate);
    assert.deepEqual(sent.at(-1).keys,[]);
    assert.equal(sent.at(-1).stop_mode,'release','normal release preserves last target');
  }
  handlers.keydown({code:'KeyW',key:'w',target:{matches:()=>true},preventDefault(){}});
  assert.equal(vm.runInContext('input.keys.size', context),0,'typing a label must not move robot');
  handlers.keydown({code:'KeyW',key:'w',target:element('plane'),preventDefault(){}});
  await new Promise(setImmediate);
  handlers.blur();
  await new Promise(setImmediate);
  assert.deepEqual(sent.at(-1).keys,[],'losing focus must clear movement');
  assert.equal(sent.at(-1).stop_mode,'immediate','blur must not request forward extension');
  handlers.keydown({key:'Escape',code:'Escape',target:element('plane'),preventDefault(){}});
  await new Promise(setImmediate);
  assert.equal(sent.at(-1).stop_mode,'immediate');
  vm.runInContext('robotAction = async (name, body) => actions.push({name,body})', context);
  assert.equal(actions.length,0,'loading the page never starts a preset');
  vm.runInContext('beginReadyHold({preventDefault(){}}); cancelReadyHold()', context);
  assert.equal(timers.size,0,'releasing before confirmation cancels the preset');
  vm.runInContext('beginReadyHold({preventDefault(){}})', context);
  await [...timers.values()][0]();
  assert.equal(actions.length,1);
  assert.equal(actions[0].name,'ready');
  assert.equal(actions[0].body.confirmation,'MOVE_READY');
  await vm.runInContext('stopGripper(true)',context);
  assert.equal(actions.at(-1).name,'gripper-jog-stop','explicit Stop must work without a local hold');
  const tick = () => new Promise(setImmediate);
  const event = (code,key='Unidentified',extra={}) => ({code,key,target:element('plane'),preventDefault(){},...extra});
  for (const [code,key,yaw,pitch] of [['Numpad8','ArrowUp',0,1],['Numpad2','ArrowDown',0,-1],['Numpad4','ArrowLeft',-1,0],['Numpad6','ArrowRight',1,0]]) {
    handlers.keydown(event(code,key)); await tick();
    assert.equal(sent.at(-1).yaw,yaw); assert.equal(sent.at(-1).pitch,pitch);
    assert.deepEqual(sent.at(-1).keys,[],'Num Lock off must not turn numpad into translation');
    handlers.keyup(event(code,key)); await tick();
    assert.equal(sent.at(-1).yaw,0); assert.equal(sent.at(-1).pitch,0);
    assert.equal(sent.at(-1).stop_mode,'release');
  }
  handlers.keydown(event('Digit8','8')); handlers.keydown(event('Numpad6','6')); await tick();
  assert(Math.abs(Math.hypot(sent.at(-1).yaw,sent.at(-1).pitch)-1)<1e-10,'diagonal turn respects angular cap');
  handlers.keydown(event('Numpad8','8')); handlers.keyup(event('Digit8','8')); await tick();
  assert(sent.at(-1).pitch>0,'release one of two physical keys must preserve the other');
  handlers.blur(); await tick();
  assert.equal(sent.at(-1).pitch,0); assert.equal(sent.at(-1).yaw,0);
  handlers.keydown(event('Digit8','8',{target:{matches:()=>true}}));
  handlers.keydown(event('Digit8','8',{ctrlKey:true}));
  assert.equal(vm.runInContext('pressed.size',context),0);

  actions.length=0;
  handlers.keydown(event('Numpad0','Insert')); await tick();
  assert.deepEqual(actions.map(a=>a.name),['gripper-jog-start']);
  assert.equal(actions[0].body.direction,'close');
  handlers.keydown(event('Numpad0','Insert',{repeat:true})); await tick();
  assert.equal(actions.length,1,'key repeat must not restart jog');
  handlers.keydown(event('NumpadDecimal','Delete')); await tick();
  assert.equal(actions.at(-1).name,'gripper-jog-stop','conflicting gripper keys stop');
  handlers.keyup(event('Numpad0','Insert')); await tick();
  assert.equal(actions.at(-1).body.direction,'open','remaining key resumes its direction');
  handlers.keyup(event('NumpadDecimal','Delete')); await tick();
  assert.equal(actions.at(-1).name,'gripper-jog-stop');

  // Release / reverse while the start response is still pending.
  vm.runInContext(`robotAction = async (name,body) => {
    actions.push({name,body});
    if (name === 'gripper-jog-start' && body.direction === 'close') await new Promise(resolve => { globalThis.resolveStart = resolve; });
  }`,context);
  actions.length=0;
  handlers.keydown(event('Digit0','0')); await tick();
  handlers.keyup(event('Digit0','0'));
  handlers.keydown(event('Period','.'));
  vm.runInContext('resolveStart()',context); await tick();
  assert.deepEqual(actions.map(a=>[a.name,a.body?.direction]),[
    ['gripper-jog-start','close'],['gripper-jog-stop',undefined],['gripper-jog-start','open']]);
  handlers.blur(); await tick(); assert.equal(actions.at(-1).name,'gripper-jog-stop');
  handlers.keydown(event('Period','.')); await tick();
  handlers.focusin({target:{matches:()=>true}}); await tick();
  assert.equal(actions.at(-1).name,'gripper-jog-stop','focus in text field stops existing jog');
  handlers.keydown(event('Numpad8','8',{repeat:true})); await tick();
  assert.equal(vm.runInContext('pressed.size',context),0,'auto-repeat cannot resume after blur');
  vm.runInContext(`
    globalThis.fakeRobot = {enabled_latch:'disabled',gripper:{control_ready:false}};
    globalThis.failGripper = false;
    api = async () => ({robot:fakeRobot,control:{armed:false}});
    renderStatus = () => {};
    robotAction = async name => {
      actions.push({name});
      if (name === 'enable') fakeRobot.enabled_latch = 'confirmed';
      if (name === 'gripper-enable') {
        if (failGripper) throw new Error('夹爪未确认');
        fakeRobot.gripper.control_ready = true;
      }
      return fakeRobot;
    };
  `,context);
  actions.length=0;
  await vm.runInContext('enableArm()',context);
  assert.deepEqual(actions.map(a=>a.name),['enable','gripper-enable']);
  await vm.runInContext('enableArm()',context);
  assert.equal(actions.length,2,'already ready motors are not enabled twice');
  vm.runInContext('fakeRobot.gripper.control_ready=false; failGripper=true',context);
  await vm.runInContext('enableArm()',context);
  assert.match(element('collect-message').textContent,/使能未全部完成/);
  assert.equal(actions.at(-1).name,'gripper-enable');
  vm.runInContext('failGripper=false',context);
  await vm.runInContext('enableArm()',context);
  assert.equal(actions.filter(a=>a.name==='enable').length,1,'retry only the missing gripper');
  vm.runInContext(`
    fakeRobot.enabled_latch='disabled';fakeRobot.gripper.control_ready=false;
    robotAction = async name => { actions.push({name}); await new Promise(resolve=>{globalThis.resolveEnable=resolve;});return fakeRobot; };
  `,context);
  actions.length=0;
  const pendingEnable=vm.runInContext('enableArm()',context);await tick();
  await vm.runInContext('emergencyStop()',context);
  vm.runInContext('resolveEnable()',context);await pendingEnable;
  assert.deepEqual(actions.map(a=>a.name),['enable'],'emergency cancels queued gripper enable');
  console.log('Combined enable: both motors, partial failure/retry, emergency cancellation passed.');
  console.log('Numpad/main digits, Num Lock, opposing keys, diagonal cap, async gripper reversal and blur passed.');
  console.log('Ready preset: no automatic motion; early release cancels; hold sends confirmation.');
  console.log('Keyboard: IME WASD, releases, text entry and blur passed (no hardware).');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
