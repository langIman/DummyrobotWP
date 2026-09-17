// Local static server + intercepted APIs only. Never connects to robot services.
const {chromium}=require('../runtime/ui-check/node_modules/playwright');
const http=require('node:http');
const fs=require('node:fs');
const path=require('node:path');
const assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
  const name=req.url.split('?')[0];
  const file=name==='/collect'?'collect.html':name.startsWith('/static/')?path.basename(name):null;
  if(!file || !/\.(html|css|js)$/.test(file)){res.writeHead(404);return res.end();}
  res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html');
  res.end(fs.readFileSync(path.join(root,'static',file)));
});
async function main(){
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const url=`http://127.0.0.1:${server.address().port}/collect`;
  const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
  const reports=[];
  try{
    for(const [name,viewport] of [['desktop',{width:1440,height:900}],['mobile',{width:390,height:844}]]){
      const context=await browser.newContext({viewport});
      const commands=[],actions=[],errors=[];
      const control={armed:false,backend_status:'ready',diagnostics:{output:'idle'},input_command:{}};
      const gripper={control_ready:false,enabled_latch:'disabled',jog_direction:null};
      let armLatch='disabled';
      const recording={active:false};
      const camera={frame_available:true,frame_age_ms:10,dimensions:[1280,720],measured_fps:30};
      const status=()=>({cameras:{wrist:camera,oak:camera},recording,control,
        robot:{bridge_status:'ready',enabled_latch:armLatch,gripper,state:{positions:[0,0,90,0,45,0],sampled_at_ms:Date.now()}}});
      await context.route('**/api/**',async route=>{
        const req=route.request(),p=new URL(req.url()).pathname;
        let result={};
        if(p.startsWith('/api/streams/')) return route.fulfill({contentType:'image/svg+xml',body:'<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720"><rect width="1280" height="720" fill="#243b40"/><text x="60" y="140" fill="white" font-size="48">Test camera</text></svg>'});
        if(p==='/api/status')result=status();
        else if(p==='/api/robot/actions/enable'){armLatch='confirmed';actions.push('enable');result=status().robot;}
        else if(p==='/api/robot/actions/gripper-enable'){gripper.control_ready=true;gripper.enabled_latch='confirmed';actions.push('gripper-enable');result={acknowledged:true};}
        else if(p==='/api/control/arm'){control.armed=true; result={...control,session_id:'fake-owner'};}
        else if(p==='/api/control/input'){
          const body=req.postDataJSON();assert.equal(body.session_id,'fake-owner');commands.push(body);result={accepted:true};
        }else if(p==='/api/robot/actions/gripper-jog-start'){
          gripper.jog_direction=req.postDataJSON().direction;actions.push(gripper.jog_direction);result={acknowledged:true};
        }else if(p==='/api/robot/actions/gripper-jog-stop'){gripper.jog_direction=null;actions.push('stop');result={acknowledged:true};}
        else if(p==='/api/robot/actions/gripper-jog-heartbeat')result={acknowledged:true};
        else if(p==='/api/control/disarm'){
          control.armed=false;armLatch='disabled';gripper.control_ready=false;gripper.enabled_latch='disabled';gripper.jog_direction=null;
          actions.push('disarm');result={armed:false,overall_acknowledged:true};
        }else if(p==='/api/recordings/start')recording.active=true;
        else if(p==='/api/recordings/stop')recording.active=false;
        else if(p==='/api/recordings/control-event')result={accepted:true};
        else{errors.push(`Unexpected API ${p}`);return route.abort();}
        return route.fulfill({contentType:'application/json',body:JSON.stringify(result)});
      });
      const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
      await page.goto(url);await page.waitForSelector('#collect-arm-enable:enabled');
      assert.equal(await page.locator('#collect-arm-enable').textContent(),'使能六轴+夹爪');
      await page.locator('#collect-arm-enable').click();
      await page.waitForFunction(()=>document.getElementById('collect-message').textContent.includes('六轴和夹爪已使能'));
      assert.deepEqual(actions,['enable','gripper-enable']);actions.length=0;
      await page.waitForSelector('#collect-arm-hold:enabled');
      for(const id of ['collect-speed','collect-feedback','collect-ready-state','collect-tool-frame','collect-release-stop','target-gap','collect-gripper-disable','look-plane'])
        assert.equal(await page.locator(`#${id}`).count(),0,`${id} removed`);
      assert.equal(await page.locator('#collect-joints strong').count(),6);
      assert.equal(await page.locator('#collect-joints strong').nth(2).textContent(),'90.00°');
      const hold=page.locator('#collect-arm-hold');await hold.scrollIntoViewIfNeeded();
      let b=await hold.boundingBox();await page.mouse.move(b.x+b.width/2,b.y+b.height/2);
      await page.mouse.down();await page.waitForTimeout(1400);await page.mouse.up();
      await page.waitForFunction(()=>document.getElementById('collect-mode').textContent==='实机控制');
      const wait=()=>page.waitForTimeout(120);
      await page.keyboard.down('w');await wait();assert.deepEqual(commands.at(-1).keys,['w']);
      await page.keyboard.down('ArrowUp');await wait();assert.deepEqual(commands.at(-1).keys,['arrowup','w']);
      await page.keyboard.up('w');await page.keyboard.up('ArrowUp');await wait();
      assert.equal(commands.at(-1).stop_mode,'release');assert.deepEqual(commands.at(-1).keys,[]);
      for(const [key,yaw,pitch] of [['8',0,1],['2',0,-1],['4',-1,0],['6',1,0]]){
        await page.keyboard.down(key);await wait();
        assert.equal(commands.at(-1).yaw,yaw);assert.equal(commands.at(-1).pitch,pitch);
        assert(await page.locator(`[data-key="${key}"]`).evaluate(e=>e.classList.contains('active')));
        await page.keyboard.up(key);await wait();
        assert.equal(commands.at(-1).yaw,0);assert.equal(commands.at(-1).pitch,0);
      }
      const panel=page.locator('#control-keyboard');await panel.scrollIntoViewIfNeeded();b=await panel.boundingBox();
      await page.mouse.move(b.x+10,b.y+10);await page.mouse.click(b.x+20,b.y+20);await wait();
      assert.equal(commands.at(-1).yaw,0);assert.equal(commands.at(-1).pitch,0);assert.equal(actions.length,0,'mouse cannot operate gripper');
      await page.keyboard.down('0');await wait();assert.equal(gripper.jog_direction,'close');
      await page.keyboard.up('0');await wait();assert.equal(gripper.jog_direction,null);
      await page.keyboard.down('.');await wait();assert.equal(gripper.jog_direction,'open');
      await page.keyboard.up('.');await wait();assert.equal(gripper.jog_direction,null);
      await page.keyboard.down('8');await page.keyboard.down('0');await wait();
      await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await wait();
      assert.equal(commands.at(-1).pitch,0);assert.equal(commands.at(-1).stop_mode,'immediate');assert.equal(gripper.jog_direction,null);
      await page.keyboard.up('8');await page.keyboard.up('0');
      await page.locator('#collect-label').fill('标签');const beforeTyping=actions.length;
      await page.keyboard.type('wasd82460.');await wait();
      assert.equal(actions.length,beforeTyping);assert.deepEqual(commands.at(-1).keys,[]);assert.equal(commands.at(-1).pitch,0);
      await page.locator('#collect-record-start').click();await page.waitForFunction(()=>document.getElementById('collect-record-state').textContent==='录制中');
      await page.locator('#collect-record-stop').click();await page.waitForFunction(()=>document.getElementById('collect-record-state').textContent==='空闲');
      const other=await context.newPage();other.on('pageerror',e=>errors.push(e.message));
      await other.goto(url);await other.waitForFunction(()=>document.getElementById('collect-safety').textContent.includes('另一个页面'));
      const beforeOther=actions.length;await other.keyboard.press('0');await other.keyboard.press('8');await other.waitForTimeout(100);
      assert.equal(actions.length,beforeOther,'non-owner page cannot control gripper');
      await other.close();await page.bringToFront();
      const layout=await page.evaluate(()=>{
        const boxes=[...document.querySelectorAll('.live-actions button,.gripper-actions button')].map(e=>{const b=e.getBoundingClientRect();return {x:b.x,y:b.y,w:b.width,h:b.height};});
        return {width:document.documentElement.scrollWidth,viewport:innerWidth,
          overlap:boxes.some((a,i)=>boxes.slice(i+1).some(b=>a.x<b.x+b.w&&b.x<a.x+a.w&&a.y<b.y+b.h&&b.y<a.y+a.h)),
          images:[...document.querySelectorAll('.camera-screen img')].map(e=>e.naturalWidth)};
      });
      assert(layout.width<=layout.viewport&&!layout.overlap,JSON.stringify(layout));assert(layout.images.every(w=>w>0));
      await page.locator('#collect-arm-disarm').click();await page.waitForFunction(()=>document.getElementById('collect-message').textContent.includes('六轴与夹爪一起失能'));
      assert.equal(gripper.control_ready,false);assert(actions.includes('disarm'));
      control.last_stop_reason='input_stop_failed: hold_feedback_unavailable: local_worker_deadline';control.diagnostics.emergency_acknowledged=false;
      await page.waitForFunction(()=>document.getElementById('collect-safety').textContent.includes('位置保持目标未发送'));
      assert.match(await page.locator('#collect-safety').textContent(),/硬件状态未知/);
      control.last_stop_reason='';await page.waitForFunction(()=>!document.getElementById('collect-safety').textContent.includes('位置保持目标未发送'));
      await page.evaluate(()=>window.scrollTo(0,0));
      await page.screenshot({path:path.join(root,'runtime',`collect-reviewed-${name}.png`),fullPage:true});
      assert.deepEqual(errors,[]);reports.push({name,layout,commands:commands.length,actions,result:'passed'});await context.close();
    }
    console.log(JSON.stringify(reports,null,2));fs.writeFileSync(path.join(root,'runtime/collect-ui-report.json'),JSON.stringify(reports,null,2));
  }finally{await browser.close();server.close();}
}
main().catch(error=>{console.error(error);server.close();process.exitCode=1;});
