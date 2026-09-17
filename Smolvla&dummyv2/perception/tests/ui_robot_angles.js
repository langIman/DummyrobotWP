// All robot reads and all writes are mocked before navigation.
const {chromium} = require(process.env.PERCEPTION_PLAYWRIGHT_MODULE || 'playwright');
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({headless:true,
    executablePath:'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe'});
  const reports = [];
  try {
    for (const [name,width,height] of [['desktop',1440,900],['mobile',390,844]]) {
      const context = await browser.newContext({viewport:{width,height}});
      const page = await context.newPage();
      const errors = [], writes = [];
      let reads = 0;
      page.on('pageerror', error => errors.push(error.message));
      const baseline = await (await context.request.get('http://127.0.0.1:8770/api/status')).json();
      baseline.robot.gripper.enabled_latch = 'disabled';
      baseline.robot.gripper.busy = false;
      baseline.robot.motion = null;
      baseline.robot.raw_console.active = false;
      baseline.robot.state.positions = [0,-75,180,0,0,0];
      baseline.robot.state.sampled_at_ms = Date.now();
      const feedback = {status:'available',angle_deg:22.099218,sampled_at_ms:Date.now(),stale:false};
      baseline.robot.state.gripper = feedback;
      await page.route('**/api/**', async route => {
        const request = route.request(), pathname = new URL(request.url()).pathname;
        if (request.method() !== 'GET') {
          writes.push(pathname);
          return route.fulfill({status:409,json:{error:{message:'Blocked by offline UI test'}}});
        }
        if (pathname === '/api/status') return route.fulfill({json:baseline});
        if (pathname === '/api/robot/state') {
          reads++;
          baseline.robot.state.positions[0] = 1.23;
          baseline.robot.state.sampled_at_ms = Date.now();
          feedback.angle_deg = -17.106153;
          feedback.sampled_at_ms = Date.now();
          return route.fulfill({json:baseline.robot});
        }
        return route.continue();
      });
      await page.goto('http://127.0.0.1:8770/',{waitUntil:'domcontentloaded'});
      await page.waitForFunction(() => document.querySelector('#joint-6').textContent === '22.10°');
      assert.equal(await page.locator('#joints .joint').count(),7);
      await page.locator('#read-joints').click();
      await page.waitForFunction(() => document.querySelector('#joint-6').textContent === '-17.11°');
      assert.equal(await page.locator('#joint-0').textContent(),'1.23°');
      assert.equal(reads,1);
      assert.match(await page.locator('#toast').textContent(),/六轴与夹爪/);
      // A later status poll updates gripper without another button click.
      feedback.angle_deg = -50.25;
      await page.waitForFunction(() => document.querySelector('#joint-6').textContent === '-50.25°');
      feedback.status = 'unavailable';
      feedback.message = '夹爪读取不可用，请检查 ST-LINK 和主控供电。';
      feedback.angle_deg = null;
      await page.waitForFunction(() => document.querySelector('#joint-6').textContent === '--');
      assert.equal(await page.locator('#joint-0').textContent(),'1.23°');
      assert.match(await page.locator('#gripper-angle').textContent(),/ST-LINK/);
      Object.assign(feedback,{status:'available',angle_deg:22.099218,stale:true,sampled_at_ms:Date.now()-10000});
      await page.waitForFunction(() => document.querySelector('#joint-time-6').textContent.startsWith('上次'));
      assert.equal(await page.locator('#joint-6').evaluate(node => node.parentElement.classList.contains('stale')),true);
      const layout = await page.locator('.robot-band').evaluate(section => ({
        overflow:document.documentElement.scrollWidth > document.documentElement.clientWidth,
        clipped:[...section.querySelectorAll('button,.joint strong,.joint small')]
          .filter(node => node.scrollWidth > node.clientWidth+1).map(node => node.textContent),
        overlap:[...section.querySelectorAll('.joint')].some((node,i,list) => list.slice(i+1).some(other => {
          const a=node.getBoundingClientRect(), b=other.getBoundingClientRect();
          return Math.min(a.right,b.right)>Math.max(a.left,b.left)+1 && Math.min(a.bottom,b.bottom)>Math.max(a.top,b.top)+1;
        }))
      }));
      await page.locator('.robot-band').screenshot({path:path.join('runtime',`robot-angles-${name}.png`)});
      assert.deepEqual(layout,{overflow:false,clipped:[],overlap:false});
      assert.deepEqual(errors,[]);
      assert.deepEqual(writes,[]);
      reports.push({name,reads,writes,layout,errors});
      await context.close();
    }
  } finally { await browser.close(); }
  console.log(JSON.stringify(reports,null,2));
})().catch(error => {console.error(error);process.exitCode=1;});
