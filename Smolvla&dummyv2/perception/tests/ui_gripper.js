// All mutating requests are intercepted. This test never controls real motors.
const {chromium} = require(process.env.PERCEPTION_PLAYWRIGHT_MODULE || 'playwright');
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({headless:true,
    executablePath:'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe'});
  const reports = [];
  try {
    for (const [name, width, height] of [['desktop',1440,900],['mobile',390,844]]) {
      const context = await browser.newContext({viewport:{width,height}});
      const page = await context.newPage();
      const errors = [], calls = [];
      page.on('pageerror', error => errors.push(error.message));
      const baseline = await (await context.request.get('http://127.0.0.1:8770/api/status')).json();
      baseline.robot.bridge_status = 'ready';
      baseline.robot.motion = null;
      baseline.robot.enabled_latch = 'unknown';
      const g = baseline.robot.gripper;
      g.enabled_latch = 'unknown'; g.history = []; g.last_transaction = null; g.busy = false;
      g.faulted = false; g.current_a = 0.2; g.remaining_ms = null;
      await page.route('**/api/**', async route => {
        const request = route.request();
        const pathname = new URL(request.url()).pathname;
        if (pathname === '/api/status') return route.fulfill({json:baseline});
        if (request.method() === 'GET') return route.continue();
        // Block every other write, including emergency, robot, recording and
        // camera endpoints, so UI mistakes cannot escape to hardware.
        if (!pathname.startsWith('/api/robot/actions/gripper-')) {
          calls.push({unexpected:pathname});
          return route.fulfill({status:409,json:{error:{message:'Blocked by offline UI test'}}});
        }
        const action = pathname.split('gripper-')[1];
        const body = request.postDataJSON();
        calls.push({action,body});
        const commands = {enable:'!HAND_EN',open:'!HAND_O',close:'!HAND_C',disable:'!HAND_DIS',current:`!HAND_I ${body.current}`};
        if (action === 'enable') { g.enabled_latch = 'confirmed'; g.remaining_ms = 3000; }
        if (action === 'disable') { g.enabled_latch = 'disabled'; g.remaining_ms = null; }
        if (action === 'current') g.current_a = body.current;
        const raw = {enable:'ok hand enable',open:'ok hand open',close:'ok hand close',disable:'ok hand disable',current:`ok hand current ${body.current}`}[action];
        const transaction = {action,command:commands[action],at_ms:Date.now(),source:'operator',acknowledged:true,raw_response:raw};
        g.last_transaction = transaction; g.history.push(transaction);
        return route.fulfill({json:{acknowledged:true,transaction,gripper:g}});
      });
      await page.goto('http://127.0.0.1:8770/', {waitUntil:'domcontentloaded'});
      await page.waitForFunction(() => document.querySelector('#gripper-enable').disabled === false);
      assert.equal(await page.locator('#gripper-open').isDisabled(), true);
      assert.equal(await page.locator('#gripper-disable').isEnabled(), true);
      await page.locator('#gripper-current').fill('2');
      assert.equal(await page.locator('#gripper-current-send').isDisabled(), true);
      await page.locator('#gripper-current').fill('0.10');
      assert.equal(await page.locator('#gripper-enable').isDisabled(), true);
      await page.locator('#gripper-current-send').click();
      await page.waitForFunction(() => document.querySelector('#gripper-enable').disabled === false);
      await page.locator('#gripper-enable').click();
      await page.waitForFunction(() => document.querySelector('#gripper-open').disabled === false);
      await page.locator('#gripper-open').click();
      await page.waitForFunction(() => document.querySelector('#gripper-replies').textContent.includes('ok hand open'));
      assert.equal(await page.locator('#gripper-open').isDisabled(), true);
      await page.locator('#gripper-disable').click();
      await page.waitForFunction(() => document.querySelector('#gripper-state').textContent === '夹爪已失能');
      assert.deepEqual(calls.map(item => item.action), ['current','enable','open','disable']);
      assert.deepEqual(calls[2].body, {duration_ms:500});
      assert.equal(await page.getByRole('button', {name:'位置控制 · 待验证'}).isDisabled(), true);
      assert.equal(await page.getByRole('button', {name:'自动校准 · 待验证'}).isDisabled(), true);
      const layout = await page.locator('.gripper-band').evaluate(section => ({
        horizontalOverflow:document.documentElement.scrollWidth > document.documentElement.clientWidth,
        clippedButtons:[...section.querySelectorAll('button')].filter(b => b.scrollWidth > b.clientWidth + 1).map(b => b.textContent),
        controlsOverlap:[...section.querySelectorAll('.gripper-controls > *')].some((item,i,list) => list.slice(i+1).some(other => {
          const a=item.getBoundingClientRect(), b=other.getBoundingClientRect();
          return Math.min(a.right,b.right) > Math.max(a.left,b.left) + 1 && Math.min(a.bottom,b.bottom) > Math.max(a.top,b.top) + 1;
        }))
      }));
      await page.locator('.gripper-band').screenshot({path:path.join('runtime',`gripper-${name}.png`)});
      assert.deepEqual(errors, []);
      assert.equal(layout.horizontalOverflow, false);
      assert.equal(layout.controlsOverlap, false);
      assert.deepEqual(layout.clippedButtons, []);
      reports.push({name,calls,layout,errors});
      await context.close();
    }
  } finally { await browser.close(); }
  console.log(JSON.stringify(reports,null,2));
})().catch(error => { console.error(error); process.exitCode=1; });
