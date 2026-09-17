// All writes are mocked before navigation. Never sends commands to hardware.
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
      const errors = [], calls = [];
      page.on('pageerror', error => errors.push(error.message));
      const baseline = await (await context.request.get('http://127.0.0.1:8770/api/status')).json();
      Object.assign(baseline.robot, {bridge_status:'ready', motion:null, enabled_latch:'unknown',
        raw_console:{active:false,busy:false,history:[]}});
      Object.assign(baseline.robot.gripper, {enabled_latch:'unknown',busy:false,faulted:false,
        remaining_ms:null,history:[],last_transaction:null});
      await page.route('**/api/**', async route => {
        const request = route.request(), pathname = new URL(request.url()).pathname;
        if (pathname === '/api/status') return route.fulfill({json:baseline});
        if (request.method() === 'GET') return route.continue();
        calls.push({pathname,body:request.postDataJSON()});
        if (pathname !== '/api/robot/command') {
          return route.fulfill({status:409,json:{error:{message:'Blocked by offline UI test'}}});
        }
        const body = request.postDataJSON();
        const bridge = {sent:true,bytes_written:body.command.length+1,
          raw_response:'ok\r\n<script>window.bad=true</script>',pending_response:'old reply'};
        const transaction = {...body,...bridge,bridge,at_ms:Date.now(),execution_status:'unknown'};
        if (body.command === 'FAIL') {
          bridge.error = {code:'read_failed'};
          transaction.error = {code:'raw_transport_unconfirmed',message:'发送或读取未确认；不要盲目重发。'};
        }
        baseline.robot.raw_console.active = true;
        baseline.robot.raw_console.history.push(transaction);
        return route.fulfill({status:transaction.error ? 503 : 200,json:transaction});
      });
      await page.goto('http://127.0.0.1:8770/', {waitUntil:'domcontentloaded'});
      await page.waitForFunction(() => document.querySelector('#raw-mode').textContent === '按钮控制模式');
      assert.equal(calls.length,0);
      assert.equal(await page.locator('#raw-send').isDisabled(),true);
      await page.locator('#raw-command').fill('打开');
      assert.equal(await page.locator('#raw-send').isDisabled(),true);
      await page.locator('#raw-command').fill('X'.repeat(63));
      assert.equal(await page.locator('#raw-send').isDisabled(),true);
      await page.locator('#raw-command').fill('!HAND_ZERO  ');
      await page.locator('#raw-command').press('Enter');
      assert.equal(calls.length,0);
      await page.locator('#raw-timeout').fill('2001');
      assert.equal(await page.locator('#raw-send').isDisabled(),true);
      await page.locator('#raw-timeout').fill('500');
      await page.locator('#raw-send').click();
      await page.waitForFunction(() => document.querySelector('#raw-replies').textContent.includes('old reply'));
      assert.deepEqual(calls,[{pathname:'/api/robot/command',body:{command:'!HAND_ZERO  ',read_timeout_ms:500}}]);
      assert.equal(await page.evaluate(() => window.bad),undefined);
      assert.equal(await page.locator('#enable').isDisabled(),true);
      assert.equal(await page.locator('#gripper-enable').isDisabled(),true);
      assert.equal(await page.locator('#emergency').isEnabled(),true);
      assert.match(await page.locator('#raw-message').textContent(),/动作是否完成未知/);
      await page.locator('#raw-command').fill('FAIL');
      await page.locator('#raw-send').click();
      await page.waitForFunction(() => document.querySelector('#raw-message').textContent.includes('不要盲目重发'));
      assert.equal(calls.length,2);
      await page.reload({waitUntil:'domcontentloaded'});
      await page.waitForFunction(() => document.querySelector('#raw-replies').textContent.includes('read_failed'));
      assert.equal(calls.length,2);
      const layout = await page.locator('.raw-band').evaluate(section => ({
        overflow:document.documentElement.scrollWidth > document.documentElement.clientWidth,
        overlap:[...section.querySelectorAll('.raw-controls > *')].some((item,i,list) => list.slice(i+1).some(other => {
          const a=item.getBoundingClientRect(),b=other.getBoundingClientRect();
          return Math.min(a.right,b.right)>Math.max(a.left,b.left)+1 && Math.min(a.bottom,b.bottom)>Math.max(a.top,b.top)+1;
        }))
      }));
      await page.locator('.raw-band').screenshot({path:path.join('runtime',`raw-${name}.png`)});
      assert.deepEqual(errors,[]);
      assert.deepEqual(layout,{overflow:false,overlap:false});
      reports.push({name,calls: calls.length,layout,errors});
      await context.close();
    }
  } finally { await browser.close(); }
  console.log(JSON.stringify(reports,null,2));
})().catch(error => {console.error(error);process.exitCode=1;});
