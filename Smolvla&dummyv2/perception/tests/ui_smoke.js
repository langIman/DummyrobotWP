const { chromium } = require(process.env.PERCEPTION_PLAYWRIGHT_MODULE || 'playwright');
const path = require('path');

const cases = [
  { name: 'desktop', viewport: { width: 1440, height: 900 } },
  { name: 'mobile', viewport: { width: 390, height: 844 } },
];

(async () => {
  const browser = await chromium.launch({
    headless: true,
    executablePath: 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  });
  const results = [];
  try {
    for (const testCase of cases) {
      const context = await browser.newContext({ viewport: testCase.viewport });
      const page = await context.newPage();
      const errors = [];
      page.on('pageerror', error => errors.push(`pageerror: ${error.message}`));
      page.on('console', message => {
        if (message.type() === 'error') errors.push(`console: ${message.text()}`);
      });
      await page.goto('http://127.0.0.1:8770/', { waitUntil: 'domcontentloaded' });
      await page.waitForTimeout(4000);
      const layout = await page.evaluate(() => ({
        viewportWidth: document.documentElement.clientWidth,
        pageWidth: document.documentElement.scrollWidth,
        title: document.querySelector('h1')?.textContent,
        service: document.querySelector('#service-state')?.textContent.trim(),
        oak: document.querySelector('#oak-summary')?.textContent.trim(),
        wrist: document.querySelector('#wrist-summary')?.textContent.trim(),
        overflowingButtons: [...document.querySelectorAll('button')]
          .filter(button => button.scrollWidth > button.clientWidth + 1)
          .map(button => button.textContent.trim()),
      }));
      await page.evaluate(() => {
        const snapshots = {
          'stream-oak-rgb': '/api/frames/oak-rgb',
          'stream-oak-depth': '/api/frames/oak-depth',
        };
        const wrist = document.getElementById('stream-wrist');
        wrist.onerror = null;
        wrist.removeAttribute('src');
        for (const [id, source] of Object.entries(snapshots)) {
          const image = document.getElementById(id);
          image.onerror = null;
          image.src = `${source}?screenshot=${Date.now()}`;
        }
      });
      await page.waitForTimeout(500);
      await page.screenshot({
        path: path.join('runtime', `dashboard-${testCase.name}.png`),
        fullPage: true,
      });
      results.push({ name: testCase.name, errors, layout });
      await context.close();
    }
  } finally {
    await browser.close();
  }
  console.log(JSON.stringify(results, null, 2));
  const failed = results.some(result =>
    result.errors.length ||
    result.layout.pageWidth > result.layout.viewportWidth ||
    result.layout.overflowingButtons.length
  );
  process.exitCode = failed ? 1 : 0;
})();
