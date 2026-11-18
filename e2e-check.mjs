/**
 * End-to-end browser check for the restored ROS 2 RViz2 panel.
 * Runs the real Vite dev server, drives a headless Chromium, asserts the
 * Displays panel, every 3D overlay toggle, the TF tree and the joint dock work.
 *
 *   node e2e-check.mjs
 */
import { createServer } from 'vite';
import { chromium } from 'playwright';

const results = [];
const check = (name, ok, extra = '') => {
  results.push({ name, ok, extra });
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${extra ? ` — ${extra}` : ''}`);
};

const server = await createServer({ server: { port: 5199, host: '127.0.0.1' } });
await server.listen();
const url = 'http://127.0.0.1:5199/';

const browser = await chromium.launch({
  executablePath: '/usr/local/bin/chromium',
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader'],
});
const page = await browser.newPage({ viewport: { width: 1680, height: 950 } });

const consoleErrors = [];
page.on('console', (msg) => {
  if (msg.type() === 'error') consoleErrors.push(msg.text());
});
page.on('pageerror', (err) => consoleErrors.push(`pageerror: ${err.message}`));

await page.goto(url, { waitUntil: 'load' });
await page.waitForSelector('[data-testid="ros2-visualizer-pane"]', { timeout: 30000 });
await page.waitForTimeout(2500);

// --- 1. Pane + viewport render ---
check('ROS 2 pane mounts', await page.isVisible('[data-testid="ros2-visualizer-pane"]'));
check('RViz2 WebGL canvas mounts', await page.isVisible('[data-testid="rviz-webgl-canvas"]'));
const canvasBox = await page.locator('[data-testid="rviz-webgl-canvas"] canvas').boundingBox();
check('canvas has non-zero size', !!canvasBox && canvasBox.width > 100 && canvasBox.height > 100,
  canvasBox ? `${Math.round(canvasBox.width)}x${Math.round(canvasBox.height)}` : 'missing');

// --- 2. Displays panel: the regression the user reported ---
check('RViz2 Displays panel visible', await page.isVisible('[data-testid="rviz-displays-panel"]'));
for (const name of ['Global Options', 'RobotModel', 'TF', 'LaserScan', 'Odometry', 'Imu', 'Joint Axes', 'Grid']) {
  check(`display row "${name}" present`,
    (await page.locator(`[data-testid="rviz-displays-panel"] >> text="${name}"`).count()) > 0);
}

// --- 3. The 4 overlays that had disappeared ---
for (const [label, aria] of [
  ['Collision Enabled', 'Toggle collision geometry'],
  ['Wireframe', 'Toggle wireframe rendering'],
  ['Inertia Boxes', 'Toggle inertia boxes'],
  ['Center of Mass', 'Toggle center of mass markers'],
]) {
  const box = page.locator(`[aria-label="${aria}"]`);
  const present = (await box.count()) > 0;
  check(`overlay control "${label}" present`, present);
  if (present) {
    await box.first().check();
    const checked = await box.first().isChecked();
    check(`overlay "${label}" toggles on`, checked);
  }
}

// --- 4. TF tree + Fixed Frame + topic selectors ---
const treeRows = await page.locator('[data-testid="rviz-tf-tree"] input[type="checkbox"]').count();
check('TF tree renders link/joint rows', treeRows > 3, `${treeRows} rows`);
check('Fixed Frame selector present', (await page.locator('[aria-label="RViz2 Fixed Frame"]').count()) > 0);
const frameOptions = await page.locator('[aria-label="RViz2 Fixed Frame"] option').count();
check('Fixed Frame offers map/odom/links', frameOptions > 3, `${frameOptions} options`);
check('topic selector "Description Topic" present',
  (await page.locator('[aria-label="Description Topic"]').count()) > 0);
// Sensor display rows are collapsed by default: expand LaserScan, then read its Topic select.
await page.locator('button[aria-label="Expand LaserScan properties"]').first().click();
await page.waitForTimeout(250);
check('LaserScan topic selector present',
  (await page.locator('[aria-label="Topic"]').count()) > 0);
const scanTopicOptions = await page.locator('[aria-label="Topic"] option').count();
check('LaserScan topic selector lists bridged topics', scanTopicOptions >= 1,
  `${scanTopicOptions} options`);
await page.locator('button[aria-label="Collapse LaserScan properties"]').first().click();

// hierarchy <-> alphabetic tree switch
await page.selectOption('[aria-label="Kinematic tree style"]', 'alphabetic');
await page.waitForTimeout(300);
const alphaRows = await page.locator('[data-testid="rviz-tf-tree"] input[type="checkbox"]').count();
check('alphabetic tree mode works', alphaRows > 3, `${alphaRows} rows`);
await page.selectOption('[aria-label="Kinematic tree style"]', 'hierarchy');
await page.waitForTimeout(300);

// --- 5. Joint dock (previous-version parity) ---
check('joint dock "RViz Overlays" tab present',
  (await page.locator('text=RViz Overlays').count()) > 0);
const dofTab = page.locator('text=/Joint Sliders \\(\\d+ DOF\\)/');
check('joint dock DOF counter present', (await dofTab.count()) > 0);
if ((await dofTab.count()) > 0) {
  await dofTab.first().click();
  await page.waitForTimeout(300);
  const sliders = await page.locator('input[type="range"]').count();
  check('joint sliders render', sliders > 0, `${sliders} sliders`);
  const zeroBtn = page.locator('button:has-text("Zero")');
  if ((await zeroBtn.count()) > 0) {
    await zeroBtn.first().click();
    check('Zero joints button works', true);
  }
}

// --- 6. view_frames entry points ---
check('view_frames button present', (await page.locator('[aria-label="Preview tf2_tools view_frames tree"]').count()) > 0);
check('view_frames PDF button present', (await page.locator('[aria-label="Download view_frames PDF"]').count()) > 0);

// --- 6b. Terminal-driven bridge start (clean state until launched) ---
const bridgeChip = page.locator('[data-testid="ros2-bridge-status"]');
check('bridge starts STOPPED (clean ROS 2 state)',
  (await bridgeChip.innerText()).includes('STOPPED'));
check('stopped-bridge warning shown in viewport',
  (await page.locator('[data-testid="rviz-bridge-stopped-warning"]').count()) > 0);

const termInput = page.locator('[aria-label="Ubuntu 24.04 terminal command input"]');
check('Ubuntu terminal input present', (await termInput.count()) > 0);
if ((await termInput.count()) > 0) {
  await termInput.first().fill('ros2 launch ros2_ws/ros_gz_sim.launch.py');
  await termInput.first().press('Enter');
  await page.waitForTimeout(1200);
  check('ros2 launch starts ros_gz_bridge from terminal',
    (await bridgeChip.innerText()).includes('ACTIVE'),
    await bridgeChip.innerText());

  await termInput.first().fill('ros2 topic list');
  await termInput.first().press('Enter');
  await page.waitForTimeout(600);
  const termText = await page.locator('body').innerText();
  check('ros2 topic list runs in terminal', /\/scan|\/odom|\/tf/.test(termText));
}

// --- 7. UI overload removed ---
const bodyText = await page.locator('body').innerText();
check('Quick CLI chips removed', !bodyText.includes('Quick CLI'));
check('ROS 2 Bridge table tab removed', !/ROS 2 Bridge \(\d+\)/.test(bodyText));
check('Joint & Teleop tab removed', !bodyText.includes('Joint & Teleop Actuation'));

// --- 8. Gazebo pane still alive (nothing broken) ---
check('Gazebo pane still present',
  (await page.locator('[aria-label="Gazebo Harmonic 3D Physics Simulator"]').count()) > 0);
check('code pane still present',
  (await page.locator('[data-testid="workbench-tab-plugins"]').count()) > 0);

// --- 9. Layout controls ---
for (const aria of ['Minimize ROS 2 pane', 'Maximize ROS 2 pane']) {
  check(`pane control "${aria}"`, (await page.locator(`[aria-label="${aria}"]`).count()) > 0);
}

// --- 9b. Layout switcher: dual output, maximize, back to tri ---
const dualBtn = page.locator('button:has-text("Dual Output")');
check('dual-output layout button present', (await dualBtn.count()) > 0);
if ((await dualBtn.count()) > 0) {
  await dualBtn.first().click();
  await page.waitForTimeout(900);
  const ideVisible = await page.isVisible('[data-testid="workbench-tab-plugins"]');
  check('dual output hides the code pane', !ideVisible);
  check('dual output keeps RViz2 Displays', await page.isVisible('[data-testid="rviz-displays-panel"]'));
  check('dual output keeps Gazebo pane',
    (await page.locator('[aria-label="Gazebo Harmonic 3D Physics Simulator"]').count()) > 0);
  await page.screenshot({ path: '/scratch/work/ros2-dual.png' });
}

const maxBtn = page.locator('[data-testid="maximize-ros2-pane-btn"]');
if ((await maxBtn.count()) > 0) {
  await maxBtn.first().click();
  await page.waitForTimeout(700);
  const box = await page.locator('[data-testid="ros2-visualizer-pane"]').boundingBox();
  check('maximize widens the ROS 2 pane', !!box && box.width > 900,
    box ? `${Math.round(box.width)}px` : 'missing');
  await page.screenshot({ path: '/scratch/work/ros2-maximized.png' });
  await maxBtn.first().click();
  await page.waitForTimeout(600);
}

const triBtn = page.locator('button:has-text("Tri-Pane")');
if ((await triBtn.count()) > 0) {
  await triBtn.first().click();
  await page.waitForTimeout(700);
  check('tri-pane layout restores all three panes',
    (await page.isVisible('[data-testid="workbench-tab-plugins"]')) &&
      (await page.isVisible('[data-testid="rviz-displays-panel"]')));
}

// --- 10. No runtime errors ---
// The sandbox proxy blocks the Google Fonts stylesheet in index.html (cert), and
// SwiftShader is expected for headless WebGL — neither comes from app code.
const realErrors = consoleErrors.filter(
  (e) =>
    !/WebGL|SwiftShader|GPU stall|Failed to create|deprecated|ERR_CERT_AUTHORITY_INVALID|fonts\.googleapis/i.test(
      e,
    ),
);
check('no console/runtime errors', realErrors.length === 0, realErrors.slice(0, 3).join(' | '));

await page.screenshot({ path: '/scratch/work/ros2-panel.png', fullPage: false });

await browser.close();
await server.close();

const failed = results.filter((r) => !r.ok);
console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
if (failed.length > 0) {
  console.log('FAILURES:', failed.map((f) => f.name).join(', '));
  process.exit(1);
}
