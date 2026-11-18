import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_FILES } from './defaultFiles';
import {
  DEFAULT_BRIDGE_YAML_PATH,
  DEFAULT_LAUNCH_PATH,
  DEFAULT_ROBOT_PATH,
  DIRS,
  FILE_PATHS,
} from './ros2Workspace';
import { compileSdfOrUrdf } from '../utils/sdfCompiler';
import { compileUrdf, computeForwardKinematics } from '../urdf/urdfCompiler';
import { resolveXacroIncludes } from '../urdf/xacroIncludes';
import { executeUbuntuCommand } from '../utils/ubuntuTerminal';
import {
  createDefaultRos2RuntimeState,
  parseRosGzBridgeYaml,
  serializeRosGzBridgeYaml,
} from '../utils/rosGzBridgeEngine';

const byPath = (path: string) => DEFAULT_FILES.find((f) => f.path === path)!;

test('the shipped xacro robot resolves its includes and compiles to a full kinematic tree', () => {
  const xacroFile = byPath(FILE_PATHS.robotXacro);
  const resolved = resolveXacroIncludes(xacroFile.content, xacroFile.path, DEFAULT_FILES);

  assert.deepEqual(resolved.diagnostics, [], 'no include diagnostics');
  assert.deepEqual(resolved.resolved, [FILE_PATHS.inertiaXacro]);
  assert.ok(!resolved.xml.includes('<xacro:include'), 'includes are inlined');

  const model = compileUrdf(resolved.xml);
  assert.equal(model.valid, true, JSON.stringify(model.diagnostics.filter((d) => d.severity === 'error')));
  assert.equal(model.robotName, 'my_robot');
  assert.equal(Object.keys(model.links).length, 7);
  assert.equal(Object.keys(model.joints).length, 6);

  // The wheel macro produced four wheels with mass properties from the shared macros.
  for (const wheel of ['front_left', 'front_right', 'rear_left', 'rear_right']) {
    const link = model.links[`${wheel}_wheel_link`];
    assert.ok(link, `missing ${wheel}_wheel_link`);
    assert.ok(link.inertial, `${wheel}_wheel_link has no inertial from the shared macro`);
    assert.ok(link.inertial!.mass > 0);
  }

  const fk = computeForwardKinematics(model, {});
  assert.ok(fk['lidar_link'], 'forward kinematics reaches the lidar link');
});

test('the macro library file is reported as valid, not as a broken robot', () => {
  const lib = byPath(FILE_PATHS.inertiaXacro);
  const res = compileSdfOrUrdf(lib.content, lib.path, DEFAULT_FILES);
  assert.equal(res.isValid, true);
  assert.equal(
    res.diagnostics.filter((d) => d.severity === 'error').length,
    0,
    JSON.stringify(res.diagnostics),
  );
});

test('package manifests and CMake files are valid workspace files, not broken worlds', () => {
  for (const path of [FILE_PATHS.descriptionPackageXml, FILE_PATHS.gazeboCMake, FILE_PATHS.rvizConfig]) {
    const file = byPath(path);
    const res = compileSdfOrUrdf(file.content, file.path, DEFAULT_FILES);
    assert.equal(res.isValid, true, `${path} should not raise errors`);
    assert.equal(res.world, null);
  }
});

test('every robot model compiles into a Gazebo-convertible world through the SDF compiler', () => {
  for (const file of DEFAULT_FILES.filter((f) => f.path.includes('/urdf/'))) {
    const res = compileSdfOrUrdf(file.content, file.path, DEFAULT_FILES);
    assert.equal(res.isValid, true, `${file.path}: ${JSON.stringify(res.diagnostics.filter((d) => d.severity === 'error'))}`);
    if (file.path.endsWith('.xacro') && file.path.includes('common/')) continue;
    assert.ok(res.world, `${file.path} produced no world`);
    assert.ok(res.world!.models.length >= 1);
  }
});

test('launch files reference files that actually exist in the workspace', () => {
  const gzLaunch = byPath(DEFAULT_LAUNCH_PATH).content;
  const displayLaunch = byPath(FILE_PATHS.displayLaunch).content;

  // gz_sim.launch.py loads the bridge config from the gazebo package share dir.
  assert.match(gzLaunch, /get_package_share_directory\('my_robot_gazebo'\)/);
  assert.match(gzLaunch, /'ros_gz_bridge\.yaml'/);
  assert.ok(DEFAULT_BRIDGE_YAML_PATH.startsWith(DIRS.gazeboConfig));

  // display.launch.py drives RViz from the description package.
  assert.match(displayLaunch, /my_robot_description/);
  assert.match(displayLaunch, /my_robot\.urdf\.xacro/);
  assert.match(displayLaunch, /urdf_config\.rviz/);
  assert.ok(byPath(FILE_PATHS.rvizConfig).content.includes('Visualization Manager'));
});

test('bridge YAML at its workspace path round-trips through the bridge engine', () => {
  const yaml = byPath(DEFAULT_BRIDGE_YAML_PATH).content;
  const { rules, diagnostics } = parseRosGzBridgeYaml(yaml);

  assert.equal(diagnostics.filter((d) => d.severity === 'error').length, 0);
  assert.ok(rules.length >= 6, `expected the full topic set, got ${rules.length}`);

  const serialized = serializeRosGzBridgeYaml(rules);
  const reparsed = parseRosGzBridgeYaml(serialized);
  assert.equal(reparsed.rules.length, rules.length);
  assert.deepEqual(
    reparsed.rules.map((r) => r.rosTopic).sort(),
    rules.map((r) => r.rosTopic).sort(),
  );
});

test('default world compiles and the bridge engine builds rules for it', () => {
  const world = compileSdfOrUrdf(
    byPath(DIRS.gazeboWorlds + '/empty.sdf').content,
    `${DIRS.gazeboWorlds}/empty.sdf`,
    DEFAULT_FILES,
  );
  assert.equal(world.isValid, true);
  assert.ok(world.world);

  const state = createDefaultRos2RuntimeState(world.world);
  assert.ok(state.bridgeRules.length > 0, 'bridge rules generated from the world');
  assert.equal(state.isBridgeRunning, false, 'bridge starts unconnected');
});

test('terminal walks the nested workspace and checks a world in place', () => {
  const ctx = {
    cwd: '/home/ubuntu/workspace',
    files: [...DEFAULT_FILES],
    activeWorld: null,
    isSimRunning: false,
    simTime: 0,
  };

  const root = executeUbuntuCommand('ls', ctx);
  assert.equal(root.output.trim(), 'ros2_ws/');

  const src = executeUbuntuCommand('ls ros2_ws/src', ctx);
  assert.ok(src.output.includes('my_robot_description/'));
  assert.ok(src.output.includes('my_robot_gazebo/'));

  const urdf = executeUbuntuCommand(`ls ${DIRS.descriptionUrdf}`, ctx);
  assert.ok(urdf.output.includes('my_robot.urdf.xacro'));
  assert.ok(urdf.output.includes('diff_drive_rover.urdf'));

  const cd = executeUbuntuCommand(`cd ${DIRS.gazeboWorlds}`, ctx);
  assert.equal(cd.newCwd, `/home/ubuntu/workspace/${DIRS.gazeboWorlds}`);

  const up = executeUbuntuCommand('cd ..', { ...ctx, cwd: `/home/ubuntu/workspace/${DIRS.gazeboWorlds}` });
  assert.equal(up.newCwd, '/home/ubuntu/workspace/ros2_ws/src/my_robot_gazebo');

  const bad = executeUbuntuCommand('cd nowhere', ctx);
  assert.equal(bad.isError, true);

  const check = executeUbuntuCommand(`gz sdf -k ${DEFAULT_ROBOT_PATH}`, ctx);
  assert.equal(check.isError, undefined, check.output);
  assert.ok(check.output.includes('VALID'));
});
