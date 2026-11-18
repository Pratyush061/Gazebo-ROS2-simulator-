import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_FILES } from './defaultFiles';
import {
  DEFAULT_BRIDGE_YAML_PATH,
  DEFAULT_LAUNCH_PATH,
  DEFAULT_ROBOT_PATH,
  DEFAULT_WORLD_PATH,
  DIRS,
  FILE_PATHS,
  LEGACY_PATH_MAP,
  WORKSPACE_ROOT,
  baseNameOf,
  buildExplorerTree,
  dirOf,
  extOf,
  isDirectory,
  isRobotModelPath,
  isWorldPath,
  listDirectory,
  resolveRelativeDir,
  resolveWorkspacePath,
  withDerivedFolder,
} from './ros2Workspace';

const paths = DEFAULT_FILES.map((f) => f.path);

test('every workspace file path is unique, relative and folder-consistent', () => {
  assert.equal(new Set(paths).size, paths.length, 'duplicate file paths');

  for (const file of DEFAULT_FILES) {
    assert.ok(!file.path.startsWith('/'), `${file.path} must be workspace-relative`);
    assert.ok(!file.path.includes('..'), `${file.path} must not escape the workspace`);
    assert.equal(
      file.folder,
      dirOf(file.path),
      `${file.path} has folder "${file.folder}" but lives in "${dirOf(file.path)}"`,
    );
  }
});

test('layout follows the ROS 2 package convention (package.xml + CMakeLists per package)', () => {
  const packages = [DIRS.descriptionUrdf.split('/').slice(0, 3).join('/'), DIRS.gazeboWorlds.split('/').slice(0, 3).join('/')];
  for (const pkg of packages) {
    assert.ok(paths.includes(`${pkg}/package.xml`), `${pkg} is missing package.xml`);
    assert.ok(paths.includes(`${pkg}/CMakeLists.txt`), `${pkg} is missing CMakeLists.txt`);
  }
});

test('robot models live in the description package, worlds in the gazebo package', () => {
  const models = DEFAULT_FILES.filter((f) => isRobotModelPath(f.path));
  const worlds = DEFAULT_FILES.filter((f) => isWorldPath(f.path));

  assert.ok(models.length >= 3, 'expected urdf + xacro models');
  assert.ok(worlds.length >= 4, 'expected the shipped SDF worlds');

  for (const model of models) {
    assert.ok(
      model.path.startsWith(`${DIRS.descriptionUrdf}/`),
      `${model.path} is outside ${DIRS.descriptionUrdf}`,
    );
  }
  for (const world of worlds) {
    assert.ok(
      world.path.startsWith(`${DIRS.gazeboWorlds}/`),
      `${world.path} is outside ${DIRS.gazeboWorlds}`,
    );
  }
});

test('xacro, launch, config and scripts land in their standard directories', () => {
  assert.ok(paths.includes(FILE_PATHS.robotXacro), 'missing robot xacro');
  assert.ok(paths.includes(FILE_PATHS.inertiaXacro), 'missing shared inertia xacro');
  assert.ok(paths.includes(FILE_PATHS.displayLaunch), 'missing display launch');
  assert.ok(paths.includes(FILE_PATHS.gzSimLaunch), 'missing gz_sim launch');
  assert.ok(paths.includes(FILE_PATHS.rvizConfig), 'missing RViz config');
  assert.ok(paths.includes(FILE_PATHS.bridgeYaml), 'missing bridge YAML');

  assert.equal(extOf(FILE_PATHS.robotXacro), '.xacro');
  assert.equal(extOf(FILE_PATHS.inertiaXacro), '.xacro');
  assert.equal(extOf(FILE_PATHS.displayLaunch), '.py');
  assert.equal(extOf(FILE_PATHS.rvizConfig), '.rviz');
  assert.equal(baseNameOf(FILE_PATHS.bridgeYaml), 'ros_gz_bridge.yaml');
});

test('default entry points point at files that exist', () => {
  for (const [label, path] of [
    ['robot', DEFAULT_ROBOT_PATH],
    ['world', DEFAULT_WORLD_PATH],
    ['bridge yaml', DEFAULT_BRIDGE_YAML_PATH],
    ['launch', DEFAULT_LAUNCH_PATH],
  ] as const) {
    assert.ok(paths.includes(path), `default ${label} path ${path} is missing`);
  }
  assert.ok(DEFAULT_ROBOT_PATH.startsWith(`${WORKSPACE_ROOT}/src/`));
});

test('legacy paths from the flat layout still resolve to real files', () => {
  for (const [legacy, current] of Object.entries(LEGACY_PATH_MAP)) {
    assert.ok(paths.includes(current), `${legacy} maps to missing ${current}`);
    assert.equal(resolveWorkspacePath(legacy), current);
    assert.equal(resolveWorkspacePath(current), current);
  }
  assert.equal(resolveWorkspacePath('unknown/file.sdf'), 'unknown/file.sdf');
});

test('explorer tree nests packages exactly like the folder layout', () => {
  const tree = buildExplorerTree(DEFAULT_FILES);
  assert.equal(tree.path, '', 'root node is the workspace');
  assert.equal(tree.children.length, 1, 'only ros2_ws sits at the top');
  assert.equal(tree.children[0].name, 'ros2_ws');

  const src = tree.children[0].children[0];
  assert.equal(src.name, 'src');
  assert.deepEqual(
    src.children.map((c) => c.name),
    ['my_robot_description', 'my_robot_gazebo'],
    'packages are sorted and directly under src/',
  );

  const description = src.children[0];
  assert.deepEqual(
    description.children.map((c) => c.name),
    ['config', 'launch', 'urdf'],
  );
  assert.ok(
    description.files.some((f) => f.path === FILE_PATHS.descriptionPackageXml),
    'package.xml sits directly in the package root',
  );
});

test('listDirectory walks one level and resolveRelativeDir handles ..', () => {
  const root = listDirectory(DEFAULT_FILES, '');
  assert.deepEqual(root.dirs, ['ros2_ws']);
  assert.equal(root.files.length, 0);

  const urdf = listDirectory(DEFAULT_FILES, DIRS.descriptionUrdf);
  assert.deepEqual(urdf.dirs, [DIRS.descriptionXacroCommon]);
  assert.deepEqual(
    urdf.files.map((f) => f.name).sort(),
    ['diff_drive_rover.urdf', 'my_robot.urdf.xacro', 'simple_manipulator.urdf'],
  );

  assert.ok(isDirectory(DEFAULT_FILES, DIRS.gazeboWorlds));
  assert.ok(isDirectory(DEFAULT_FILES, WORKSPACE_ROOT));
  assert.ok(!isDirectory(DEFAULT_FILES, 'ros2_ws/src/nope'));

  assert.equal(resolveRelativeDir(DIRS.descriptionUrdf, '..'), DIRS.descriptionUrdf.split('/').slice(0, -1).join('/'));
  assert.equal(resolveRelativeDir('', 'ros2_ws/src'), 'ros2_ws/src');
  assert.equal(resolveRelativeDir('ros2_ws/src', '../..'), '');
});

test('withDerivedFolder keeps folder in sync with path', () => {
  const normalised = withDerivedFolder({ path: 'a/b/c.txt', folder: 'wrong' });
  assert.equal(normalised.folder, 'a/b');
  assert.equal(dirOf('top.txt'), '');
});
