/**
 * Canonical ROS 2 + Gazebo workspace layout.
 *
 * This is the single source of truth for every path in the simulated file tree,
 * so the explorer, terminal, compilers and panes never hard-code a directory.
 * Layout follows the standard ROS 2 package convention:
 *
 *   ros2_ws/src/
 *     my_robot_description/          # robot model: URDF, xacro, RViz, display launch
 *       urdf/  urdf/common/  launch/  config/  package.xml  CMakeLists.txt
 *     my_robot_gazebo/               # simulation: SDF worlds, gz launch, bridge config
 *       worlds/  launch/  config/  scripts/  package.xml  CMakeLists.txt
 */
export const WORKSPACE_ROOT = 'ros2_ws';
export const WORKSPACE_SRC = `${WORKSPACE_ROOT}/src`;

export const DESCRIPTION_PKG = `${WORKSPACE_SRC}/my_robot_description`;
export const GAZEBO_PKG = `${WORKSPACE_SRC}/my_robot_gazebo`;

/** Directories, as absolute workspace paths. */
export const DIRS = {
  descriptionUrdf: `${DESCRIPTION_PKG}/urdf`,
  descriptionXacroCommon: `${DESCRIPTION_PKG}/urdf/common`,
  descriptionLaunch: `${DESCRIPTION_PKG}/launch`,
  descriptionConfig: `${DESCRIPTION_PKG}/config`,
  gazeboWorlds: `${GAZEBO_PKG}/worlds`,
  gazeboLaunch: `${GAZEBO_PKG}/launch`,
  gazeboConfig: `${GAZEBO_PKG}/config`,
  gazeboScripts: `${GAZEBO_PKG}/scripts`,
} as const;

/** Every file the simulator ships with, as absolute workspace paths. */
export const FILE_PATHS = {
  descriptionPackageXml: `${DESCRIPTION_PKG}/package.xml`,
  descriptionCMake: `${DESCRIPTION_PKG}/CMakeLists.txt`,
  diffDriveUrdf: `${DIRS.descriptionUrdf}/diff_drive_rover.urdf`,
  manipulatorUrdf: `${DIRS.descriptionUrdf}/simple_manipulator.urdf`,
  robotXacro: `${DIRS.descriptionUrdf}/my_robot.urdf.xacro`,
  inertiaXacro: `${DIRS.descriptionXacroCommon}/inertia.xacro`,
  displayLaunch: `${DIRS.descriptionLaunch}/display.launch.py`,
  rvizConfig: `${DIRS.descriptionConfig}/urdf_config.rviz`,

  gazeboPackageXml: `${GAZEBO_PKG}/package.xml`,
  gazeboCMake: `${GAZEBO_PKG}/CMakeLists.txt`,
  emptyWorld: `${DIRS.gazeboWorlds}/empty.sdf`,
  r2d2World: `${DIRS.gazeboWorlds}/r2d2_starwars.sdf`,
  armWorld: `${DIRS.gazeboWorlds}/robotic_arm_6dof.sdf`,
  warehouseWorld: `${DIRS.gazeboWorlds}/rover_4wd_warehouse.sdf`,
  gzSimLaunch: `${DIRS.gazeboLaunch}/gz_sim.launch.py`,
  bridgeYaml: `${DIRS.gazeboConfig}/ros_gz_bridge.yaml`,
  obstacleNode: `${DIRS.gazeboScripts}/obstacle_avoidance_node.py`,
} as const;

/** Default entry points used across the app. */
export const DEFAULT_ROBOT_PATH = FILE_PATHS.diffDriveUrdf;
export const DEFAULT_WORLD_PATH = FILE_PATHS.emptyWorld;
export const DEFAULT_BRIDGE_YAML_PATH = FILE_PATHS.bridgeYaml;
export const DEFAULT_LAUNCH_PATH = FILE_PATHS.gzSimLaunch;

/** Folders the explorer opens by default, root first. */
export const EXPLORER_ROOTS = [
  WORKSPACE_ROOT,
  DESCRIPTION_PKG,
  GAZEBO_PKG,
] as const;

/** Paths from the pre-ROS-2-workspace tree, kept so old links keep resolving. */
export const LEGACY_PATH_MAP: Record<string, string> = {
  'worlds/empty.sdf': FILE_PATHS.emptyWorld,
  'worlds/r2d2_starwars.sdf': FILE_PATHS.r2d2World,
  'worlds/robotic_arm_6dof.sdf': FILE_PATHS.armWorld,
  'worlds/rover_4wd_warehouse.sdf': FILE_PATHS.warehouseWorld,
  'urdf/simple_manipulator.urdf': FILE_PATHS.manipulatorUrdf,
  'urdf/diff_drive_rover.urdf': FILE_PATHS.diffDriveUrdf,
  'ros2_ws/ros_gz_bridge.yaml': FILE_PATHS.bridgeYaml,
  'ros2_ws/ros_gz_sim.launch.py': FILE_PATHS.gzSimLaunch,
  'ros2_ws/obstacle_avoidance_node.py': FILE_PATHS.obstacleNode,
};

/** Parent directory of a workspace path ('' for a top-level file). */
export function dirOf(path: string): string {
  const clean = path.replace(/\\/g, '/').replace(/^\.\//, '');
  const idx = clean.lastIndexOf('/');
  return idx === -1 ? '' : clean.slice(0, idx);
}

/** Lower-cased extension including the dot, e.g. '.urdf.xacro' -> '.xacro'. */
export function extOf(path: string): string {
  const base = path.slice(path.lastIndexOf('/') + 1);
  const idx = base.lastIndexOf('.');
  return idx === -1 ? '' : base.slice(idx).toLowerCase();
}

export function baseNameOf(path: string): string {
  return path.slice(path.lastIndexOf('/') + 1);
}

/** Resolves a possibly-legacy path to its current workspace location. */
export function resolveWorkspacePath(path: string): string {
  const clean = path.replace(/^\.\//, '');
  return LEGACY_PATH_MAP[clean] ?? clean;
}

/** Robot model sources that the URDF/xacro compiler understands. */
export function isRobotModelPath(path: string): boolean {
  const p = path.toLowerCase();
  return p.endsWith('.urdf') || p.endsWith('.xacro');
}

/** Gazebo world files. */
export function isWorldPath(path: string): boolean {
  return path.toLowerCase().endsWith('.sdf');
}

/**
 * A file is only valid if `folder` is exactly its parent directory — the
 * explorer, `ls` and file creation all rely on it. Returns a normalised copy.
 */
export function withDerivedFolder<T extends { path: string; folder?: string }>(
  file: T,
): T & { folder: string } {
  return { ...file, folder: dirOf(file.path) };
}

/** Immediate children of a directory, derived from file paths (no stored tree). */
export interface DirectoryListing {
  dirs: string[];
  files: { name: string; path: string }[];
}

function normaliseDir(dir: string): string {
  return dir.replace(/^\/+|\/+$/g, '');
}

/**
 * Lists one directory level. O(files) with a single pass, so `ls` stays cheap
 * no matter how deep the workspace gets.
 */
export function listDirectory(
  files: { path: string; name?: string }[],
  dir: string,
): DirectoryListing {
  const target = normaliseDir(dir);
  const prefix = target === '' ? '' : `${target}/`;
  const dirs = new Set<string>();
  const out: { name: string; path: string }[] = [];

  for (const file of files) {
    const parent = dirOf(file.path);
    if (parent === target) {
      out.push({ name: file.name ?? baseNameOf(file.path), path: file.path });
      continue;
    }
    if (!parent.startsWith(prefix)) continue;
    const segment = parent.slice(prefix.length).split('/')[0];
    if (segment) dirs.add(prefix + segment);
  }

  return { dirs: Array.from(dirs).sort(), files: out };
}

/** True when `dir` is the workspace root or an ancestor of some file. */
export function isDirectory(files: { path: string }[], dir: string): boolean {
  const target = normaliseDir(dir);
  if (target === '') return true;
  return files.some((f) => {
    const parent = dirOf(f.path);
    return parent === target || parent.startsWith(`${target}/`);
  });
}

/** Joins a cwd-relative target to an absolute workspace directory. */
export function resolveRelativeDir(cwd: string, target: string): string {
  if (target.startsWith('/')) return normaliseDir(target);
  const parts = normaliseDir(cwd).split('/').filter(Boolean);
  for (const segment of target.split('/')) {
    if (segment === '' || segment === '.') continue;
    if (segment === '..') parts.pop();
    else parts.push(segment);
  }
  return parts.join('/');
}

/** One node of the workspace explorer tree. */
export interface ExplorerNode<T extends { path: string } = { path: string }> {
  name: string;
  path: string;
  children: ExplorerNode<T>[];
  files: T[];
}

/**
 * Builds the nested explorer tree from file paths alone, so the folder view is
 * always a projection of the workspace layout — never a second source of truth.
 * Single pass plus one sort per directory.
 */
export function buildExplorerTree<T extends { path: string }>(
  files: T[],
): ExplorerNode<T> {
  const root: ExplorerNode<T> = {
    name: 'workspace',
    path: '',
    children: [],
    files: [],
  };
  const nodes = new Map<string, ExplorerNode<T>>([['', root]]);

  const ensureDir = (dir: string): ExplorerNode<T> => {
    const found = nodes.get(dir);
    if (found) return found;
    const node: ExplorerNode<T> = {
      name: baseNameOf(dir),
      path: dir,
      children: [],
      files: [],
    };
    ensureDir(dirOf(dir)).children.push(node);
    nodes.set(dir, node);
    return node;
  };

  for (const file of files) ensureDir(dirOf(file.path)).files.push(file);

  const sortNode = (node: ExplorerNode<T>) => {
    node.children.sort((a, b) => a.name.localeCompare(b.name));
    node.files.sort((a, b) => baseNameOf(a.path).localeCompare(baseNameOf(b.path)));
    node.children.forEach(sortNode);
  };
  sortNode(root);

  return root;
}
