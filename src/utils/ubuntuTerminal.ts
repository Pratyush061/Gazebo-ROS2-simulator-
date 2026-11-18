import {
  ParsedWorld,
  Ros2RuntimeState,
  RosGzBridgeRule,
  VirtualFile,
} from '../types/simulation';
import {
  directionToCliSymbol,
  formatParameterBridgeCliArg,
  parseParameterBridgeCliArg,
  parseRosGzBridgeYaml,
} from './rosGzBridgeEngine';
import { compileSdfOrUrdf } from './sdfCompiler';
import {
  baseNameOf,
  isDirectory,
  listDirectory,
  resolveRelativeDir,
  DEFAULT_BRIDGE_YAML_PATH,
  DEFAULT_LAUNCH_PATH,
} from '../data/ros2Workspace';

export interface TerminalCommandContext {
  cwd: string;
  files: VirtualFile[];
  activeWorld: ParsedWorld | null;
  isSimRunning: boolean;
  simTime: number;
  ros2State?: Ros2RuntimeState;
}

export interface TerminalCommandResult {
  output: string;
  isError?: boolean;
  clearScreen?: boolean;
  newCwd?: string;
  updatedFiles?: VirtualFile[];
  selectFileId?: string;
  loadAndCompileFile?: VirtualFile;
  startSimulation?: boolean;
  pauseSimulation?: boolean;
  resetSimulation?: boolean;
  cmdVelUpdate?: {
    modelName: string;
    linearX: number;
    linearY: number;
    angularZ: number;
  };
  jointUpdate?: {
    modelName: string;
    jointName: string;
    targetPosition: number;
  };
  windUpdate?: [number, number, number];
  ros2Action?: {
    setBridgeRunning?: boolean;
    replaceBridgeRules?: RosGzBridgeRule[];
    upsertBridgeRule?: RosGzBridgeRule;
    setNodeEnabled?: {
      node: keyof Ros2RuntimeState['nodes'];
      enabled: boolean;
    };
    setUseSimTime?: boolean;
    publishRosCmdVel?: {
      linearX: number;
      linearY: number;
      angularZ: number;
    };
  };
}

/**
 * Resolves a relative path from the Ubuntu terminal current working directory (`/home/ubuntu/workspace/...`)
 * into a VirtualFile path (`worlds/empty.sdf`).
 */
function resolveWorkspacePath(cwd: string, rawTarget: string): string {
  const baseRel = cwd.replace(/^\/home\/ubuntu\/workspace\/?/, '');
  const cleanTarget = rawTarget.replace(/^\/home\/ubuntu\/workspace\/?/, '').replace(/^\.\//, '');
  if (!baseRel || cleanTarget.includes('/')) {
    return cleanTarget.replace(/^\/+/, '');
  }
  return `${baseRel}/${cleanTarget}`.replace(/^\/+/, '');
}

/**
 * Executes an Ubuntu 24.04 LTS Bash or Gazebo Harmonic (`gz`) command.
 */
export function executeUbuntuCommand(
  rawInput: string,
  ctx: TerminalCommandContext
): TerminalCommandResult {
  const trimmed = rawInput.trim();
  if (!trimmed) {
    return { output: '' };
  }

  const tokens = trimmed.match(/(?:[^\s"']+|"[^"]*"|'[^']*')+/g) || [];
  const cmd = tokens[0];
  const args = tokens.slice(1).map((a) => a.replace(/^['"]|['"]$/g, ''));

  switch (cmd) {
    case 'clear':
      return { output: '', clearScreen: true };

    case 'help':
      return {
        output: [
          'Ubuntu 24.04.1 LTS (Noble Numbat) - Gazebo Harmonic CLI Environment',
          '──────────────────────────────────────────────────────────────────────────────',
          'Gazebo Harmonic (gz-sim8) Commands:',
          '  gz sim <file.sdf> [-r]       Compile and launch SDF/URDF world (-r to run immediately)',
          '  gz sdf -k <file.sdf>         Run strict SDF 1.10 / URDF schema & inertia check',
          '  gz sdf -p <file.urdf>        Convert URDF to formatted SDF 1.10 and print XML',
          '  gz topic -l                  List all active Gazebo transport topics',
          '  gz topic -i -t <topic>       Inspect publishers and message type for a topic',
          '  gz topic -t <topic> -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.5}"',
          '                               Publish live velocity command to a robot model',
          '  gz model --list              List all spawned models, entities, and poses',
          '  gz model -m <name>           Inspect links, joints, and inertia of a specific model',
          '  gz wind <vx> <vy> <vz>       Update world aerodynamic wind velocity vector (m/s)',
          '',
          'Ubuntu 24.04 File & System Commands:',
          '  ls [-la] [dir]               List directory contents in virtual workspace',
          '  cd <dir>                     Change current working directory',
          '  pwd                          Print working directory',
          '  cat <file>                   Display file contents',
          '  touch <file.sdf>             Create a new SDF file from template',
          '  mkdir <folder>               Create a virtual directory',
          '  cp <src> <dest>              Copy a file in the workspace',
          '  mv <src> <dest>              Rename or move a file in the workspace',
          '  rm <file>                    Remove a file from the workspace',
          '  grep <pattern> [file]        Search for text or XML tags across files',
          '  uname -a / lsb_release -a    Display Ubuntu 24.04 kernel & distribution info',
          '  top / ps                     Show active Gazebo physics server & GUI processes',
          '  clear                        Clear terminal output',
        ].join('\n'),
      };

    case 'pwd':
      return { output: ctx.cwd };

    case 'whoami':
      return { output: 'ubuntu' };

    case 'uname':
      return {
        output:
          'Linux gz-harmonic-studio 6.8.0-45-generic #45-Ubuntu SMP PREEMPT_DYNAMIC x86_64 GNU/Linux',
      };

    case 'lsb_release':
      return {
        output: [
          'No LSB modules are available.',
          'Distributor ID:\tUbuntu',
          'Description:\tUbuntu 24.04.1 LTS',
          'Release:\t24.04',
          'Codename:\tnoble',
        ].join('\n'),
      };

    case 'top':
    case 'ps': {
      const rtf = ctx.isSimRunning ? '100.0%' : '0.0%';
      const modelCount = ctx.activeWorld?.models.length || 0;
      return {
        output: [
          `top - ${new Date().toTimeString().slice(0, 8)} up 4:12,  1 user,  load average: 0.34, 0.28, 0.21`,
          'Tasks:  14 total,   2 running,  12 sleeping,   0 stopped,   0 zombie',
          '%Cpu(s): 14.2 us,  2.1 sy,  0.0 ni, 83.4 id,  0.3 wa,  0.0 hi,  0.0 si',
          'MiB Mem :   8192.0 total,   5412.4 free,   1640.2 used,   1139.4 buff/cache',
          '',
          '    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND',
          `   1042 ubuntu    20   0 1420840 248120  94200 R  ${ctx.isSimRunning ? '24.5' : ' 1.2'}   3.0   1:14.22 gz-sim-server (models=${modelCount}, RTF=${rtf})`,
          '   1048 ubuntu    20   0  984120 182400  78100 S   8.4   2.2   0:42.19 gz-sim-gui',
          '   1081 ubuntu    20   0  312400  44200  28400 S   0.5   0.5   0:04.11 gz-transport-router',
          '   1102 ubuntu    20   0   14280   5420   3600 S   0.0   0.1   0:00.08 bash',
        ].join('\n'),
      };
    }

    case 'ls': {
      const showLong = args.some((a) => a.startsWith('-'));
      const targetArg = args.find((a) => !a.startsWith('-'));
      const cwdRel = ctx.cwd.replace(/^\/home\/ubuntu\/workspace\/?/, '');
      const relDir = targetArg
        ? resolveRelativeDir(cwdRel, targetArg)
        : cwdRel;

      const listing = listDirectory(ctx.files, relDir);
      if (listing.dirs.length === 0 && listing.files.length === 0) {
        return {
          output: `ls: cannot access '${targetArg || relDir || '.'}': No such file or directory`,
          isError: true,
        };
      }

      const dirNames = listing.dirs.map((d) => `${baseNameOf(d)}/`);
      const fileNames = listing.files.map((f) => f.name);

      if (!showLong) {
        return { output: [...dirNames, ...fileNames].join('  ') };
      }

      const lines = [
        `total ${listing.dirs.length * 4 + listing.files.length * 4}`,
        ...listing.dirs.map(
          (d) => `drwxr-xr-x 2 ubuntu ubuntu   4096 Nov 14 09:44 ${baseNameOf(d)}/`,
        ),
        ...listing.files.map((f) => {
          const size = ctx.files.find((x) => x.path === f.path)?.content.length ?? 0;
          return `-rw-r--r-- 1 ubuntu ubuntu ${String(size).padStart(5, ' ')} Nov 14 09:44 ${f.name}`;
        }),
      ];
      return { output: lines.join('\n') };
    }

    case 'cd': {
      const target = args[0] || '';
      const cwdRel = ctx.cwd.replace(/^\/home\/ubuntu\/workspace\/?/, '');

      if (!target || target === '~' || target === '/home/ubuntu/workspace') {
        return { output: '', newCwd: '/home/ubuntu/workspace' };
      }

      const resolved = resolveRelativeDir(cwdRel, target);
      if (resolved === '') {
        return { output: '', newCwd: '/home/ubuntu/workspace' };
      }
      if (!isDirectory(ctx.files, resolved)) {
        return {
          output: `cd: no such file or directory: ${target}`,
          isError: true,
        };
      }
      return { output: '', newCwd: `/home/ubuntu/workspace/${resolved}` };
    }

    case 'cat': {
      const target = args[0];
      if (!target) {
        return { output: 'cat: missing file operand', isError: true };
      }
      const resolved = resolveWorkspacePath(ctx.cwd, target);
      const found = ctx.files.find(
        (f) => f.path === resolved || f.name === target
      );
      if (!found) {
        return {
          output: `cat: ${target}: No such file or directory`,
          isError: true,
        };
      }
      return { output: found.content };
    }

    case 'touch': {
      const target = args[0];
      if (!target) {
        return { output: 'touch: missing file operand', isError: true };
      }
      const resolved = resolveWorkspacePath(ctx.cwd, target);
      const existing = ctx.files.find((f) => f.path === resolved);
      if (existing) {
        return { output: '', selectFileId: existing.id };
      }
      const parts = resolved.split('/');
      const name = parts[parts.length - 1];
      const folder = parts.length > 1 ? parts[0] : 'worlds';
      const newFile: VirtualFile = {
        id: `file-${Date.now()}`,
        name,
        path: `${folder}/${name}`,
        folder,
        lastModified: Date.now(),
        content: `<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="default">
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>`,
      };
      return {
        output: `Created ${newFile.path}`,
        updatedFiles: [...ctx.files, newFile],
        selectFileId: newFile.id,
      };
    }

    case 'mkdir': {
      const folderName = (args[0] || '').replace(/^\/+|\/+$/g, '');
      if (!folderName) {
        return { output: 'mkdir: missing operand', isError: true };
      }
      return {
        output: `Directory '${folderName}/' ready in workspace.`,
        newCwd: `/home/ubuntu/workspace/${folderName}`,
      };
    }

    case 'rm': {
      const target = args.find((a) => !a.startsWith('-'));
      if (!target) {
        return { output: 'rm: missing operand', isError: true };
      }
      const resolved = resolveWorkspacePath(ctx.cwd, target);
      const found = ctx.files.find(
        (f) => f.path === resolved || f.name === target
      );
      if (!found) {
        return {
          output: `rm: cannot remove '${target}': No such file or directory`,
          isError: true,
        };
      }
      if (ctx.files.length <= 1) {
        return {
          output: 'rm: cannot delete the last remaining file in workspace.',
          isError: true,
        };
      }
      return {
        output: `Removed '${found.path}'`,
        updatedFiles: ctx.files.filter((f) => f.id !== found.id),
      };
    }

    case 'mv': {
      if (args.length < 2) {
        return {
          output: 'mv: missing destination file operand',
          isError: true,
        };
      }
      const srcResolved = resolveWorkspacePath(ctx.cwd, args[0]);
      const found = ctx.files.find(
        (f) => f.path === srcResolved || f.name === args[0]
      );
      if (!found) {
        return {
          output: `mv: cannot stat '${args[0]}': No such file or directory`,
          isError: true,
        };
      }
      const destResolved = resolveWorkspacePath(ctx.cwd, args[1]);
      const parts = destResolved.split('/');
      const newName = parts[parts.length - 1];
      const newFolder = parts.length > 1 ? parts[0] : found.folder;
      const updated = ctx.files.map((f) =>
        f.id === found.id
          ? {
              ...f,
              name: newName,
              folder: newFolder,
              path: `${newFolder}/${newName}`,
              lastModified: Date.now(),
            }
          : f
      );
      return {
        output: `Renamed '${found.path}' -> '${newFolder}/${newName}'`,
        updatedFiles: updated,
      };
    }

    case 'cp': {
      if (args.length < 2) {
        return {
          output: 'cp: missing destination file operand',
          isError: true,
        };
      }
      const srcResolved = resolveWorkspacePath(ctx.cwd, args[0]);
      const found = ctx.files.find(
        (f) => f.path === srcResolved || f.name === args[0]
      );
      if (!found) {
        return {
          output: `cp: cannot stat '${args[0]}': No such file or directory`,
          isError: true,
        };
      }
      const destResolved = resolveWorkspacePath(ctx.cwd, args[1]);
      const parts = destResolved.split('/');
      const newName = parts[parts.length - 1];
      const newFolder = parts.length > 1 ? parts[0] : found.folder;
      const copyFile: VirtualFile = {
        ...found,
        id: `file-${Date.now()}`,
        name: newName,
        folder: newFolder,
        path: `${newFolder}/${newName}`,
        lastModified: Date.now(),
      };
      return {
        output: `Copied '${found.path}' -> '${copyFile.path}'`,
        updatedFiles: [...ctx.files, copyFile],
        selectFileId: copyFile.id,
      };
    }

    case 'grep': {
      const pattern = args[0];
      if (!pattern) {
        return { output: 'Usage: grep <pattern> [file]', isError: true };
      }
      const fileArg = args[1];
      const targetFiles = fileArg
        ? ctx.files.filter(
            (f) =>
              f.path === resolveWorkspacePath(ctx.cwd, fileArg) ||
              f.name === fileArg
          )
        : ctx.files;

      const results: string[] = [];
      for (const f of targetFiles) {
        const lines = f.content.split('\n');
        lines.forEach((lineText, idx) => {
          if (lineText.toLowerCase().includes(pattern.toLowerCase())) {
            results.push(`${f.path}:${idx + 1}: ${lineText.trim()}`);
          }
        });
      }
      return {
        output:
          results.length > 0
            ? results.slice(0, 25).join('\n')
            : `(no matches found for "${pattern}")`,
      };
    }

    case 'echo':
      return { output: args.join(' ') };

    case 'gz':
    case 'ign': {
      const sub = args[0];
      if (!sub || sub === '--help' || sub === '-h') {
        return {
          output: [
            'Gazebo Harmonic CLI (gz-sim8 8.6.0)',
            'Available subcommands:',
            '  gz sim [options] [file.sdf]   Launch or control simulation',
            '  gz sdf -k <file.sdf>          Check & validate SDF/URDF syntax and inertia math',
            '  gz sdf -p <file>              Print converted SDF 1.10 representation',
            '  gz topic -l | -t <topic> ...  Inspect or publish to Gazebo transport topics',
            '  gz model --list | -m <name>   Inspect live models, links, and joints',
            '  gz wind <vx> <vy> <vz>        Set world wind velocity vector (m/s)',
          ].join('\n'),
        };
      }

      if (sub === 'sdf') {
        const flag = args[1];
        const fileTarget = args[2] || args[1];
        if (!fileTarget || fileTarget.startsWith('-')) {
          return {
            output: 'Usage: gz sdf -k <file.sdf>  OR  gz sdf -p <file.urdf>',
            isError: true,
          };
        }
        const resolved = resolveWorkspacePath(ctx.cwd, fileTarget);
        const file = ctx.files.find(
          (f) => f.path === resolved || f.name === fileTarget
        );
        if (!file) {
          return {
            output: `Error: File [${fileTarget}] not found in workspace.`,
            isError: true,
          };
        }

        const res = compileSdfOrUrdf(file.content, file.path);
        if (flag === '-p') {
          return {
            output: res.convertedSdfXml || file.content,
          };
        }

        const errCount = res.diagnostics.filter((d) => d.severity === 'error').length;
        const warnCount = res.diagnostics.filter((d) => d.severity === 'warning').length;
        const diagLines = res.diagnostics.map(
          (d) =>
            `  [${d.severity.toUpperCase()}] Line ${d.line}:${d.column} (${d.code}): ${d.message}`
        );

        if (res.isValid) {
          return {
            output: [
              `[gz-sdf] Checking '${file.path}' (Format: ${res.format.toUpperCase()} -> SDF 1.10)...`,
              `[gz-sdf] Check complete in ${res.compileTimeMs}ms: VALID (${errCount} errors, ${warnCount} warnings).`,
              `  World: "${res.world?.name}" | Models: ${res.world?.models.length} | Lights: ${res.world?.lights.length}`,
              ...diagLines,
            ].join('\n'),
          };
        } else {
          return {
            output: [
              `[gz-sdf] Checking '${file.path}'...`,
              `[gz-sdf] Validation FAILED (${errCount} errors, ${warnCount} warnings):`,
              ...diagLines,
            ].join('\n'),
            isError: true,
          };
        }
      }

      if (sub === 'sim') {
        const autoRun = args.includes('-r');
        const fileArg = args.find((a, idx) => idx > 0 && !a.startsWith('-'));
        if (!fileArg) {
          return {
            output: `[gz-sim] Starting simulation clock on active world "${ctx.activeWorld?.name || 'default'}"...`,
            startSimulation: true,
          };
        }
        const resolved = resolveWorkspacePath(ctx.cwd, fileArg);
        const file = ctx.files.find(
          (f) => f.path === resolved || f.name === fileArg
        );
        if (!file) {
          return {
            output: `[gz-sim] Error: Unable to find world file [${fileArg}]`,
            isError: true,
          };
        }
        const res = compileSdfOrUrdf(file.content, file.path);
        if (!res.isValid) {
          const errs = res.diagnostics
            .filter((d) => d.severity === 'error')
            .map((d) => `  [Err] [Line ${d.line}] ${d.message}`)
            .join('\n');
          return {
            output: `[gz-sim] Failed to load '${file.path}' due to SDF compilation errors:\n${errs}`,
            isError: true,
            loadAndCompileFile: file,
          };
        }
        return {
          output: [
            `[Msg] Gazebo Sim Server v8.6.0-harmonic`,
            `[Msg] Loading SDF world file [/usr/share/gz/gz-sim8/${file.path}]`,
            `[Msg] Serving world [${res.world?.name}] with ${res.world?.models.length} models and ${res.world?.lights.length} lights.`,
            autoRun
              ? '[Msg] Physics stepper RUNNING (-r flag active).'
              : '[Msg] World loaded. Press Play or run `gz sim -r` to step physics.',
          ].join('\n'),
          loadAndCompileFile: file,
          startSimulation: autoRun,
        };
      }

      if (sub === 'topic') {
        const worldName = ctx.activeWorld?.name || 'default';
        if (args.includes('-l') || args.includes('--list')) {
          const modelTopics: string[] = [];
          for (const m of ctx.activeWorld?.models || []) {
            if (!m.isStatic) {
              modelTopics.push(`/model/${m.name}/cmd_vel`);
              modelTopics.push(`/model/${m.name}/pose`);
              for (const j of m.joints) {
                modelTopics.push(`/model/${m.name}/joint/${j.name}/cmd_pos`);
              }
            }
          }
          return {
            output: [
              `/clock`,
              `/stats`,
              `/world/${worldName}/clock`,
              `/world/${worldName}/dynamic_pose/info`,
              `/world/${worldName}/scene/info`,
              `/world/${worldName}/wind`,
              ...modelTopics,
            ].join('\n'),
          };
        }

        const tIdx = args.indexOf('-t');
        const pIdx = args.indexOf('-p');
        if (tIdx !== -1 && args[tIdx + 1]) {
          const topic = args[tIdx + 1];
          if (args.includes('-i') || args.includes('--info')) {
            return {
              output: `Publishers [Address, Message Type]:\n  tcp://172.17.0.1:41921, gz.msgs.Twist\nSubscribers:\n  gz-sim-physics-system (${topic})`,
            };
          }
          if (pIdx !== -1 && args[pIdx + 1]) {
            const payload = args.slice(pIdx + 1).join(' ');
            const cmdVelMatch = topic.match(/\/model\/([^/]+)\/cmd_vel/);
            if (cmdVelMatch) {
              const modelName = cmdVelMatch[1];
              const lxMatch = payload.match(/linear\s*:\s*\{[^}]*x\s*:\s*([+-]?\d+(?:\.\d+)?)/i);
              const lyMatch = payload.match(/linear\s*:\s*\{[^}]*y\s*:\s*([+-]?\d+(?:\.\d+)?)/i);
              const azMatch = payload.match(/angular\s*:\s*\{[^}]*z\s*:\s*([+-]?\d+(?:\.\d+)?)/i);
              const linearX = lxMatch ? Number(lxMatch[1]) : 0;
              const linearY = lyMatch ? Number(lyMatch[1]) : 0;
              const angularZ = azMatch ? Number(azMatch[1]) : 0;
              return {
                output: `[gz-topic] Published Twist { linear: {x: ${linearX}, y: ${linearY}}, angular: {z: ${angularZ}} } to ${topic}`,
                cmdVelUpdate: { modelName, linearX, linearY, angularZ },
                startSimulation: true,
              };
            }

            const jointMatch = topic.match(/\/model\/([^/]+)\/joint\/([^/]+)\/cmd_pos/);
            if (jointMatch) {
              const modelName = jointMatch[1];
              const jointName = jointMatch[2];
              const numMatch = payload.match(/([+-]?\d+(?:\.\d+)?)/);
              const targetPosition = numMatch ? Number(numMatch[1]) : 0;
              return {
                output: `[gz-topic] Published Double { data: ${targetPosition} } to ${topic}`,
                jointUpdate: { modelName, jointName, targetPosition },
                startSimulation: true,
              };
            }
          }
        }
        return {
          output:
            'Usage: gz topic -l  OR  gz topic -t /model/<name>/cmd_vel -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.5}"',
        };
      }

      if (sub === 'model') {
        if (args.includes('--list') || args.includes('-l') || args.length === 1) {
          const models = ctx.activeWorld?.models || [];
          return {
            output: [
              `Active Models in World [${ctx.activeWorld?.name || 'default'}]:`,
              ...models.map(
                (m) =>
                  `  • [Entity ${m.entityId}] ${m.name} (static=${m.isStatic}, links=${m.links.length}, joints=${m.joints.length}) pose=[${m.pose.map((v) => v.toFixed(2)).join(' ')}]`
              ),
            ].join('\n'),
          };
        }
        const mIdx = args.indexOf('-m');
        if (mIdx !== -1 && args[mIdx + 1]) {
          const mName = args[mIdx + 1];
          const found = ctx.activeWorld?.models.find((m) => m.name === mName);
          if (!found) {
            return {
              output: `Model [${mName}] not found in world.`,
              isError: true,
            };
          }
          return {
            output: [
              `Model: ${found.name} (Entity ${found.entityId})`,
              `  Pose (x y z r p y): ${found.pose.map((v) => v.toFixed(3)).join(' ')}`,
              `  Static: ${found.isStatic} | Wind Enabled: ${found.enableWind}`,
              `  Links (${found.links.length}):`,
              ...found.links.map(
                (l) =>
                  `    - [Entity ${l.entityId}] ${l.name} (mass=${l.inertial.mass}kg, ixx=${l.inertial.inertia.ixx}, iyy=${l.inertial.inertia.iyy}, izz=${l.inertial.inertia.izz})`
              ),
              `  Joints (${found.joints.length}):`,
              ...found.joints.map(
                (j) =>
                  `    - [Entity ${j.entityId}] ${j.name} (${j.type}: ${j.parent} -> ${j.child}, pos=${j.position.toFixed(2)} rad)`
              ),
            ].join('\n'),
          };
        }
      }

      if (sub === 'plugin') {
        const worldPlugins = ctx.activeWorld?.plugins || [];
        const modelPlugins = (ctx.activeWorld?.models || []).flatMap(
          (m) => m.plugins || []
        );
        const allPlugins = [...worldPlugins, ...modelPlugins];
        return {
          output: [
            'Gazebo Harmonic System Plugins (https://github.com/gazebosim/gz-sim/tree/main/src/systems):',
            ...allPlugins.map(
              (p) =>
                `  • [${p.scope.toUpperCase()}: ${p.targetName || 'default'}] ${p.name} (${p.filename})`
            ),
          ].join('\n'),
        };
      }

      if (sub === 'wind') {
        const vx = Number(args[1] ?? 0);
        const vy = Number(args[2] ?? 0);
        const vz = Number(args[3] ?? 0);
        if ([vx, vy, vz].some((n) => Number.isNaN(n))) {
          return { output: 'Usage: gz wind <vx> <vy> <vz>', isError: true };
        }
        return {
          output: `[gz-wind] Updated world wind linear_velocity to [${vx.toFixed(2)}, ${vy.toFixed(2)}, ${vz.toFixed(2)}] m/s`,
          windUpdate: [vx, vy, vz],
        };
      }

      return {
        output: `gz: unknown subcommand '${sub}'. Type 'help' or 'gz --help' for usage.`,
        isError: true,
      };
    }

    case 'ros2': {
      const sub = args[0];
      const rules = ctx.ros2State?.bridgeRules || [];
      const isBridgeRunning = ctx.ros2State?.isBridgeRunning ?? true;
      const heroModel =
        ctx.activeWorld?.models.find((m) => m.name !== 'ground_plane') ||
        ctx.activeWorld?.models[0];

      if (!sub || sub === '--help' || sub === '-h') {
        return {
          output: [
            'ROS 2 Jazzy Jalisco CLI (Ubuntu 24.04 LTS)',
            'Available commands:',
            '  ros2 launch ' + DEFAULT_LAUNCH_PATH,
            '  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=' + DEFAULT_BRIDGE_YAML_PATH,
            '  ros2 run ros_gz_bridge parameter_bridge <topic>@<ros_type>[<gz_type> ...',
            '  ros2 run tf2_ros tf2_echo odom base_footprint',
            '  ros2 topic list | echo <topic> | info <topic> --verbose | hz <topic> | pub <topic> ...',
            '  ros2 node list | info <node>',
            '  ros2 param list | get <node> use_sim_time',
          ].join('\n'),
        };
      }

      if (sub === 'launch') {
        const launchFileArg = args[1] || DEFAULT_LAUNCH_PATH;
        const yamlFile = ctx.files.find((f) =>
          f.path.endsWith('ros_gz_bridge.yaml')
        );
        const parsedYaml = yamlFile ? parseRosGzBridgeYaml(yamlFile.content) : null;
        const activeRules =
          parsedYaml && parsedYaml.rules.length > 0 ? parsedYaml.rules : rules;

        return {
          output: [
            `[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log`,
            `[INFO] [launch]: Default logging verbosity is set to INFO`,
            `[INFO] [gz-sim-1]: process started with pid [2104] (Gazebo Harmonic gz-sim8 -r)`,
            `[INFO] [robot_state_publisher-2]: process started with pid [2108] (use_sim_time:=True)`,
            `[INFO] [ros_gz_bridge-3]: process started with pid [2112] (parameter_bridge -> ${launchFileArg})`,
            ...activeRules
              .filter((r) => r.enabled)
              .map(
                (r) =>
                  `[INFO] [ros_gz_bridge]: Creating bridge: [${r.gzTopic}] (${r.gzType}) ${directionToCliSymbol(r.direction)} [${r.rosTopic}] (${r.rosType})`
              ),
            `[INFO] [rviz2-4]: Dual-Output ROS 2 RViz2 + Gazebo Harmonic bridge active.`,
          ].join('\n'),
          startSimulation: true,
          ros2Action: {
            setBridgeRunning: true,
            replaceBridgeRules:
              parsedYaml && parsedYaml.rules.length > 0
                ? parsedYaml.rules
                : undefined,
            setNodeEnabled: { node: 'robotStatePublisher', enabled: true },
          },
        };
      }

      if (sub === 'run') {
        const pkg = args[1];
        const exec = args[2];
        if (pkg === 'ros_gz_bridge' && exec === 'parameter_bridge') {
          const cliRuleArgs = args
            .slice(3)
            .filter((a) => a.includes('@') && !a.startsWith('-'));
          if (cliRuleArgs.length > 0) {
            const parsedCliRules = cliRuleArgs
              .map((a, idx) => parseParameterBridgeCliArg(a, idx))
              .filter((r): r is RosGzBridgeRule => r !== null);
            return {
              output: [
                '[INFO] [ros_gz_bridge]: Starting parameter_bridge with explicit CLI topic rules:',
                ...parsedCliRules.map(
                  (r) =>
                    `  • ${formatParameterBridgeCliArg(r)}  (direction=${r.direction}, qos=${r.qosProfile})`
                ),
              ].join('\n'),
              startSimulation: true,
              ros2Action: {
                setBridgeRunning: true,
                upsertBridgeRule: parsedCliRules[0],
              },
            };
          }

          const yamlFile = ctx.files.find((f) =>
            f.path.endsWith('ros_gz_bridge.yaml')
          );
          const parsedYaml = yamlFile
            ? parseRosGzBridgeYaml(yamlFile.content)
            : null;
          const activeRules =
            parsedYaml && parsedYaml.rules.length > 0 ? parsedYaml.rules : rules;

          return {
            output: [
              '[INFO] [ros_gz_bridge]: Creating GZ <-> ROS 2 parameter_bridge node...',
              ...activeRules.map(
                (r) =>
                  `[INFO] [ros_gz_bridge]: [${directionToCliSymbol(r.direction)}] ${r.rosTopic} (${r.rosType}) <-> ${r.gzTopic} (${r.gzType}) [QoS: ${r.qosProfile}]`
              ),
            ].join('\n'),
            startSimulation: true,
            ros2Action: {
              setBridgeRunning: true,
              replaceBridgeRules:
                parsedYaml && parsedYaml.rules.length > 0
                  ? parsedYaml.rules
                  : undefined,
            },
          };
        }

        if (pkg === 'tf2_ros' && exec === 'tf2_echo') {
          const sourceFrame = args[3] || 'odom';
          const targetFrame =
            args[4] || heroModel?.virtualRootLink || heroModel?.canonicalLink || 'base_footprint';
          const pose = heroModel?.pose || [0, 0, 0, 0, 0, 0];
          return {
            output: [
              `At time ${(ctx.ros2State?.rosClockSec ?? ctx.simTime).toFixed(3)}`,
              `- Translation: [${pose[0].toFixed(3)}, ${pose[1].toFixed(3)}, ${pose[2].toFixed(3)}]`,
              `- Rotation: in Quaternion [0.000, 0.000, ${Math.sin(pose[5] * 0.5).toFixed(3)}, ${Math.cos(pose[5] * 0.5).toFixed(3)}]`,
              `- Rotation: in RPY (radian) [${pose[3].toFixed(3)}, ${pose[4].toFixed(3)}, ${pose[5].toFixed(3)}]`,
              `            Frame: ${sourceFrame} -> ${targetFrame} (Bridged=${isBridgeRunning})`,
            ].join('\n'),
          };
        }
      }

      if (sub === 'topic') {
        const topicCmd = args[1] || 'list';
        if (topicCmd === 'list') {
          const showTypes = args.includes('-t') || args.includes('--show-types');
          const activeRosTopics = isBridgeRunning
            ? rules
                .filter((r) => r.enabled)
                .map((r) =>
                  showTypes ? `${r.rosTopic} [${r.rosType}]` : r.rosTopic
                )
            : [];
          return {
            output: [
              showTypes
                ? '/parameter_events [rcl_interfaces/msg/ParameterEvent]'
                : '/parameter_events',
              showTypes ? '/rosout [rcl_interfaces/msg/Log]' : '/rosout',
              showTypes
                ? '/robot_description [std_msgs/msg/String]'
                : '/robot_description',
              showTypes ? '/tf_static [tf2_msgs/msg/TFMessage]' : '/tf_static',
              ...activeRosTopics,
            ].join('\n'),
          };
        }

        if (topicCmd === 'info') {
          const tName = args[2] || '/scan';
          const rule = rules.find((r) => r.rosTopic === tName);
          if (!rule) {
            return {
              output: `Unknown topic '${tName}'. Run 'ros2 topic list' to see active ROS 2 topics.`,
              isError: true,
            };
          }
          const reliability =
            rule.qosProfile === 'sensor_data' ? 'BEST_EFFORT' : 'RELIABLE';
          return {
            output: [
              `Type: ${rule.rosType}`,
              `Gazebo Counterpart: ${rule.gzTopic} (${rule.gzType})`,
              `Bridge Direction: ${rule.direction} (${directionToCliSymbol(rule.direction)}) | Active: ${isBridgeRunning && rule.enabled}`,
              `Publisher count: ${isBridgeRunning && rule.enabled ? 1 : 0}`,
              `Subscription count: 2 (/rviz2, /ros_gz_bridge)`,
              `QoS Profile (${rule.qosProfile}):`,
              `  Reliability: ${reliability}`,
              `  History (Depth): KEEP_LAST (10)`,
              `  Durability: VOLATILE`,
            ].join('\n'),
          };
        }

        if (topicCmd === 'hz') {
          const tName = args[2] || '/scan';
          const rule = rules.find((r) => r.rosTopic === tName);
          if (!rule || !isBridgeRunning || !rule.enabled) {
            return {
              output: `WARNING: topic [${tName}] does not appear to be published yet (ros_gz_bridge paused or unbridged).`,
              isError: true,
            };
          }
          return {
            output: [
              `average rate: ${rule.rateHz.toFixed(3)}`,
              `\tmin: ${(1 / rule.rateHz).toFixed(4)}s max: ${(1 / rule.rateHz).toFixed(4)}s std dev: 0.00012s window: ${Math.max(10, rule.msgCount)}`,
            ].join('\n'),
          };
        }

        if (topicCmd === 'echo') {
          const tName = args[2] || '/scan';
          const rule = rules.find((r) => r.rosTopic === tName);
          if (!rule || !isBridgeRunning || !rule.enabled) {
            return {
              output: `WARNING: No messages received on [${tName}]. Check that ros_gz_bridge is active and direction is '[' or '@'.`,
              isError: true,
            };
          }
          return {
            output: [
              `header:`,
              `  stamp:`,
              `    sec: ${Math.floor(ctx.ros2State?.rosClockSec ?? ctx.simTime)}`,
              `    nanosec: ${Math.floor(((ctx.ros2State?.rosClockSec ?? ctx.simTime) % 1) * 1e9)}`,
              `  frame_id: ${heroModel?.canonicalLink || 'base_link'}`,
              `bridge_summary: "${rule.lastPayloadSummary}"`,
              `---`,
            ].join('\n'),
          };
        }

        if (topicCmd === 'pub') {
          const topic = args.find((a, idx) => idx >= 2 && a.startsWith('/')) || '/cmd_vel';
          const payload = args.slice(2).join(' ');
          const lxMatch = payload.match(/linear\s*:\s*\{[^}]*x\s*:\s*([+-]?\d+(?:\.\d+)?)/i);
          const azMatch = payload.match(/angular\s*:\s*\{[^}]*z\s*:\s*([+-]?\d+(?:\.\d+)?)/i);
          const linearX = lxMatch ? Number(lxMatch[1]) : 0.8;
          const angularZ = azMatch ? Number(azMatch[1]) : 0.35;

          const cmdRule = rules.find((r) => r.rosTopic === '/cmd_vel');
          const canCrossBridge =
            isBridgeRunning &&
            cmdRule &&
            cmdRule.enabled &&
            (cmdRule.direction === 'ROS_TO_GZ' ||
              cmdRule.direction === 'BIDIRECTIONAL');

          return {
            output: [
              `publisher: beginning loop`,
              `publishing #1: geometry_msgs.msg.Twist(linear=Vector3(x=${linearX}, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=${angularZ}))`,
              canCrossBridge
                ? `[ros_gz_bridge]: Forwarded ${topic} (]) -> ${cmdRule?.gzTopic} (gz.msgs.Twist)`
                : `[ros_gz_bridge] WARNING: ${topic} published on ROS 2 bus, but ROS_TO_GZ (]) bridge is disabled or stopped!`,
            ].join('\n'),
            startSimulation: true,
            ros2Action: {
              publishRosCmdVel: { linearX, linearY: 0, angularZ },
            },
          };
        }
      }

      if (sub === 'node') {
        const nodeCmd = args[1] || 'list';
        if (nodeCmd === 'list') {
          const activeNodes = ['/rviz2'];
          if (isBridgeRunning) activeNodes.unshift('/ros_gz_bridge');
          if (ctx.ros2State?.nodes.robotStatePublisher ?? true)
            activeNodes.push('/robot_state_publisher');
          if (ctx.ros2State?.nodes.teleopTwistRos2 ?? true)
            activeNodes.push('/teleop_twist_ros2');
          if (ctx.ros2State?.nodes.lidarObstacleAvoidance)
            activeNodes.push('/lidar_obstacle_avoidance');
          if (ctx.ros2State?.nodes.jointTrajectoryPublisher)
            activeNodes.push('/joint_trajectory_publisher');
          return { output: activeNodes.join('\n') };
        }
        const targetNode = args[2] || '/ros_gz_bridge';
        return {
          output: [
            targetNode,
            '  Subscribers:',
            ...rules
              .filter(
                (r) =>
                  r.enabled &&
                  (r.direction === 'ROS_TO_GZ' || r.direction === 'BIDIRECTIONAL')
              )
              .map((r) => `    ${r.rosTopic}: ${r.rosType}`),
            '  Publishers:',
            ...rules
              .filter(
                (r) =>
                  r.enabled &&
                  (r.direction === 'GZ_TO_ROS' || r.direction === 'BIDIRECTIONAL')
              )
              .map((r) => `    ${r.rosTopic}: ${r.rosType}`),
            '  Service Servers:',
            `    ${targetNode}/describe_parameters: rcl_interfaces/srv/DescribeParameters`,
            `    ${targetNode}/get_parameters: rcl_interfaces/srv/GetParameters`,
          ].join('\n'),
        };
      }

      if (sub === 'param') {
        const pCmd = args[1] || 'list';
        if (pCmd === 'list') {
          return {
            output: [
              '/ros_gz_bridge:',
              '  config_file',
              '  qos_overrides./clock.publisher.reliability',
              '  use_sim_time',
              '/robot_state_publisher:',
              '  publish_frequency',
              '  robot_description',
              '  use_sim_time',
            ].join('\n'),
          };
        }
        return {
          output: `Boolean value is: ${ctx.ros2State?.useSimTime ?? true} (synchronized to /clock)`,
        };
      }

      return {
        output:
          "Unknown ros2 subcommand. Run 'ros2 --help' for supported ROS 2 Jazzy CLI commands.",
        isError: true,
      };
    }

    default:
      return {
        output: `${cmd}: command not found (Ubuntu 24.04 LTS container). Type 'help' for supported Gazebo, ROS 2 & bash commands.`,
        isError: true,
      };
  }
}
