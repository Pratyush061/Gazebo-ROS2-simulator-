import React, { useState, useEffect, useRef, useCallback } from 'react';
import {
  Code2,
  Share2,
  Globe,
  Columns,
  EyeOff,
  Maximize2,
  Minimize2,
  Play,
  Pause,
  RotateCcw,
  Radio,
  Cpu,
  Puzzle,
} from 'lucide-react';
import { DEFAULT_FILES } from './data/defaultFiles';
import {
  CompilationResult,
  DebugOverlaySettings,
  LightSpawnType,
  ParsedWorld,
  Pose6D,
  PrimitiveShapeType,
  Ros2RuntimeState,
  RosGzBridgeDirection,
  TerminalHistoryEntry,
  TransformGizmoMode,
  VirtualFile,
} from './types/simulation';
import {
  cloneModelEntityForPaste,
  compileSdfOrUrdf,
  createPrimitiveModelEntity,
  createSpawnLightEntity,
  generateSpawnLightSdfSnippet,
  generateSpawnPrimitiveSdfSnippet,
} from './utils/sdfCompiler';
import {
  deleteEntityFromWorld,
  getNextAvailableEntityId,
  scaleEntityGeometryInWorld,
  stepPhysicsWorld,
  updateEntityPoseInWorld,
} from './utils/physicsEngine';
import { executeUbuntuCommand } from './utils/ubuntuTerminal';
import { injectSystemPluginIntoXml } from './utils/gzSystemsCatalog';
import {
  createDefaultRos2RuntimeState,
  parseRosGzBridgeYaml,
  retargetBridgeRulesToModel,
  serializeRosGzBridgeYaml,
  stepRosGzBridgeAndNodes,
} from './utils/rosGzBridgeEngine';
import { VsCodeIdePane } from './components/VsCodeIdePane';
import { GazeboSimulatorPane } from './components/GazeboSimulatorPane';
import { Ros2VisualizerPane } from './components/Ros2VisualizerPane';

type PaneId = 'ide' | 'ros2' | 'gazebo';

export function App() {
  // Default to diff_drive_rover.urdf so the user immediately sees the URDF rover + LiDAR + IMU + DiffDrive in both Gazebo & ROS 2
  const initialFile =
    DEFAULT_FILES.find((f) => f.path === 'urdf/diff_drive_rover.urdf') ||
    DEFAULT_FILES[0];

  const [files, setFiles] = useState<VirtualFile[]>(DEFAULT_FILES);
  const [activeFileId, setActiveFileId] = useState<string>(initialFile.id);

  const activeFile =
    files.find((f) => f.id === activeFileId) || files[0];

  // Compile initial world
  const [compilationResult, setCompilationResult] = useState<CompilationResult>(() =>
    compileSdfOrUrdf(initialFile.content, initialFile.path)
  );

  const [world, setWorld] = useState<ParsedWorld | null>(() => {
    const res = compileSdfOrUrdf(initialFile.content, initialFile.path);
    return res.world ? JSON.parse(JSON.stringify(res.world)) : null;
  });

  // Official ROS 2 Jazzy + ros_gz_bridge runtime state
  const [ros2State, setRos2State] = useState<Ros2RuntimeState>(() =>
    createDefaultRos2RuntimeState(world)
  );
  const ros2StateRef = useRef<Ros2RuntimeState>(ros2State);
  ros2StateRef.current = ros2State;

  // Simulation playback state
  const [isSimRunning, setIsSimRunning] = useState<boolean>(true);
  const [stepTrigger, setStepTrigger] = useState<number>(0);
  const [selectedEntityId, setSelectedEntityId] = useState<number | null>(null);
  const [gizmoMode, setGizmoMode] = useState<TransformGizmoMode>('select');
  const [isPluginModalOpen, setIsPluginModalOpen] = useState<boolean>(false);

  const [debugOverlays, setDebugOverlays] = useState<DebugOverlaySettings>({
    showGrid: true,
    showCollisionWireframes: false,
    showInertiaAndCoM: false,
    showJointAxesAndFrames: true,
    showWindVectors: false,
    showLightFrustums: false,
    wireframeMode: false,
    showLidarRays: true,
    showContactPoints: true,
  });

  // Telemetry state
  const [simTime, setSimTime] = useState<number>(0);
  const [realTime, setRealTime] = useState<number>(0);
  const [rtf, setRtf] = useState<number>(1.0);
  const [iterations, setIterations] = useState<number>(0);

  // Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic terminal history
  const [terminalCwd, setTerminalCwd] = useState<string>('/home/ubuntu/workspace');
  const [terminalHistory, setTerminalHistory] = useState<TerminalHistoryEntry[]>([
    {
      id: 'init-1',
      cwd: '/home/ubuntu/workspace',
      command: 'ros2 launch ros2_ws/ros_gz_sim.launch.py use_sim_time:=true',
      output: `[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gz_sim-1]: Launching Gazebo Harmonic (gz-sim8) with separate ROS 2 & Gazebo workspaces...
[INFO] [parameter_bridge-2]: Starting ros_gz_bridge parameter_bridge with ros2_ws/ros_gz_bridge.yaml
  ├── /clock @ rosgraph_msgs/msg/Clock [ gz.msgs.Clock (GZ_TO_ROS)
  ├── /scan  @ sensor_msgs/msg/LaserScan [ gz.msgs.LaserScan (GZ_TO_ROS)
  ├── /imu   @ sensor_msgs/msg/Imu [ gz.msgs.IMU (GZ_TO_ROS)
  ├── /odom  @ nav_msgs/msg/Odometry [ gz.msgs.Odometry (GZ_TO_ROS)
  ├── /joint_states @ sensor_msgs/msg/JointState [ gz.msgs.Model (GZ_TO_ROS)
  ├── /tf    @ tf2_msgs/msg/TFMessage [ gz.msgs.Pose_V (GZ_TO_ROS)
  ├── /cmd_vel @ geometry_msgs/msg/Twist ] gz.msgs.Twist (ROS_TO_GZ)
  └── /joint_cmd @ std_msgs/msg/Float64 ] gz.msgs.Double (ROS_TO_GZ)
[INFO] [robot_state_publisher-3]: Publishing URDF TF tree with use_sim_time:=True`,
        timestamp: Date.now(),
    },
  ]);

  // Multi-pane visibility, Minimize, Maximize & Dual-Output Mode state
  const [isIdeMinimized, setIsIdeMinimized] = useState<boolean>(false);
  const [isRos2Minimized, setIsRos2Minimized] = useState<boolean>(false);
  const [isGazeboMinimized, setIsGazeboMinimized] = useState<boolean>(false);
  const [maximizedPane, setMaximizedPane] = useState<PaneId | null>(null);

  // Resizable splitter percentages
  const [ideWidthPct, setIdeWidthPct] = useState<number>(32);
  const [ros2WidthPct, setRos2WidthPct] = useState<number>(34);
  const [isDesktopViewport, setIsDesktopViewport] = useState<boolean>(() =>
    typeof window !== 'undefined' ? window.innerWidth >= 1024 : true
  );
  const workspaceContainerRef = useRef<HTMLDivElement | null>(null);
  const draggingSplitterRef = useRef<'ide-ros2' | 'ros2-gazebo' | null>(null);

  useEffect(() => {
    const onResize = () => setIsDesktopViewport(window.innerWidth >= 1024);
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, []);

  // Counters for spawned primitives/lights/clones
  const spawnCounterRef = useRef<number>(1);
  const simTimeRef = useRef<number>(0);
  simTimeRef.current = simTime;

  // Handle dragging between panes
  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (!draggingSplitterRef.current || !workspaceContainerRef.current) return;
      const rect = workspaceContainerRef.current.getBoundingClientRect();
      if (rect.width <= 100) return;
      const relPct = ((e.clientX - rect.left) / rect.width) * 100;

      if (draggingSplitterRef.current === 'ide-ros2') {
        if (isIdeMinimized) return;
        const clampedIde = Math.max(20, Math.min(65, relPct));
        setIdeWidthPct(clampedIde);
      } else if (draggingSplitterRef.current === 'ros2-gazebo') {
        if (isIdeMinimized) {
          // Dual-Output Mode (ROS 2 + Gazebo only): splitter adjusts ros2WidthPct directly between 22% and 78%
          const clampedRos2 = Math.max(22, Math.min(78, relPct));
          setRos2WidthPct(clampedRos2);
        } else {
          // Tri-Pane Mode: relPct is ideWidthPct + ros2WidthPct
          const nextRos2 = Math.max(20, Math.min(78 - ideWidthPct, relPct - ideWidthPct));
          setRos2WidthPct(nextRos2);
        }
      }
    };

    const handleMouseUp = () => {
      if (draggingSplitterRef.current) {
        draggingSplitterRef.current = null;
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
      }
    };

    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isIdeMinimized, ideWidthPct]);

  // Synchronize `ros2_ws/ros_gz_bridge.yaml` file content when bridge rules change via UI
  const syncBridgeRulesToYamlFile = useCallback((nextRules: Ros2RuntimeState['bridgeRules']) => {
    const serialized = serializeRosGzBridgeYaml(nextRules);
    setFiles((prev) =>
      prev.map((f) =>
        f.path === 'ros2_ws/ros_gz_bridge.yaml'
          ? { ...f, content: serialized, lastModified: Date.now() }
          : f
      )
    );
  }, []);

  // Compile and optionally replace the running 3D world
  const compileFileContent = useCallback(
    (file: VirtualFile, applyToWorld: boolean) => {
      const res = compileSdfOrUrdf(file.content, file.path);
      setCompilationResult(res);

      if (res.format === 'ros2_yaml' && res.isValid) {
        const parsedYaml = parseRosGzBridgeYaml(file.content);
        if (parsedYaml.rules.length > 0) {
          setRos2State((prev) => ({
            ...prev,
            bridgeRules: parsedYaml.rules,
          }));
        }
        return res;
      }

      if (applyToWorld && res.isValid && res.world) {
        const freshWorld: ParsedWorld = JSON.parse(JSON.stringify(res.world));
        const heroModel =
          freshWorld.models.find((m) => m.name !== 'ground_plane') ||
          freshWorld.models[0];

        setWorld(freshWorld);
        setSelectedEntityId(heroModel ? heroModel.entityId : null);

        setRos2State((prev) => {
          const retargetedRules = heroModel
            ? retargetBridgeRulesToModel(
                prev.bridgeRules,
                heroModel.name,
                freshWorld.name
              )
            : prev.bridgeRules;
          const nextState: Ros2RuntimeState = {
            ...prev,
            targetModelName: heroModel?.name || prev.targetModelName,
            bridgeRules: retargetedRules,
            odometryTrail: [],
          };
          stepRosGzBridgeAndNodes(freshWorld, nextState, simTimeRef.current, 0.016);
          return nextState;
        });
        setStepTrigger((v) => v + 1);
      }

      return res;
    },
    []
  );

  // Telemetry callback from GazeboViewport3D animation loop:
  // also steps `ros_gz_bridge` and active ROS 2 nodes in lockstep with Gazebo!
  const handleTelemetryUpdate = useCallback(
    (nextSimTime: number, nextRealTime: number, nextRtf: number, nextIterations: number) => {
      setSimTime(nextSimTime);
      setRealTime(nextRealTime);
      setRtf(nextRtf);
      setIterations(nextIterations);

      if (world) {
        const clonedRos2: Ros2RuntimeState = {
          ...ros2StateRef.current,
          bridgeRules: ros2StateRef.current.bridgeRules.map((r) => ({ ...r })),
          bridgedTelemetry: {
            ...ros2StateRef.current.bridgedTelemetry,
            jointStates: [...ros2StateRef.current.bridgedTelemetry.jointStates],
            tfTree: [...ros2StateRef.current.bridgedTelemetry.tfTree],
          },
          odometryTrail: [...ros2StateRef.current.odometryTrail],
          pendingRosCmdVel: { ...ros2StateRef.current.pendingRosCmdVel },
          pendingRosJointCmds: { ...ros2StateRef.current.pendingRosJointCmds },
        };
        stepRosGzBridgeAndNodes(world, clonedRos2, nextSimTime, 0.05);
        setRos2State(clonedRos2);
      }
    },
    [world]
  );

  // Single-step simulation when paused
  const handleStepSimulation = useCallback(() => {
    if (!world) return;
    setIsSimRunning(false);
    const dt = 0.01;
    stepPhysicsWorld(world, dt);
    const nextSimTime = simTime + dt;
    setSimTime(nextSimTime);
    setIterations((it) => it + 1);

    const clonedRos2: Ros2RuntimeState = {
      ...ros2StateRef.current,
      bridgeRules: ros2StateRef.current.bridgeRules.map((r) => ({ ...r })),
      bridgedTelemetry: { ...ros2StateRef.current.bridgedTelemetry },
      odometryTrail: [...ros2StateRef.current.odometryTrail],
      pendingRosCmdVel: { ...ros2StateRef.current.pendingRosCmdVel },
      pendingRosJointCmds: { ...ros2StateRef.current.pendingRosJointCmds },
    };
    stepRosGzBridgeAndNodes(world, clonedRos2, nextSimTime, dt);
    setRos2State(clonedRos2);
    setStepTrigger((s) => s + 1);
  }, [world, simTime]);

  // Reset simulation to compiled file state
  const handleResetSimulation = useCallback(() => {
    const simFile =
      activeFile.path.endsWith('.sdf') || activeFile.path.endsWith('.urdf')
        ? activeFile
        : files.find((f) => f.path === 'urdf/diff_drive_rover.urdf') || files[0];
    const res = compileSdfOrUrdf(simFile.content, simFile.path);
    if (res.world) {
      const freshWorld: ParsedWorld = JSON.parse(JSON.stringify(res.world));
      setWorld(freshWorld);
      setSimTime(0);
      setRealTime(0);
      setIterations(0);
      setRos2State((prev) => {
        const next = {
          ...prev,
          rosClockSec: 0,
          odometryTrail: [],
          pendingRosCmdVel: { linearX: 0, linearY: 0, angularZ: 0, active: false },
          pendingRosJointCmds: {},
        };
        stepRosGzBridgeAndNodes(freshWorld, next, 0, 0.016);
        return next;
      });
      setStepTrigger((s) => s + 1);
    }
  }, [activeFile, files]);

  // File operations
  const handleSelectFile = (file: VirtualFile) => {
    setActiveFileId(file.id);
    // If selecting an SDF or URDF file, compile and load into Gazebo + ROS 2 bridge
    const isModelOrWorld =
      file.path.endsWith('.sdf') ||
      file.path.endsWith('.urdf') ||
      file.path.endsWith('.xml');
    compileFileContent(file, isModelOrWorld);
  };

  const handleUpdateFileContent = (fileId: string, newContent: string) => {
    setFiles((prev) =>
      prev.map((f) =>
        f.id === fileId
          ? { ...f, content: newContent, lastModified: Date.now() }
          : f
      )
    );
    const target = files.find((f) => f.id === fileId);
    if (target) {
      const updatedFile = { ...target, content: newContent };
      const res = compileSdfOrUrdf(newContent, updatedFile.path);
      setCompilationResult(res);
      if (updatedFile.path === 'ros2_ws/ros_gz_bridge.yaml' && res.isValid) {
        const parsed = parseRosGzBridgeYaml(newContent);
        if (parsed.rules.length > 0) {
          setRos2State((prev) => ({
            ...prev,
            bridgeRules: parsed.rules,
          }));
        }
      }
    }
  };

  const handleCreateFile = (
    name: string,
    folder: string,
    templateType: 'sdf' | 'urdf'
  ) => {
    const cleanName = name.trim();
    if (!cleanName) return;
    const finalName =
      cleanName.includes('.')
        ? cleanName
        : templateType === 'urdf'
        ? `${cleanName}.urdf`
        : `${cleanName}.sdf`;
    const newPath = `${folder}/${finalName}`;
    const templateContent =
      templateType === 'urdf'
        ? DEFAULT_FILES.find((f) => f.path === 'urdf/diff_drive_rover.urdf')?.content || ''
        : DEFAULT_FILES[0].content;

    const newFile: VirtualFile = {
      id: `file-${Date.now()}`,
      name: finalName,
      path: newPath,
      folder,
      description: `User created ${templateType.toUpperCase()} file in ${folder}/`,
      lastModified: Date.now(),
      content: templateContent,
    };
    setFiles((prev) => [...prev, newFile]);
    setActiveFileId(newFile.id);
    compileFileContent(newFile, true);
  };

  const handleRenameFile = (fileId: string, newName: string) => {
    const trimmed = newName.trim();
    if (!trimmed) return;
    setFiles((prev) =>
      prev.map((f) => {
        if (f.id !== fileId) return f;
        return {
          ...f,
          name: trimmed,
          path: `${f.folder}/${trimmed}`,
          lastModified: Date.now(),
        };
      })
    );
  };

  const handleDuplicateFile = (file: VirtualFile) => {
    const dotIdx = file.name.lastIndexOf('.');
    const base = dotIdx !== -1 ? file.name.slice(0, dotIdx) : file.name;
    const ext = dotIdx !== -1 ? file.name.slice(dotIdx) : '';
    const copyName = `${base}_copy${ext}`;
    const copyFile: VirtualFile = {
      ...file,
      id: `file-copy-${Date.now()}`,
      name: copyName,
      path: `${file.folder}/${copyName}`,
      lastModified: Date.now(),
    };
    setFiles((prev) => [...prev, copyFile]);
    setActiveFileId(copyFile.id);
  };

  const handleDeleteFile = (fileId: string) => {
    if (files.length <= 1) return;
    const remaining = files.filter((f) => f.id !== fileId);
    setFiles(remaining);
    if (activeFileId === fileId) {
      setActiveFileId(remaining[0].id);
      compileFileContent(remaining[0], true);
    }
  };

  const handleUploadFile = (uploaded: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      const content = String(reader.result || '');
      const lower = uploaded.name.toLowerCase();
      const folder = lower.endsWith('.urdf')
        ? 'urdf'
        : lower.endsWith('.yaml') || lower.endsWith('.py')
        ? 'ros2_ws'
        : 'worlds';
      const newFile: VirtualFile = {
        id: `file-upload-${Date.now()}`,
        name: uploaded.name,
        path: `${folder}/${uploaded.name}`,
        folder,
        description: 'Uploaded file',
        lastModified: Date.now(),
        content,
      };
      setFiles((prev) => [...prev, newFile]);
      setActiveFileId(newFile.id);
      compileFileContent(
        newFile,
        lower.endsWith('.sdf') || lower.endsWith('.urdf')
      );
    };
    reader.readAsText(uploaded);
  };

  const handleDownloadActiveFile = () => {
    const blob = new Blob([activeFile.content], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = activeFile.name;
    a.click();
    URL.revokeObjectURL(url);
  };

  // ROS 2 & ros_gz_bridge callbacks
  const handleToggleBridgeRunning = useCallback(() => {
    setRos2State((prev) => {
      const next: Ros2RuntimeState = {
        ...prev,
        isBridgeRunning: !prev.isBridgeRunning,
      };
      if (world) {
        stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
      }
      return next;
    });
    setStepTrigger((s) => s + 1);
  }, [world]);

  const handleToggleUseSimTime = useCallback(() => {
    setRos2State((prev) => {
      const next: Ros2RuntimeState = {
        ...prev,
        useSimTime: !prev.useSimTime,
      };
      if (world) {
        stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
      }
      return next;
    });
  }, [world]);

  const handleToggleBridgeRule = useCallback(
    (ruleId: string) => {
      setRos2State((prev) => {
        const updatedRules = prev.bridgeRules.map((r) =>
          r.id === ruleId ? { ...r, enabled: !r.enabled } : r
        );
        syncBridgeRulesToYamlFile(updatedRules);
        const next: Ros2RuntimeState = {
          ...prev,
          bridgeRules: updatedRules,
        };
        if (world) {
          stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
        }
        return next;
      });
      setStepTrigger((s) => s + 1);
    },
    [world, syncBridgeRulesToYamlFile]
  );

  const handleCycleBridgeRuleDirection = useCallback(
    (ruleId: string) => {
      const order: RosGzBridgeDirection[] = [
        'GZ_TO_ROS',
        'ROS_TO_GZ',
        'BIDIRECTIONAL',
      ];
      setRos2State((prev) => {
        const updatedRules = prev.bridgeRules.map((r) => {
          if (r.id !== ruleId) return r;
          const nextDir = order[(order.indexOf(r.direction) + 1) % order.length];
          return { ...r, direction: nextDir };
        });
        syncBridgeRulesToYamlFile(updatedRules);
        const next: Ros2RuntimeState = {
          ...prev,
          bridgeRules: updatedRules,
        };
        if (world) {
          stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
        }
        return next;
      });
      setStepTrigger((s) => s + 1);
    },
    [world, syncBridgeRulesToYamlFile]
  );

  const handleToggleRos2Node = useCallback(
    (nodeKey: keyof Ros2RuntimeState['nodes']) => {
      setRos2State((prev) => {
        const nextEnabled = !prev.nodes[nodeKey];
        const next: Ros2RuntimeState = {
          ...prev,
          nodes: {
            ...prev.nodes,
            [nodeKey]: nextEnabled,
          },
        };
        // If disabling obstacle avoidance, stop pending cmd_vel
        if (nodeKey === 'lidarObstacleAvoidance' && !nextEnabled) {
          next.pendingRosCmdVel = {
            linearX: 0,
            linearY: 0,
            angularZ: 0,
            active: false,
          };
          next.lastAvoidanceStatus = 'Idle (Node standby)';
          if (world) {
            const hero =
              world.models.find((m) => m.name === next.targetModelName) ||
              world.models.find((m) => m.name !== 'ground_plane');
            if (hero) {
              hero.cmdVel = { linearX: 0, linearY: 0, angularZ: 0 };
            }
          }
        }
        if (world) {
          stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
        }
        return next;
      });
      setStepTrigger((s) => s + 1);
    },
    [world]
  );

  const handlePublishRosCmdVel = useCallback(
    (linearX: number, linearY: number, angularZ: number) => {
      setRos2State((prev) => {
        const next: Ros2RuntimeState = {
          ...prev,
          pendingRosCmdVel: {
            linearX,
            linearY,
            angularZ,
            active: true,
          },
        };
        if (world) {
          stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
        }
        return next;
      });
      setStepTrigger((s) => s + 1);
    },
    [world]
  );

  const handlePublishRosJointCmd = useCallback(
    (jointName: string, positionRad: number) => {
      setRos2State((prev) => {
        const next: Ros2RuntimeState = {
          ...prev,
          pendingRosJointCmds: {
            ...prev.pendingRosJointCmds,
            [jointName]: positionRad,
          },
        };
        if (world) {
          stepRosGzBridgeAndNodes(world, next, simTimeRef.current, 0.016);
        }
        return next;
      });
      setStepTrigger((s) => s + 1);
    },
    [world]
  );

  const handleOpenBridgeYamlInIde = useCallback(() => {
    const yamlFile = files.find((f) => f.path === 'ros2_ws/ros_gz_bridge.yaml');
    if (yamlFile) {
      setActiveFileId(yamlFile.id);
      setIsIdeMinimized(false);
      if (maximizedPane && maximizedPane !== 'ide') {
        setMaximizedPane(null);
      }
      compileFileContent(yamlFile, false);
    }
  }, [files, maximizedPane, compileFileContent]);

  // Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic CLI command runner
  const handleRunTerminalCommand = (cmd: string) => {
    const res = executeUbuntuCommand(cmd, {
      cwd: terminalCwd,
      files,
      activeWorld: world,
      isSimRunning,
      simTime,
      ros2State,
    });

    if (res.clearScreen) {
      setTerminalHistory([]);
      return;
    }
    if (res.newCwd) {
      setTerminalCwd(res.newCwd);
    }
    if (res.updatedFiles) {
      setFiles(res.updatedFiles);
    }
    if (res.selectFileId) {
      setActiveFileId(res.selectFileId);
    }
    if (res.loadAndCompileFile) {
      setActiveFileId(res.loadAndCompileFile.id);
      compileFileContent(res.loadAndCompileFile, true);
    }
    if (res.startSimulation) {
      setIsSimRunning(true);
    }
    if (res.pauseSimulation) {
      setIsSimRunning(false);
    }
    if (res.resetSimulation) {
      handleResetSimulation();
    }
    if (res.cmdVelUpdate && world) {
      const m = world.models.find((mod) => mod.name === res.cmdVelUpdate?.modelName);
      if (m) {
        m.cmdVel = {
          linearX: res.cmdVelUpdate.linearX,
          linearY: res.cmdVelUpdate.linearY,
          angularZ: res.cmdVelUpdate.angularZ,
        };
        setStepTrigger((s) => s + 1);
      }
    }
    if (res.jointUpdate && world) {
      const m = world.models.find((mod) => mod.name === res.jointUpdate?.modelName);
      const j = m?.joints.find((jt) => jt.name === res.jointUpdate?.jointName);
      if (j) {
        j.controlMode = 'position';
        j.targetPosition = res.jointUpdate.targetPosition;
        setStepTrigger((s) => s + 1);
      }
    }
    if (res.windUpdate && world) {
      world.wind = {
        enabled: Math.hypot(...res.windUpdate) > 0.001,
        linearVelocity: res.windUpdate,
      };
      setStepTrigger((s) => s + 1);
    }
    if (res.ros2Action) {
      const act = res.ros2Action;
      setRos2State((prev) => {
        let nextRules = [...prev.bridgeRules];
        if (act.replaceBridgeRules) {
          nextRules = act.replaceBridgeRules;
          syncBridgeRulesToYamlFile(nextRules);
        }
        if (act.upsertBridgeRule) {
          const rule = act.upsertBridgeRule;
          const idx = nextRules.findIndex(
            (r) => r.rosTopic === rule.rosTopic || r.gzTopic === rule.gzTopic
          );
          if (idx >= 0) {
            nextRules[idx] = { ...nextRules[idx], ...rule, enabled: true };
          } else {
            nextRules.push(rule);
          }
          syncBridgeRulesToYamlFile(nextRules);
        }

        const nextNodes = { ...prev.nodes };
        if (act.setNodeEnabled) {
          nextNodes[act.setNodeEnabled.node] = act.setNodeEnabled.enabled;
        }

        const nextCmdVel = act.publishRosCmdVel
          ? {
              linearX: act.publishRosCmdVel.linearX,
              linearY: act.publishRosCmdVel.linearY,
              angularZ: act.publishRosCmdVel.angularZ,
              active: true,
            }
          : prev.pendingRosCmdVel;

        const nextState: Ros2RuntimeState = {
          ...prev,
          isBridgeRunning:
            act.setBridgeRunning !== undefined
              ? act.setBridgeRunning
              : prev.isBridgeRunning,
          useSimTime:
            act.setUseSimTime !== undefined ? act.setUseSimTime : prev.useSimTime,
          bridgeRules: nextRules,
          nodes: nextNodes,
          pendingRosCmdVel: nextCmdVel,
        };

        if (world) {
          stepRosGzBridgeAndNodes(world, nextState, simTimeRef.current, 0.016);
        }
        return nextState;
      });
      setStepTrigger((s) => s + 1);
    }

    setTerminalHistory((prev) => [
      ...prev,
      {
        id: `cmd-${Date.now()}-${Math.random()}`,
        cwd: res.newCwd || terminalCwd,
        command: cmd,
        output: res.output,
        isError: res.isError,
          timestamp: Date.now(),
      },
    ]);
  };

  // Gazebo entity manipulation callbacks
  const handleUpdateEntityPose = (entityId: number, newPose: Pose6D) => {
    if (!world) return;
    const updated = updateEntityPoseInWorld(world, entityId, newPose);
    setWorld(updated);
    setStepTrigger((s) => s + 1);
  };

  const handleScaleEntity = (
    entityId: number,
    scaleFactors: [number, number, number]
  ) => {
    if (!world) return;
    const updated = scaleEntityGeometryInWorld(world, entityId, scaleFactors);
    setWorld(updated);
    setStepTrigger((s) => s + 1);
  };

  const handleDeleteEntity = (entityId: number) => {
    if (!world) return;
    const updated = deleteEntityFromWorld(world, entityId);
    setWorld(updated);
    if (selectedEntityId === entityId) {
      setSelectedEntityId(null);
    }
    setStepTrigger((s) => s + 1);
  };

  const handleToggleModelProperty = (
    modelId: string,
    prop: 'enableWind' | 'selfCollide' | 'isStatic'
  ) => {
    if (!world) return;
    const target = world.models.find((m) => m.id === modelId);
    if (!target) return;
    target[prop] = !target[prop];
    if (prop === 'enableWind') {
      target.links.forEach((l) => {
        l.enableWind = target.enableWind;
      });
    }
    setStepTrigger((s) => s + 1);
  };

  const handleSpawnPrimitiveAtPose = (shape: PrimitiveShapeType, pose: Pose6D) => {
    if (!world) return;
    const idx = spawnCounterRef.current++;
    const nextEntityId = getNextAvailableEntityId(world);
    const modelEntity = createPrimitiveModelEntity(
      shape,
      idx,
      pose,
      nextEntityId,
      activeFile.path
    );
    world.models.push(modelEntity);
    setSelectedEntityId(modelEntity.entityId);

    // If active file is SDF, append snippet
    if (activeFile.path.endsWith('.sdf')) {
      const snippet = generateSpawnPrimitiveSdfSnippet(shape, idx, pose);
      const closeWorldIdx = activeFile.content.lastIndexOf('</world>');
      if (closeWorldIdx !== -1) {
        const updatedXml =
          activeFile.content.slice(0, closeWorldIdx) +
          snippet +
          '\n  ' +
          activeFile.content.slice(closeWorldIdx);
        handleUpdateFileContent(activeFile.id, updatedXml);
      }
    }
    setStepTrigger((s) => s + 1);
  };

  const handleSpawnLightAtPose = (lightType: LightSpawnType, pose: Pose6D) => {
    if (!world) return;
    const idx = spawnCounterRef.current++;
    const nextEntityId = getNextAvailableEntityId(world);
    const lightEntity = createSpawnLightEntity(lightType, idx, pose, nextEntityId);
    world.lights.push(lightEntity);
    setSelectedEntityId(lightEntity.entityId);

    if (activeFile.path.endsWith('.sdf')) {
      const snippet = generateSpawnLightSdfSnippet(lightType, idx, pose);
      const closeWorldIdx = activeFile.content.lastIndexOf('</world>');
      if (closeWorldIdx !== -1) {
        const updatedXml =
          activeFile.content.slice(0, closeWorldIdx) +
          snippet +
          '\n  ' +
          activeFile.content.slice(closeWorldIdx);
        handleUpdateFileContent(activeFile.id, updatedXml);
      }
    }
    setStepTrigger((s) => s + 1);
  };

  const handlePasteModelAtPose = (sourceEntityId: number, pose: Pose6D) => {
    if (!world) return;
    const srcModel = world.models.find((m) => m.entityId === sourceEntityId);
    if (!srcModel) return;
    const idx = spawnCounterRef.current++;
    const nextId = getNextAvailableEntityId(world);
    const { model: cloned, sdfSnippet } = cloneModelEntityForPaste(
      srcModel,
      idx,
      pose,
      nextId
    );
    world.models.push(cloned);
    setSelectedEntityId(cloned.entityId);

    if (activeFile.path.endsWith('.sdf')) {
      const closeWorldIdx = activeFile.content.lastIndexOf('</world>');
      if (closeWorldIdx !== -1) {
        const updatedXml =
          activeFile.content.slice(0, closeWorldIdx) +
          sdfSnippet +
          '\n  ' +
          activeFile.content.slice(closeWorldIdx);
        handleUpdateFileContent(activeFile.id, updatedXml);
      }
    }
    setStepTrigger((s) => s + 1);
  };

  const handleSyncSceneToEditor = () => {
    if (!world) return;
    setStepTrigger((s) => s + 1);
  };

  const handleUpdateJointTarget = (
    modelId: string,
    jointId: string,
    mode: 'position' | 'velocity' | 'effort',
    value: number
  ) => {
    if (!world) return;
    const m = world.models.find((mod) => mod.id === modelId);
    const j = m?.joints.find((jt) => jt.id === jointId);
    if (!j) return;
    j.controlMode = mode;
    if (mode === 'position') {
      j.targetPosition = value;
    } else if (mode === 'velocity') {
      j.targetVelocity = value;
    } else {
      j.appliedEffort = value;
    }
    setStepTrigger((s) => s + 1);
  };

  const handleUpdateModelCmdVel = (
    modelId: string,
    linearX: number,
    linearY: number,
    angularZ: number
  ) => {
    if (!world) return;
    const m = world.models.find((mod) => mod.id === modelId);
    if (!m) return;
    m.cmdVel = { linearX, linearY, angularZ };
    setStepTrigger((s) => s + 1);
  };

  const handleUpdateWorldWind = (
    vx: number,
    vy: number,
    vz: number,
    enabled: boolean
  ) => {
    if (!world) return;
    world.wind = {
      enabled,
      linearVelocity: [vx, vy, vz],
    };
    setStepTrigger((s) => s + 1);
  };

  const handleInsertSystemPlugin = (
    templateId: string,
    targetModelName?: string,
    autoRun = true
  ) => {
    const format = activeFile.path.endsWith('.urdf') ? 'urdf' : 'sdf';
    const targetModel =
      world?.models.find((m) => m.name === targetModelName) ||
      world?.models.find((m) => m.name !== 'ground_plane') ||
      world?.models[0];
    const updatedXml = injectSystemPluginIntoXml(
      activeFile.content,
      format,
      templateId,
      targetModel
    );
    const updatedFile: VirtualFile = {
      ...activeFile,
      content: updatedXml,
      lastModified: Date.now(),
    };
    setFiles((prev) =>
      prev.map((f) => (f.id === activeFile.id ? updatedFile : f))
    );
    compileFileContent(updatedFile, autoRun);
  };

  // Layout Preset Handlers
  const activateDualOutputMode = () => {
    setMaximizedPane(null);
    setIsIdeMinimized(true);
    setIsRos2Minimized(false);
    setIsGazeboMinimized(false);
    setRos2WidthPct(48);
  };

  const activateTriPaneMode = () => {
    setMaximizedPane(null);
    setIsIdeMinimized(false);
    setIsRos2Minimized(false);
    setIsGazeboMinimized(false);
    setIdeWidthPct(31);
    setRos2WidthPct(34);
  };

  const activateIdeAndGazeboMode = () => {
    setMaximizedPane(null);
    setIsIdeMinimized(false);
    setIsRos2Minimized(true);
    setIsGazeboMinimized(false);
    setIdeWidthPct(42);
  };

  const activateIdeAndRos2Mode = () => {
    setMaximizedPane(null);
    setIsIdeMinimized(false);
    setIsRos2Minimized(false);
    setIsGazeboMinimized(true);
    setIdeWidthPct(42);
  };

  const toggleMaximizePane = (pane: PaneId) => {
    setMaximizedPane((prev) => (prev === pane ? null : pane));
  };

  // Determine which panes are currently rendered
  const showIde =
    maximizedPane === 'ide' || (maximizedPane === null && !isIdeMinimized);
  const showRos2 =
    maximizedPane === 'ros2' || (maximizedPane === null && !isRos2Minimized);
  const showGazebo =
    maximizedPane === 'gazebo' || (maximizedPane === null && !isGazeboMinimized);

  // Active URDF XML for ViewFrames modal inside Ros2VisualizerPane
  const activeUrdfXml =
    activeFile.path.endsWith('.urdf')
      ? activeFile.content
      : files.find((f) => f.path === 'urdf/diff_drive_rover.urdf')?.content;

  // Compute dynamic pane width styles for desktop flex layout (full width on mobile < 1024px)
  const getIdeStyle = (): React.CSSProperties => {
    if (!isDesktopViewport) return { width: '100%' };
    if (maximizedPane === 'ide') return { width: '100%' };
    if (!showRos2 && !showGazebo) return { width: '100%' };
    return { width: `${ideWidthPct}%` };
  };

  const getRos2Style = (): React.CSSProperties => {
    if (!isDesktopViewport) return { width: '100%' };
    if (maximizedPane === 'ros2') return { width: '100%' };
    if (!showGazebo && !showIde) return { width: '100%' };
    if (!showGazebo && showIde) return { flex: 1 };
    if (!showIde && showGazebo) return { width: `${ros2WidthPct}%` };
    return { width: `${ros2WidthPct}%` };
  };

  const isDualOutputActive =
    maximizedPane === null && isIdeMinimized && !isRos2Minimized && !isGazeboMinimized;
  const isTriPaneActive =
    maximizedPane === null && !isIdeMinimized && !isRos2Minimized && !isGazeboMinimized;

  return (
    <div className="flex flex-col w-screen h-screen bg-[#0d1117] text-slate-100 overflow-hidden select-none">
      {/* Top Global Workspace Header: ROS 2 Jazzy + ros_gz_bridge + Gazebo Harmonic */}
      <header className="h-10 bg-[#161b22] border-b border-[#30363d] px-3 flex items-center justify-between gap-3 shrink-0 z-30">
        {/* Left: Brand & Active Separation Status */}
        <div className="flex items-center gap-2.5 min-w-0">
          <div className="flex items-center gap-1.5 text-xs font-bold tracking-tight text-white shrink-0">
            <Globe className="w-4 h-4 text-orange-400" />
            <span>Gazebo Harmonic</span>
            <span className="text-slate-500 font-mono">⇄</span>
            <Share2 className="w-3.5 h-3.5 text-teal-400" />
            <span className="text-teal-300">ROS 2 Jazzy</span>
          </div>

          <span className="hidden xl:inline-flex items-center gap-1.5 px-2 py-0.5 rounded bg-[#21262d] border border-[#30363d] text-[10px] font-mono text-slate-300">
            <Radio
              className={`w-3 h-3 ${
                ros2State.isBridgeRunning
                  ? 'text-emerald-400 animate-pulse'
                  : 'text-amber-400'
              }`}
            />
            <span>
              ros_gz_bridge:{' '}
              {ros2State.isBridgeRunning ? 'ACTIVE' : 'STOPPED'} (
              {ros2State.bridgeRules.filter((r) => r.enabled).length}/
              {ros2State.bridgeRules.length} topics)
            </span>
          </span>
        </div>

        {/* Center: Workspace View Switcher (Dual Output Mode hiding Code vs Tri-Pane vs Split) */}
        <div
          role="toolbar"
          aria-label="Workspace Display Layout"
          className="flex items-center gap-1 bg-[#0d1117] p-0.5 rounded border border-[#30363d] overflow-x-auto"
        >
          <button
            type="button"
            data-testid="layout-dual-output-btn"
            onClick={activateDualOutputMode}
            title="Hide Code IDE and display both ROS 2 (RViz2 + rqt) and Gazebo 3D Simulator simultaneously side-by-side"
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-[11px] font-semibold transition-colors whitespace-nowrap ${
              isDualOutputActive
                ? 'bg-teal-500/20 text-teal-200 border border-teal-500/50 shadow-sm'
                : 'text-slate-400 hover:text-white hover:bg-[#21262d]'
            }`}
          >
            <EyeOff className="w-3.5 h-3.5 text-teal-400" />
            <span>Dual Output: ROS 2 + Gazebo (Hide Code)</span>
          </button>

          <button
            type="button"
            data-testid="layout-tri-pane-btn"
            onClick={activateTriPaneMode}
            title="Show Code IDE, ROS 2 Display, and Gazebo 3D Simulator simultaneously"
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-[11px] font-semibold transition-colors whitespace-nowrap ${
              isTriPaneActive
                ? 'bg-sky-500/20 text-sky-200 border border-sky-500/50 shadow-sm'
                : 'text-slate-400 hover:text-white hover:bg-[#21262d]'
            }`}
          >
            <Columns className="w-3.5 h-3.5 text-sky-400" />
            <span>Tri-Pane (Code + ROS 2 + Gazebo)</span>
          </button>

          <button
            type="button"
            data-testid="layout-ide-gazebo-btn"
            onClick={activateIdeAndGazeboMode}
            title="Show Code IDE + Gazebo Harmonic 3D Simulator"
            className={`hidden md:flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium transition-colors whitespace-nowrap ${
              maximizedPane === null &&
              !isIdeMinimized &&
              isRos2Minimized &&
              !isGazeboMinimized
                ? 'bg-orange-500/20 text-orange-200 border border-orange-500/40'
                : 'text-slate-400 hover:text-white hover:bg-[#21262d]'
            }`}
          >
            <Code2 className="w-3 h-3 text-orange-400" />
            <span>Code + Gazebo</span>
          </button>

          <button
            type="button"
            data-testid="layout-ide-ros2-btn"
            onClick={activateIdeAndRos2Mode}
            title="Show Code IDE + ROS 2 Visualizer"
            className={`hidden md:flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium transition-colors whitespace-nowrap ${
              maximizedPane === null &&
              !isIdeMinimized &&
              !isRos2Minimized &&
              isGazeboMinimized
                ? 'bg-teal-500/20 text-teal-200 border border-teal-500/40'
                : 'text-slate-400 hover:text-white hover:bg-[#21262d]'
            }`}
          >
            <Share2 className="w-3 h-3 text-teal-400" />
            <span>Code + ROS 2</span>
          </button>
        </div>

        {/* Right: Minimized Pane Restore Chips & Global Quick Actions */}
        <div className="flex items-center gap-1.5 shrink-0">
          {maximizedPane !== null && (
            <button
              type="button"
              data-testid="restore-maximized-pane-btn"
              onClick={() => setMaximizedPane(null)}
              className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-semibold bg-amber-500/20 text-amber-300 border border-amber-500/40 hover:bg-amber-500/30 transition-colors"
            >
              <Minimize2 className="w-3 h-3" />
              <span>Exit Fullscreen ({maximizedPane.toUpperCase()})</span>
            </button>
          )}

          {maximizedPane === null && isIdeMinimized && (
            <button
              type="button"
              data-testid="restore-ide-pane-btn"
              onClick={() => setIsIdeMinimized(false)}
              className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium bg-[#21262d] hover:bg-[#30363d] text-sky-300 border border-[#30363d] transition-colors"
              title="Restore Code Editor Pane"
            >
              <Code2 className="w-3 h-3" />
              <span>+ Show Code IDE</span>
            </button>
          )}

          {maximizedPane === null && isRos2Minimized && (
            <button
              type="button"
              data-testid="restore-ros2-pane-btn"
              onClick={() => setIsRos2Minimized(false)}
              className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium bg-[#21262d] hover:bg-[#30363d] text-teal-300 border border-[#30363d] transition-colors"
              title="Restore ROS 2 Display Pane"
            >
              <Share2 className="w-3 h-3" />
              <span>+ Show ROS 2</span>
            </button>
          )}

          {maximizedPane === null && isGazeboMinimized && (
            <button
              type="button"
              data-testid="restore-gazebo-pane-btn"
              onClick={() => setIsGazeboMinimized(false)}
              className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium bg-[#21262d] hover:bg-[#30363d] text-orange-300 border border-[#30363d] transition-colors"
              title="Restore Gazebo 3D Simulator Pane"
            >
              <Globe className="w-3 h-3" />
              <span>+ Show Gazebo</span>
            </button>
          )}

          <button
            type="button"
            data-testid="global-sim-play-pause-btn"
            onClick={() => setIsSimRunning((r) => !r)}
            className={`flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-semibold transition-colors ${
              isSimRunning
                ? 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/40'
                : 'bg-amber-500/20 text-amber-300 border border-amber-500/40'
            }`}
          >
            {isSimRunning ? (
              <>
                <Pause className="w-3 h-3" />
                <span className="hidden sm:inline">Running</span>
              </>
            ) : (
              <>
                <Play className="w-3 h-3" />
                <span className="hidden sm:inline">Paused</span>
              </>
            )}
          </button>
        </div>
      </header>

      {/* Main Multi-Pane Workspace Container */}
      <main
        ref={workspaceContainerRef}
        className="flex-1 flex flex-col lg:flex-row min-h-0 min-w-0 overflow-hidden relative"
      >
        {/* PANE 1: VS Code IDE (`worlds/`, `urdf/`, `ros2_ws/` folders + Ubuntu 24.04 & ROS 2 Jazzy CLI) */}
        {showIde && (
          <section
            aria-label="VS Code ROS 2 and Gazebo Editor"
            style={getIdeStyle()}
            className="h-full min-w-[280px] flex flex-col shrink-0 overflow-hidden"
          >
            <VsCodeIdePane
              files={files}
              activeFile={activeFile}
              onSelectFile={handleSelectFile}
              onUpdateFileContent={handleUpdateFileContent}
              onCreateFile={handleCreateFile}
              onRenameFile={handleRenameFile}
              onDuplicateFile={handleDuplicateFile}
              onDeleteFile={handleDeleteFile}
              onUploadFile={handleUploadFile}
              onDownloadActiveFile={handleDownloadActiveFile}
              compilationResult={compilationResult}
              onVerifyAndCompile={() => compileFileContent(activeFile, false)}
              onCompileAndRun={() => compileFileContent(activeFile, true)}
              terminalHistory={terminalHistory}
              terminalCwd={terminalCwd}
              onRunTerminalCommand={handleRunTerminalCommand}
              activeWorld={world}
              onUpdateJointTarget={handleUpdateJointTarget}
              onUpdateModelCmdVel={handleUpdateModelCmdVel}
              onUpdateWorldWind={handleUpdateWorldWind}
              onInsertSystemPlugin={handleInsertSystemPlugin}
              isPluginModalOpen={isPluginModalOpen}
              onSetPluginModalOpen={setIsPluginModalOpen}
              onMinimizePane={() => {
                if (maximizedPane === 'ide') setMaximizedPane(null);
                setIsIdeMinimized(true);
                if (isRos2Minimized && isGazeboMinimized) {
                  setIsRos2Minimized(false);
                  setIsGazeboMinimized(false);
                }
              }}
              isMaximized={maximizedPane === 'ide'}
              onToggleMaximizePane={() => toggleMaximizePane('ide')}
            />
          </section>
        )}

        {/* Draggable Splitter 1: Between Code IDE and ROS 2 / Gazebo */}
        {showIde && (showRos2 || showGazebo) && maximizedPane === null && (
          <div
            role="separator"
            aria-orientation="vertical"
            aria-label="Resize Code IDE Pane"
            data-testid="splitter-ide-ros2"
            onMouseDown={() => {
              draggingSplitterRef.current = 'ide-ros2';
              document.body.style.cursor = 'col-resize';
              document.body.style.userSelect = 'none';
            }}
            className="hidden lg:flex w-1.5 h-full bg-[#161b22] hover:bg-sky-500/60 cursor-col-resize items-center justify-center transition-colors shrink-0 z-20"
          >
            <div className="w-0.5 h-10 bg-slate-600 rounded-full" />
          </div>
        )}

        {/* PANE 2: ROS 2 Jazzy Visualizer (`RViz2 3D` + `rqt` `ros_gz_bridge` & Closed-Loop Nodes) */}
        {showRos2 && (
          <section
            aria-label="ROS 2 Jazzy RViz2 and ros_gz_bridge Display"
            style={getRos2Style()}
            className="h-full min-w-[300px] flex flex-col shrink-0 overflow-hidden"
          >
            <Ros2VisualizerPane
              world={world}
              ros2State={ros2State}
              stepTrigger={stepTrigger}
              activeUrdfXml={activeUrdfXml}
              onToggleBridgeRunning={handleToggleBridgeRunning}
              onToggleUseSimTime={handleToggleUseSimTime}
              onToggleBridgeRule={handleToggleBridgeRule}
              onCycleBridgeRuleDirection={handleCycleBridgeRuleDirection}
              onToggleRos2Node={handleToggleRos2Node}
              onPublishRosCmdVel={handlePublishRosCmdVel}
              onPublishRosJointCmd={handlePublishRosJointCmd}
              onOpenBridgeYamlInIde={handleOpenBridgeYamlInIde}
              onMinimizePane={() => {
                if (maximizedPane === 'ros2') setMaximizedPane(null);
                setIsRos2Minimized(true);
                if (isIdeMinimized && isGazeboMinimized) {
                  setIsGazeboMinimized(false);
                }
              }}
              isMaximized={maximizedPane === 'ros2'}
              onToggleMaximizePane={() => toggleMaximizePane('ros2')}
            />
          </section>
        )}

        {/* Draggable Splitter 2: Between ROS 2 Display and Gazebo Harmonic 3D Simulator */}
        {showRos2 && showGazebo && maximizedPane === null && (
          <div
            role="separator"
            aria-orientation="vertical"
            aria-label="Resize ROS 2 and Gazebo Simulator Panes"
            data-testid="splitter-ros2-gazebo"
            onMouseDown={() => {
              draggingSplitterRef.current = 'ros2-gazebo';
              document.body.style.cursor = 'col-resize';
              document.body.style.userSelect = 'none';
            }}
            className="hidden lg:flex w-1.5 h-full bg-[#161b22] hover:bg-teal-500/60 cursor-col-resize items-center justify-center transition-colors shrink-0 z-20"
          >
            <div className="w-0.5 h-10 bg-teal-500/70 rounded-full" />
          </div>
        )}

        {/* PANE 3: Gazebo Harmonic 3D Physics Simulator (`gz-sim8`) */}
        {showGazebo && (
          <section
            aria-label="Gazebo Harmonic 3D Physics Simulator"
            className="flex-1 min-w-[320px] h-full flex flex-col overflow-hidden"
          >
            <GazeboSimulatorPane
              world={world}
              isSimRunning={isSimRunning}
              onTogglePlay={() => setIsSimRunning((r) => !r)}
              onPauseSimulation={() => setIsSimRunning(false)}
              onStepSimulation={handleStepSimulation}
              onResetSimulation={handleResetSimulation}
              stepTrigger={stepTrigger}
              selectedEntityId={selectedEntityId}
              onSelectEntity={setSelectedEntityId}
              onUpdateEntityPose={handleUpdateEntityPose}
              onScaleEntity={handleScaleEntity}
              onDeleteEntity={handleDeleteEntity}
              onToggleModelProperty={handleToggleModelProperty}
              onSpawnPrimitiveAtPose={handleSpawnPrimitiveAtPose}
              onSpawnLightAtPose={handleSpawnLightAtPose}
              onPasteModelAtPose={handlePasteModelAtPose}
              onSyncSceneToEditor={handleSyncSceneToEditor}
              onUpdateJointTarget={handleUpdateJointTarget}
              gizmoMode={gizmoMode}
              onChangeGizmoMode={setGizmoMode}
              debugOverlays={debugOverlays}
              onToggleDebugOverlay={(key) =>
                setDebugOverlays((prev) => ({ ...prev, [key]: !prev[key] }))
              }
              simTime={simTime}
              realTime={realTime}
              rtf={rtf}
              iterations={iterations}
              onTelemetryUpdate={handleTelemetryUpdate}
              onOpenPluginBrowser={() => {
                setIsIdeMinimized(false);
                setIsPluginModalOpen(true);
              }}
              onMinimizePane={() => {
                if (maximizedPane === 'gazebo') setMaximizedPane(null);
                setIsGazeboMinimized(true);
                if (isIdeMinimized && isRos2Minimized) {
                  setIsRos2Minimized(false);
                }
              }}
              isMaximized={maximizedPane === 'gazebo'}
              onToggleMaximizePane={() => toggleMaximizePane('gazebo')}
            />
          </section>
        )}
      </main>
    </div>
  );
}

export default App;
