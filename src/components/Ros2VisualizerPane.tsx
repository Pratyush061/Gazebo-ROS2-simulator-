import React, { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  Activity,
  ArrowLeftRight,
  CheckCircle2,
  Clock,
  Compass,
  Cpu,
  Eye,
  FileCode,
  GitBranch,
  Maximize2,
  Minimize2,
  Minus,
  Navigation,
  Pause,
  Play,
  Radio,
  RefreshCw,
  Share2,
  Sliders,
  AlertTriangle,
} from 'lucide-react';
import {
  ParsedWorld,
  Pose6D,
  Ros2RuntimeState,
  RosGzBridgeDirection,
  RosGzBridgeRule,
} from '../types/simulation';
import {
  directionToCliSymbol,
  formatParameterBridgeCliArg,
} from '../utils/rosGzBridgeEngine';
import {
  computeModelLinkWorldPoses,
} from '../utils/physicsEngine';
import { poseToMatrix4 } from '../utils/sdfCompiler';
import { compileUrdf } from '../urdf/urdfCompiler';
import { STARTER_TEMPLATES } from '../urdf/templates';
import { ViewFramesModal } from './ViewFramesModal';

interface Ros2VisualizerPaneProps {
  world: ParsedWorld | null;
  ros2State: Ros2RuntimeState;
  stepTrigger: number;
  activeUrdfXml?: string;
  onToggleBridgeRunning: () => void;
  onToggleUseSimTime: () => void;
  onToggleBridgeRule: (ruleId: string) => void;
  onCycleBridgeRuleDirection: (ruleId: string) => void;
  onToggleRos2Node: (nodeKey: keyof Ros2RuntimeState['nodes']) => void;
  onPublishRosCmdVel: (linearX: number, linearY: number, angularZ: number) => void;
  onPublishRosJointCmd: (jointName: string, positionRad: number) => void;
  onOpenBridgeYamlInIde?: () => void;
  onMinimizePane?: () => void;
  isMaximized?: boolean;
  onToggleMaximizePane?: () => void;
}

function createRvizAxes(size: number): THREE.Group {
  const group = new THREE.Group();
  const radius = Math.max(0.004, size * 0.04);
  const cylGeo = new THREE.CylinderGeometry(radius, radius, size, 8);

  const matX = new THREE.MeshBasicMaterial({ color: 0xef4444 });
  const meshX = new THREE.Mesh(cylGeo, matX);
  meshX.rotation.z = -Math.PI / 2;
  meshX.position.x = size * 0.5;

  const matY = new THREE.MeshBasicMaterial({ color: 0x22c55e });
  const meshY = new THREE.Mesh(cylGeo, matY);
  meshY.position.y = size * 0.5;

  const matZ = new THREE.MeshBasicMaterial({ color: 0x3b82f6 });
  const meshZ = new THREE.Mesh(cylGeo, matZ);
  meshZ.rotation.x = Math.PI / 2;
  meshZ.position.z = size * 0.5;

  group.add(meshX, meshY, meshZ);
  return group;
}

export const Ros2VisualizerPane: React.FC<Ros2VisualizerPaneProps> = ({
  world,
  ros2State,
  stepTrigger,
  activeUrdfXml,
  onToggleBridgeRunning,
  onToggleUseSimTime,
  onToggleBridgeRule,
  onCycleBridgeRuleDirection,
  onToggleRos2Node,
  onPublishRosCmdVel,
  onPublishRosJointCmd,
  onOpenBridgeYamlInIde,
  onMinimizePane,
  isMaximized = false,
  onToggleMaximizePane,
}) => {
  const [subViewMode, setSubViewMode] = useState<'split' | 'rviz' | 'rqt'>('split');
  const [fixedFrame, setFixedFrame] = useState<'odom' | 'base_footprint'>('odom');
  const [rvizDisplays, setRvizDisplays] = useState({
    grid: true,
    tf: true,
    laserScan: true,
    odometry: true,
    robotModel: true,
  });
  const [viewFramesOpen, setViewFramesOpen] = useState(false);

  const mountRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const dynamicGroupRef = useRef<THREE.Group | null>(null);
  const gridGroupRef = useRef<THREE.Group | null>(null);

  // Compile URDF model via existing /src/urdf/urdfCompiler.ts for tf2_tools view_frames modal
  const compiledUrdfForFrames = useMemo(() => {
    const fallbackXml =
      activeUrdfXml && activeUrdfXml.includes('<robot')
        ? activeUrdfXml
        : STARTER_TEMPLATES[1]?.urdfXml || STARTER_TEMPLATES[0].urdfXml;
    return compileUrdf(fallbackXml);
  }, [activeUrdfXml]);

  // Initialize RViz2 3D WebGL Canvas (Z-Up ROS 2 coordinate convention)
  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const width = Math.max(1, container.clientWidth || 480);
    const height = Math.max(1, container.clientHeight || 320);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f172a);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(48, width / height, 0.05, 80);
    camera.up.set(0, 0, 1);
    camera.position.set(2.4, -2.5, 1.85);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      preserveDrawingBuffer: true,
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.innerHTML = '';
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const ambient = new THREE.AmbientLight(0xdbeafe, 0.85);
    scene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
    dirLight.position.set(4, -3, 6);
    scene.add(dirLight);

    // Z-Up RViz2 Grid
    const gridGroup = new THREE.Group();
    const gridHelper = new THREE.GridHelper(12, 12, 0x38bdf8, 0x1e293b);
    gridHelper.rotation.x = Math.PI / 2;
    gridGroup.add(gridHelper);
    const originAxes = createRvizAxes(0.45);
    originAxes.position.z = 0.002;
    gridGroup.add(originAxes);
    scene.add(gridGroup);
    gridGroupRef.current = gridGroup;

    const dynGroup = new THREE.Group();
    scene.add(dynGroup);
    dynamicGroupRef.current = dynGroup;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.25);
    controls.enableDamping = false;
    controls.update();

    const handleChange = () => {
      renderer.render(scene, camera);
    };
    controls.addEventListener('change', handleChange);

    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const w = Math.floor(entry.contentRect.width);
        const h = Math.floor(entry.contentRect.height);
        if (w > 0 && h > 0) {
          camera.aspect = w / h;
          camera.updateProjectionMatrix();
          renderer.setSize(w, h);
          renderer.render(scene, camera);
        }
      }
    });
    ro.observe(container);

    renderer.render(scene, camera);

    return () => {
      ro.disconnect();
      controls.removeEventListener('change', handleChange);
      controls.dispose();
      renderer.dispose();
    };
  }, [subViewMode]);

  // Update RViz2 3D scene exclusively from bridged ROS 2 telemetry (`ros2State.bridgedTelemetry`)
  useEffect(() => {
    const scene = sceneRef.current;
    const camera = cameraRef.current;
    const renderer = rendererRef.current;
    const dynGroup = dynamicGroupRef.current;
    const gridGroup = gridGroupRef.current;
    if (!scene || !camera || !renderer || !dynGroup) return;

    if (gridGroup) {
      gridGroup.visible = rvizDisplays.grid;
    }

    // Clear previous frame's dynamic objects
    while (dynGroup.children.length > 0) {
      const child = dynGroup.children.pop()!;
      dynGroup.remove(child);
    }

    const heroModel =
      world?.models.find((m) => m.name === ros2State.targetModelName) ||
      world?.models.find((m) => m.name !== 'ground_plane');

    const odomPose: Pose6D = ros2State.bridgedTelemetry.odom?.pose || [0, 0, 0, 0, 0, 0];
    const frameOffset: [number, number, number] =
      fixedFrame === 'base_footprint'
        ? [-odomPose[0], -odomPose[1], 0]
        : [0, 0, 0];

    // 1. Bridged RobotModel & TF2 Frames (only when /tf or /odom is bridged!)
    if (
      heroModel &&
      (ros2State.bridgedTelemetry.tfReceived || ros2State.bridgedTelemetry.odomReceived)
    ) {
      const linkWorldPoses = computeModelLinkWorldPoses(heroModel);

      for (const link of heroModel.links) {
        const lPose = linkWorldPoses.get(link.name) || heroModel.pose;
        const linkGroup = new THREE.Group();
        linkGroup.position.set(
          lPose[0] + frameOffset[0],
          lPose[1] + frameOffset[1],
          lPose[2] + frameOffset[2]
        );
        const euler = new THREE.Euler(lPose[3], lPose[4], lPose[5], 'ZYX');
        linkGroup.quaternion.setFromEuler(euler);

        if (rvizDisplays.tf) {
          const tfAxes = createRvizAxes(0.22);
          linkGroup.add(tfAxes);
        }

        if (rvizDisplays.robotModel) {
          for (const vis of link.visuals) {
            let geom: THREE.BufferGeometry;
            if (vis.geometry.type === 'box') {
              const [sx, sy, sz] = vis.geometry.size || [0.2, 0.2, 0.2];
              geom = new THREE.BoxGeometry(sx, sy, sz);
            } else if (vis.geometry.type === 'cylinder') {
              const r = vis.geometry.radius ?? 0.15;
              const l = vis.geometry.length ?? 0.2;
              geom = new THREE.CylinderGeometry(r, r, l, 20);
              geom.rotateX(Math.PI / 2);
            } else if (vis.geometry.type === 'sphere') {
              geom = new THREE.SphereGeometry(vis.geometry.radius ?? 0.15, 16, 16);
            } else {
              geom = new THREE.BoxGeometry(0.2, 0.2, 0.2);
            }

            const [r, g, b, a] = vis.material.diffuse;
            const mat = new THREE.MeshStandardMaterial({
              color: new THREE.Color(r, g, b),
              roughness: 0.35,
              metalness: 0.25,
              transparent: a < 1,
              opacity: a,
            });
            const mesh = new THREE.Mesh(geom, mat);
            mesh.position.set(vis.pose[0], vis.pose[1], vis.pose[2]);
            const vEuler = new THREE.Euler(vis.pose[3], vis.pose[4], vis.pose[5], 'ZYX');
            mesh.quaternion.setFromEuler(vEuler);
            linkGroup.add(mesh);
          }
        }

        dynGroup.add(linkGroup);
      }
    }

    // 2. Bridged /odom Trajectory Path & Pose Arrow
    if (
      rvizDisplays.odometry &&
      ros2State.bridgedTelemetry.odomReceived &&
      ros2State.bridgedTelemetry.odom
    ) {
      const [ox, oy, oz, , , oyaw] = ros2State.bridgedTelemetry.odom.pose;
      const dir = new THREE.Vector3(Math.cos(oyaw), Math.sin(oyaw), 0).normalize();
      const origin = new THREE.Vector3(
        ox + frameOffset[0],
        oy + frameOffset[1],
        oz + frameOffset[2] + 0.04
      );
      const arrow = new THREE.ArrowHelper(dir, origin, 0.65, 0xf97316, 0.18, 0.1);
      dynGroup.add(arrow);

      if (ros2State.odometryTrail.length >= 2) {
        const pts = ros2State.odometryTrail.map(
          ([tx, ty, tz]) =>
            new THREE.Vector3(
              tx + frameOffset[0],
              ty + frameOffset[1],
              tz + frameOffset[2] + 0.02
            )
        );
        const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
        const lineMat = new THREE.LineBasicMaterial({ color: 0x38bdf8 });
        dynGroup.add(new THREE.Line(lineGeo, lineMat));
      }
    }

    // 3. Bridged /scan (sensor_msgs/msg/LaserScan) 3D Point Cloud & Rays
    if (
      rvizDisplays.laserScan &&
      ros2State.bridgedTelemetry.scanReceived &&
      ros2State.bridgedTelemetry.scan
    ) {
      const scan = ros2State.bridgedTelemetry.scan;
      const sPose: Pose6D = [
        scan.sensorWorldPose[0] + frameOffset[0],
        scan.sensorWorldPose[1] + frameOffset[1],
        scan.sensorWorldPose[2] + frameOffset[2],
        scan.sensorWorldPose[3],
        scan.sensorWorldPose[4],
        scan.sensorWorldPose[5],
      ];
      const mat = poseToMatrix4(sPose);
      const n = scan.ranges.length;
      const hitPositions: number[] = [];
      const rayPositions: number[] = [];

      for (let i = 0; i < n; i++) {
        const dist = scan.ranges[i];
        const angle =
          n > 1
            ? scan.minAngle + (i / (n - 1)) * (scan.maxAngle - scan.minAngle)
            : 0;
        const lx = dist * Math.cos(angle);
        const ly = dist * Math.sin(angle);
        const wx = mat[0] * lx + mat[1] * ly + mat[3];
        const wy = mat[4] * lx + mat[5] * ly + mat[7];
        const wz = mat[8] * lx + mat[9] * ly + mat[11];

        rayPositions.push(sPose[0], sPose[1], sPose[2], wx, wy, wz);
        if (dist < scan.maxRange - 0.05) {
          hitPositions.push(wx, wy, wz);
        }
      }

      if (rayPositions.length > 0) {
        const rayGeo = new THREE.BufferGeometry();
        rayGeo.setAttribute(
          'position',
          new THREE.Float32BufferAttribute(rayPositions, 3)
        );
        const rayMat = new THREE.LineBasicMaterial({
          color: 0x06b6d4,
          transparent: true,
          opacity: 0.32,
        });
        dynGroup.add(new THREE.LineSegments(rayGeo, rayMat));
      }

      if (hitPositions.length > 0) {
        const ptsGeo = new THREE.BufferGeometry();
        ptsGeo.setAttribute(
          'position',
          new THREE.Float32BufferAttribute(hitPositions, 3)
        );
        const ptsMat = new THREE.PointsMaterial({
          color: 0x22c55e,
          size: 0.09,
        });
        dynGroup.add(new THREE.Points(ptsGeo, ptsMat));
      }
    }

    renderer.render(scene, camera);
  }, [world, ros2State, stepTrigger, fixedFrame, rvizDisplays]);

  const activeBridgedCount = ros2State.bridgeRules.filter(
    (r) => ros2State.isBridgeRunning && r.enabled
  ).length;

  const directionBadgeStyle = (dir: RosGzBridgeDirection) => {
    if (dir === 'GZ_TO_ROS') {
      return 'bg-sky-500/15 text-sky-300 border-sky-500/40';
    }
    if (dir === 'ROS_TO_GZ') {
      return 'bg-amber-500/15 text-amber-300 border-amber-500/40';
    }
    return 'bg-emerald-500/15 text-emerald-300 border-emerald-500/40';
  };

  return (
    <div
      data-testid="ros2-visualizer-pane"
      className="flex flex-col h-full w-full bg-[#0f172a] text-slate-100 select-none overflow-hidden border-r border-[#2d2d30]"
    >
      {/* Top ROS 2 Jazzy & ros_gz_bridge Header Bar */}
      <div className="flex flex-wrap items-center justify-between gap-2 px-3 py-1.5 min-h-[42px] bg-[#161f30] border-b border-[#26334d] shrink-0">
        <div className="flex items-center gap-2 min-w-0">
          <div className="w-2.5 h-2.5 rounded-full bg-teal-400 shadow-[0_0_8px_#2dd4bf] shrink-0" />
          <span className="text-xs font-bold tracking-tight text-white whitespace-nowrap">
            ROS 2 Jazzy · RViz2 &amp; ros_gz_bridge
          </span>

          {/* Master parameter_bridge Active / Stopped Toggle */}
          <button
            type="button"
            data-testid="toggle-ros-gz-bridge-btn"
            onClick={onToggleBridgeRunning}
            className={`flex items-center gap-1.5 px-2.5 py-0.5 rounded text-[11px] font-semibold border transition-colors whitespace-nowrap shrink-0 ${
              ros2State.isBridgeRunning
                ? 'bg-emerald-500/20 text-emerald-300 border-emerald-500/50 hover:bg-emerald-500/30'
                : 'bg-rose-500/20 text-rose-300 border-rose-500/50 hover:bg-rose-500/30'
            }`}
            title="Start or Stop the official ros_gz_bridge (parameter_bridge) node"
          >
            <ArrowLeftRight className="w-3 h-3" />
            <span>
              parameter_bridge: {ros2State.isBridgeRunning ? 'ACTIVE' : 'STOPPED'} (
              {activeBridgedCount}/{ros2State.bridgeRules.length})
            </span>
          </button>

          {/* use_sim_time /clock Sync Badge */}
          <button
            type="button"
            data-testid="toggle-use-sim-time-btn"
            onClick={onToggleUseSimTime}
            className={`hidden xl:flex items-center gap-1 px-2 py-0.5 rounded text-[11px] font-mono border transition-colors whitespace-nowrap shrink-0 ${
              ros2State.useSimTime
                ? 'bg-sky-500/15 text-sky-300 border-sky-500/40'
                : 'bg-slate-800 text-slate-400 border-slate-700'
            }`}
            title="Toggle ROS 2 use_sim_time (/clock synchronization)"
          >
            <Clock className="w-3 h-3" />
            <span>
              /clock: {ros2State.rosClockSec.toFixed(2)}s (use_sim_time=
              {ros2State.useSimTime ? 'true' : 'false'})
            </span>
          </button>
        </div>

        {/* Right Controls: Sub-View Mode + Minimize / Maximize Buttons */}
        <div className="flex items-center gap-1 shrink-0">
          <div className="flex items-center bg-[#0f172a] border border-[#26334d] rounded p-0.5 text-[11px]">
            {(
              [
                { id: 'split', label: 'RViz2 + rqt' },
                { id: 'rviz', label: 'RViz2 3D' },
                { id: 'rqt', label: 'rqt Bridge' },
              ] as const
            ).map((tab) => (
              <button
                key={tab.id}
                type="button"
                data-testid={`ros2-subview-${tab.id}`}
                onClick={() => setSubViewMode(tab.id)}
                className={`px-2 py-0.5 rounded font-medium transition-colors whitespace-nowrap ${
                  subViewMode === tab.id
                    ? 'bg-sky-500 text-slate-950 font-semibold'
                    : 'text-slate-300 hover:text-white'
                }`}
              >
                {tab.label}
              </button>
            ))}
          </div>

          <button
            type="button"
            onClick={() => setViewFramesOpen(true)}
            className="hidden sm:flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium bg-[#1e293b] hover:bg-[#334155] text-sky-300 border border-sky-500/30 transition-colors whitespace-nowrap"
            title="Open ROS 2 tf2_tools view_frames Diagram & PDF Exporter"
          >
            <GitBranch className="w-3 h-3" />
            <span>view_frames</span>
          </button>

          {onMinimizePane && (
            <button
              type="button"
              data-testid="minimize-ros2-pane-btn"
              onClick={onMinimizePane}
              className="p-1 rounded bg-[#1e293b] hover:bg-[#334155] text-slate-300 hover:text-white transition-colors"
              title="Minimize ROS 2 Display Pane"
              aria-label="Minimize ROS 2 Display Pane"
            >
              <Minus className="w-3.5 h-3.5" />
            </button>
          )}

          {onToggleMaximizePane && (
            <button
              type="button"
              data-testid="maximize-ros2-pane-btn"
              onClick={onToggleMaximizePane}
              className="p-1 rounded bg-[#1e293b] hover:bg-[#334155] text-slate-300 hover:text-white transition-colors"
              title={
                isMaximized
                  ? 'Restore ROS 2 Display Pane Size'
                  : 'Maximize ROS 2 Display Pane'
              }
              aria-label={
                isMaximized
                  ? 'Restore ROS 2 Display Pane Size'
                  : 'Maximize ROS 2 Display Pane'
              }
            >
              {isMaximized ? (
                <Minimize2 className="w-3.5 h-3.5 text-sky-400" />
              ) : (
                <Maximize2 className="w-3.5 h-3.5" />
              )}
            </button>
          )}
        </div>
      </div>

      {/* Main Content Body: RViz2 3D Viewport (Top) + rqt Bridge & Nodes Console (Bottom) */}
      <div className="flex-1 flex flex-col min-h-0 overflow-hidden">
        {/* Top Section: Interactive 3D RViz2 Viewport */}
        {(subViewMode === 'split' || subViewMode === 'rviz') && (
          <div
            className={`relative w-full min-h-[220px] overflow-hidden ${
              subViewMode === 'split' ? 'h-[52%] border-b border-[#26334d]' : 'flex-1'
            }`}
          >
            <div
              ref={mountRef}
              data-testid="ros2-rviz-canvas"
              className="w-full h-full cursor-grab active:cursor-grabbing"
            />

            {/* Top-Left RViz2 Global Options & Display Toggles Overlay */}
            <div className="absolute top-2.5 left-2.5 z-10 flex flex-wrap items-center gap-1.5 bg-[#111827]/90 border border-[#26334d] rounded-md px-2 py-1 text-[11px] pointer-events-auto shadow">
              <div className="flex items-center gap-1 pr-1.5 border-r border-[#26334d]">
                <Compass className="w-3.5 h-3.5 text-sky-400" />
                <span className="text-slate-400">Fixed Frame:</span>
                <select
                  aria-label="RViz2 Fixed Frame"
                  value={fixedFrame}
                  onChange={(e) =>
                    setFixedFrame(e.target.value as 'odom' | 'base_footprint')
                  }
                  className="bg-[#1e293b] text-sky-300 font-mono text-[11px] rounded px-1.5 py-0.5 border border-[#334155]"
                >
                  <option value="odom">odom</option>
                  <option value="base_footprint">base_footprint</option>
                </select>
              </div>

              {(
                [
                  { key: 'tf', label: 'TF2 Axes' },
                  { key: 'laserScan', label: 'LaserScan (/scan)' },
                  { key: 'odometry', label: 'Odometry (/odom)' },
                  { key: 'robotModel', label: 'RobotModel' },
                ] as const
              ).map((d) => (
                <button
                  key={d.key}
                  type="button"
                  onClick={() =>
                    setRvizDisplays((prev) => ({
                      ...prev,
                      [d.key]: !prev[d.key],
                    }))
                  }
                  className={`px-1.5 py-0.5 rounded font-medium transition-colors ${
                    rvizDisplays[d.key]
                      ? 'bg-sky-500/20 text-sky-300 border border-sky-500/40'
                      : 'bg-slate-800/70 text-slate-400'
                  }`}
                >
                  {d.label}
                </button>
              ))}
            </div>

            {/* Bottom-Left Live Bridged Telemetry Status Strip */}
            <div className="absolute bottom-2 left-2.5 right-2.5 z-10 flex flex-wrap items-center justify-between gap-2 bg-[#111827]/95 border border-[#26334d] rounded px-2.5 py-1 text-[11px] font-mono pointer-events-auto">
              {!ros2State.isBridgeRunning ? (
                <div
                  data-testid="rviz-bridge-stopped-warning"
                  className="flex items-center gap-1.5 text-rose-400 font-semibold"
                >
                  <AlertTriangle className="w-3.5 h-3.5 shrink-0" />
                  <span>
                    ros_gz_bridge is STOPPED — No Gazebo messages crossing into ROS 2!
                  </span>
                </div>
              ) : (
                <div className="flex flex-wrap items-center gap-3 text-slate-300">
                  <span className="flex items-center gap-1">
                    <span
                      className={`w-2 h-2 rounded-full ${
                        ros2State.bridgedTelemetry.scanReceived
                          ? 'bg-emerald-400'
                          : 'bg-rose-400'
                      }`}
                    />
                    <span>
                      /scan:{' '}
                      {ros2State.bridgedTelemetry.scanReceived
                        ? `${ros2State.bridgedTelemetry.scan?.ranges.length || 0} rays`
                        : 'UNBRIDGED'}
                    </span>
                  </span>
                  <span className="flex items-center gap-1">
                    <span
                      className={`w-2 h-2 rounded-full ${
                        ros2State.bridgedTelemetry.odomReceived
                          ? 'bg-emerald-400'
                          : 'bg-rose-400'
                      }`}
                    />
                    <span>
                      /odom:{' '}
                      {ros2State.bridgedTelemetry.odomReceived &&
                      ros2State.bridgedTelemetry.odom
                        ? `[${ros2State.bridgedTelemetry.odom.pose[0].toFixed(2)}, ${ros2State.bridgedTelemetry.odom.pose[1].toFixed(2)}]`
                        : 'UNBRIDGED'}
                    </span>
                  </span>
                  <span className="flex items-center gap-1">
                    <span
                      className={`w-2 h-2 rounded-full ${
                        ros2State.bridgedTelemetry.tfReceived
                          ? 'bg-emerald-400'
                          : 'bg-rose-400'
                      }`}
                    />
                    <span>
                      /tf: {ros2State.bridgedTelemetry.tfTree.length} frames
                    </span>
                  </span>
                </div>
              )}

              <span className="text-sky-400">
                Target: {ros2State.targetModelName}
              </span>
            </div>
          </div>
        )}

        {/* Bottom Section: rqt Bridge Table & Active ROS 2 Nodes */}
        {(subViewMode === 'split' || subViewMode === 'rqt') && (
          <div className="flex-1 min-h-0 overflow-y-auto p-3 space-y-3 bg-[#0b1120] text-xs">
            {/* Section 1: Closed-Loop ROS 2 Nodes & ROS_TO_GZ Twist Publisher */}
            <div className="grid grid-cols-1 xl:grid-cols-2 gap-2.5">
              {/* Active ROS 2 Nodes Card */}
              <div className="bg-[#111827] border border-[#26334d] rounded-lg p-2.5 space-y-2">
                <div className="flex items-center justify-between">
                  <span className="font-semibold text-white flex items-center gap-1.5">
                    <Cpu className="w-3.5 h-3.5 text-teal-400" />
                    <span>Active ROS 2 Jazzy Nodes (rclpy / rclcpp)</span>
                  </span>
                  <span className="text-[10.5px] font-mono text-slate-400">
                    DDS Domain ID: 0
                  </span>
                </div>

                <div className="grid grid-cols-1 sm:grid-cols-2 gap-1.5">
                  <button
                    type="button"
                    data-testid="toggle-node-rsp"
                    onClick={() => onToggleRos2Node('robotStatePublisher')}
                    className={`flex items-center justify-between px-2.5 py-1.5 rounded border text-left transition-colors ${
                      ros2State.nodes.robotStatePublisher
                        ? 'bg-teal-500/15 border-teal-500/40 text-teal-200'
                        : 'bg-[#161f30] border-[#26334d] text-slate-400'
                    }`}
                  >
                    <div className="min-w-0">
                      <div className="font-mono text-[11px] font-semibold truncate">
                        /robot_state_publisher
                      </div>
                      <div className="text-[10px] text-slate-400 truncate">
                        /joint_states &rarr; /tf
                      </div>
                    </div>
                    <span className="font-mono text-[10px] px-1.5 py-0.5 rounded bg-black/30">
                      {ros2State.nodes.robotStatePublisher ? 'ON' : 'OFF'}
                    </span>
                  </button>

                  <button
                    type="button"
                    data-testid="toggle-node-obstacle-avoidance"
                    onClick={() => onToggleRos2Node('lidarObstacleAvoidance')}
                    className={`flex items-center justify-between px-2.5 py-1.5 rounded border text-left transition-colors ${
                      ros2State.nodes.lidarObstacleAvoidance
                        ? 'bg-sky-500/20 border-sky-400/60 text-sky-200'
                        : 'bg-[#161f30] border-[#26334d] text-slate-400'
                    }`}
                  >
                    <div className="min-w-0">
                      <div className="font-mono text-[11px] font-semibold truncate">
                        /lidar_obstacle_avoidance
                      </div>
                      <div className="text-[10px] text-slate-400 truncate">
                        Closed-Loop /scan &rarr; /cmd_vel
                      </div>
                    </div>
                    <span className="font-mono text-[10px] px-1.5 py-0.5 rounded bg-black/30">
                      {ros2State.nodes.lidarObstacleAvoidance ? 'ACTIVE' : 'IDLE'}
                    </span>
                  </button>

                  <button
                    type="button"
                    data-testid="toggle-node-joint-trajectory"
                    onClick={() => onToggleRos2Node('jointTrajectoryPublisher')}
                    className={`flex items-center justify-between px-2.5 py-1.5 rounded border text-left transition-colors ${
                      ros2State.nodes.jointTrajectoryPublisher
                        ? 'bg-purple-500/20 border-purple-400/60 text-purple-200'
                        : 'bg-[#161f30] border-[#26334d] text-slate-400'
                    }`}
                  >
                    <div className="min-w-0">
                      <div className="font-mono text-[11px] font-semibold truncate">
                        /joint_trajectory_node
                      </div>
                      <div className="text-[10px] text-slate-400 truncate">
                        Sinusoidal wave &rarr; /joint_cmd
                      </div>
                    </div>
                    <span className="font-mono text-[10px] px-1.5 py-0.5 rounded bg-black/30">
                      {ros2State.nodes.jointTrajectoryPublisher ? 'WAVE' : 'OFF'}
                    </span>
                  </button>

                  <div className="flex flex-col justify-center px-2.5 py-1 rounded bg-[#0f172a] border border-[#26334d] font-mono text-[10.5px]">
                    <span className="text-slate-400">Avoidance Status:</span>
                    <span
                      data-testid="avoidance-node-status"
                      className="text-sky-300 truncate"
                      title={ros2State.lastAvoidanceStatus}
                    >
                      {ros2State.lastAvoidanceStatus}
                    </span>
                  </div>
                </div>
              </div>

              {/* ROS 2 /cmd_vel (ROS_TO_GZ `]`) Teleop Publisher Card */}
              <div className="bg-[#111827] border border-[#26334d] rounded-lg p-2.5 flex flex-col justify-between space-y-2">
                <div className="flex items-center justify-between">
                  <span className="font-semibold text-white flex items-center gap-1.5">
                    <Navigation className="w-3.5 h-3.5 text-amber-400" />
                    <span>ROS 2 /cmd_vel Publisher (ROS_TO_GZ `]`)</span>
                  </span>
                  <span className="font-mono text-[10.5px] text-amber-300">
                    geometry_msgs/msg/Twist
                  </span>
                </div>

                <div className="flex flex-wrap items-center gap-1.5">
                  <button
                    type="button"
                    data-testid="ros2-cmdvel-fwd"
                    onClick={() => onPublishRosCmdVel(1.0, 0, 0)}
                    className="px-2.5 py-1 rounded bg-[#1e293b] hover:bg-sky-600 text-white font-mono text-[11px] border border-[#334155] transition-colors"
                  >
                    Fwd (+1.0 m/s)
                  </button>
                  <button
                    type="button"
                    data-testid="ros2-cmdvel-left"
                    onClick={() => onPublishRosCmdVel(0.5, 0, 1.0)}
                    className="px-2.5 py-1 rounded bg-[#1e293b] hover:bg-sky-600 text-white font-mono text-[11px] border border-[#334155] transition-colors"
                  >
                    Arc Left (+1.0 rad/s)
                  </button>
                  <button
                    type="button"
                    data-testid="ros2-cmdvel-right"
                    onClick={() => onPublishRosCmdVel(0.5, 0, -1.0)}
                    className="px-2.5 py-1 rounded bg-[#1e293b] hover:bg-sky-600 text-white font-mono text-[11px] border border-[#334155] transition-colors"
                  >
                    Arc Right (-1.0 rad/s)
                  </button>
                  <button
                    type="button"
                    data-testid="ros2-cmdvel-rev"
                    onClick={() => onPublishRosCmdVel(-0.8, 0, 0)}
                    className="px-2.5 py-1 rounded bg-[#1e293b] hover:bg-sky-600 text-white font-mono text-[11px] border border-[#334155] transition-colors"
                  >
                    Rev (-0.8 m/s)
                  </button>
                  <button
                    type="button"
                    data-testid="ros2-cmdvel-stop"
                    onClick={() => onPublishRosCmdVel(0, 0, 0)}
                    className="px-2.5 py-1 rounded bg-rose-950/70 hover:bg-rose-800 text-rose-200 font-mono text-[11px] border border-rose-700/50 transition-colors"
                  >
                    Stop (0.0)
                  </button>
                </div>

                <div className="flex items-center justify-between text-[10.5px] font-mono text-slate-400 bg-[#0f172a] px-2 py-1 rounded border border-[#1e293b]">
                  <span>
                    Published Twist: lin.x=
                    {ros2State.pendingRosCmdVel.linearX.toFixed(2)}, ang.z=
                    {ros2State.pendingRosCmdVel.angularZ.toFixed(2)}
                  </span>
                  {onOpenBridgeYamlInIde && (
                    <button
                      type="button"
                      onClick={onOpenBridgeYamlInIde}
                      className="text-sky-400 hover:underline flex items-center gap-1"
                    >
                      <FileCode className="w-3 h-3" />
                      <span>ros2_ws/ros_gz_bridge.yaml</span>
                    </button>
                  )}
                </div>
              </div>
            </div>

            {/* Section 2: Official ros_gz_bridge (parameter_bridge) Directional Mapping Table */}
            <div className="bg-[#111827] border border-[#26334d] rounded-lg overflow-hidden">
              <div className="flex flex-wrap items-center justify-between gap-2 px-3 py-2 bg-[#161f30] border-b border-[#26334d]">
                <div className="flex items-center gap-2">
                  <Share2 className="w-3.5 h-3.5 text-sky-400" />
                  <span className="font-semibold text-white">
                    Official ros_gz_bridge (parameter_bridge) Channels
                  </span>
                </div>
                <div className="flex items-center gap-2 text-[10.5px] font-mono text-slate-300">
                  <span className="px-1.5 py-0.5 rounded bg-sky-500/15 text-sky-300">
                    [ GZ_TO_ROS
                  </span>
                  <span className="px-1.5 py-0.5 rounded bg-amber-500/15 text-amber-300">
                    ] ROS_TO_GZ
                  </span>
                  <span className="px-1.5 py-0.5 rounded bg-emerald-500/15 text-emerald-300">
                    @ BIDIRECTIONAL
                  </span>
                </div>
              </div>

              <div className="divide-y divide-[#1e293b] font-mono text-[11px]">
                {ros2State.bridgeRules.map((rule: RosGzBridgeRule) => {
                  const sym = directionToCliSymbol(rule.direction);
                  return (
                    <div
                      key={rule.id}
                      data-testid={`bridge-rule-row-${rule.rosTopic}`}
                      className={`px-3 py-2 flex flex-wrap items-center justify-between gap-2 transition-colors ${
                        rule.enabled && ros2State.isBridgeRunning
                          ? 'bg-[#111827]'
                          : 'bg-[#0b1120]/60 opacity-60'
                      }`}
                    >
                      <div className="flex items-center gap-2 min-w-0 flex-1">
                        <input
                          type="checkbox"
                          data-testid={`toggle-bridge-rule-${rule.rosTopic}`}
                          checked={rule.enabled}
                          onChange={() => onToggleBridgeRule(rule.id)}
                          aria-label={`Enable bridge rule for ${rule.rosTopic}`}
                          className="rounded border-slate-600 accent-sky-400 cursor-pointer shrink-0"
                        />
                        <div className="min-w-0 flex-1">
                          <div className="flex items-center gap-1.5 flex-wrap">
                            <span className="text-sky-300 font-semibold">
                              {rule.rosTopic}
                            </span>
                            <button
                              type="button"
                              data-testid={`cycle-bridge-dir-${rule.rosTopic}`}
                              onClick={() => onCycleBridgeRuleDirection(rule.id)}
                              className={`px-1.5 py-0.5 rounded border text-[10px] font-bold transition-colors ${directionBadgeStyle(
                                rule.direction
                              )}`}
                              title="Click to cycle official parameter_bridge direction: [ (GZ_TO_ROS), ] (ROS_TO_GZ), @ (BIDIRECTIONAL)"
                            >
                              {sym} {rule.direction}
                            </button>
                            <span className="text-amber-300 truncate">
                              {rule.gzTopic}
                            </span>
                          </div>
                          <div className="text-[10px] text-slate-400 truncate mt-0.5">
                            {formatParameterBridgeCliArg(rule)} · QoS:{' '}
                            <span className="text-slate-300">{rule.qosProfile}</span> (
                            {rule.qosProfile === 'sensor_data'
                              ? 'BestEffort'
                              : 'Reliable'}
                            )
                          </div>
                        </div>
                      </div>

                      <div className="flex items-center gap-3 text-right shrink-0">
                        <div className="text-[10px] text-slate-400 max-w-[210px] truncate">
                          {rule.lastPayloadSummary}
                        </div>
                        <span className="px-1.5 py-0.5 rounded bg-[#1e293b] text-emerald-300 text-[10px] tabular-nums">
                          {ros2State.isBridgeRunning && rule.enabled
                            ? `${rule.rateHz} Hz`
                            : '0 Hz'}
                        </span>
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        )}
      </div>

      {/* ROS 2 tf2_tools view_frames Modal (from /src/components/ViewFramesModal.tsx) */}
      <ViewFramesModal
        open={viewFramesOpen}
        onClose={() => setViewFramesOpen(false)}
        model={compiledUrdfForFrames}
      />
    </div>
  );
};
