import React, { useEffect, useMemo, useState } from 'react';
import {
  AlertTriangle,
  Compass,
  FileDown,
  Maximize2,
  Minimize2,
  PanelLeftOpen,
  Radio,
  Share2,
} from 'lucide-react';
import { ParsedWorld, Ros2RuntimeState } from '../types/simulation';
import {
  compileUrdf,
  computeForwardKinematics,
  resolveMimicJointValues,
} from '../urdf/urdfCompiler';
import { getDefaultBuiltInMeshes, STARTER_TEMPLATES } from '../urdf/templates';
import {
  CustomMeshAsset,
  ParsedUrdfModel,
  RvizTreeDisplaySettings,
  UrdfJoint,
  ViewportLayers,
} from '../urdf/types';
import { downloadViewFramesPdf } from '../urdf/viewFramesPdf';
import { RvizSensorData, RvizViewport } from './RvizViewport';
import { BottomRvizDock } from './BottomRvizDock';
import {
  DEFAULT_ROS2_DISPLAY_TOPICS,
  Ros2DisplayTopics,
  Ros2DisplaysPanel,
} from './Ros2DisplaysPanel';
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

const INITIAL_LAYERS: ViewportLayers = {
  visual: true,
  collision: false,
  tfFrames: true,
  tfLabels: false,
  jointAxes: false,
  centerOfMass: false,
  inertiaBoxes: false,
  grid: true,
  wireframeVisual: false,
  laserScan: true,
  odometry: true,
  imu: false,
};

const INITIAL_TREE_SETTINGS: RvizTreeDisplaySettings = {
  treeStyle: 'hierarchy',
  expandDetails: false,
  disabledLinks: {},
  disabledTfFrames: {},
  disabledJoints: {},
  tfMarkerScale: 1,
};

function zeroJointValues(model: ParsedUrdfModel): Record<string, number> {
  const next: Record<string, number> = {};
  for (const joint of Object.values(model.joints) as UrdfJoint[]) {
    if (joint.type === 'fixed' || joint.type === 'floating') continue;
    next[joint.name] =
      joint.type === 'prismatic'
        ? Math.max(joint.limit.lower, Math.min(joint.limit.upper, 0))
        : 0;
  }
  return resolveMimicJointValues(model, next);
}

/**
 * ROS 2 pane: RViz2-parity viewport.
 * Left: the RViz2 Displays tree (Global Options, RobotModel links, TF frames, topic selectors).
 * Right: the 3D viewport with the joint-state publisher dock, exactly like the previous version.
 * The ros_gz_bridge is started from the Ubuntu terminal, not from this pane.
 */
export const Ros2VisualizerPane: React.FC<Ros2VisualizerPaneProps> = ({
  ros2State,
  activeUrdfXml,
  onToggleUseSimTime,
  onMinimizePane,
  isMaximized = false,
  onToggleMaximizePane,
}) => {
  const [showDisplays, setShowDisplays] = useState(true);
  const [viewFramesOpen, setViewFramesOpen] = useState(false);
  const [fixedFrame, setFixedFrame] = useState('base_footprint');
  const [layers, setLayers] = useState<ViewportLayers>(INITIAL_LAYERS);
  const [treeSettings, setTreeSettings] =
    useState<RvizTreeDisplaySettings>(INITIAL_TREE_SETTINGS);
  const [topics, setTopics] = useState<Ros2DisplayTopics>(
    DEFAULT_ROS2_DISPLAY_TOPICS,
  );
  const [selectedNode, setSelectedNode] = useState<{
    type: 'link' | 'joint';
    name: string;
  } | null>(null);
  const [jointValues, setJointValues] = useState<Record<string, number>>({});
  const [meshAssets] = useState<Record<string, CustomMeshAsset>>(() =>
    getDefaultBuiltInMeshes(),
  );

  // The pane always renders a real URDF: the active file when it is one, else the
  // diff-drive rover that both Gazebo and ROS 2 are built around.
  const model = useMemo<ParsedUrdfModel>(() => {
    if (activeUrdfXml && activeUrdfXml.includes('<robot')) {
      return compileUrdf(activeUrdfXml);
    }
    const fallback =
      STARTER_TEMPLATES.find((t) => !t.isXacro) ?? STARTER_TEMPLATES[0];
    return compileUrdf(fallback.urdfXml);
  }, [activeUrdfXml]);

  // Re-seed the joint sliders whenever a different robot is compiled.
  useEffect(() => {
    setJointValues(zeroJointValues(model));
    setSelectedNode(null);
  }, [model]);

  // Live /joint_states from the bridged Gazebo simulation drive the same sliders.
  useEffect(() => {
    const states = ros2State.bridgedTelemetry.jointStates;
    if (!ros2State.isBridgeRunning || states.length === 0) return;
    setJointValues((prev) => {
      const next = { ...prev };
      let changed = false;
      for (const state of states) {
        const joint = model.joints[state.name];
        if (!joint || joint.type === 'fixed' || joint.type === 'floating') {
          continue;
        }
        if (next[state.name] !== state.position) {
          next[state.name] = state.position;
          changed = true;
        }
      }
      return changed ? resolveMimicJointValues(model, next) : prev;
    });
  }, [ros2State.bridgedTelemetry.jointStates, ros2State.isBridgeRunning, model]);

  const worldTransforms = useMemo(
    () => computeForwardKinematics(model, resolveMimicJointValues(model, jointValues)),
    [model, jointValues],
  );

  const sensors = useMemo<RvizSensorData>(
    () => ({
      laserScan: ros2State.bridgedTelemetry.scanReceived
        ? ros2State.bridgedTelemetry.scan
        : null,
      odomPose: ros2State.bridgedTelemetry.odomReceived
        ? (ros2State.bridgedTelemetry.odom?.pose ?? null)
        : null,
      odometryTrail: ros2State.odometryTrail,
      showLaserScan: layers.laserScan !== false,
      showOdometry: layers.odometry !== false,
    }),
    [
      ros2State.bridgedTelemetry.scan,
      ros2State.bridgedTelemetry.scanReceived,
      ros2State.bridgedTelemetry.odom,
      ros2State.bridgedTelemetry.odomReceived,
      ros2State.odometryTrail,
      layers.laserScan,
      layers.odometry,
    ],
  );

  const bridgedRosTopics = useMemo(
    () =>
      ros2State.bridgeRules
        .filter((rule) => rule.enabled)
        .map((rule) => rule.rosTopic),
    [ros2State.bridgeRules],
  );

  const toggleLayer = (key: keyof ViewportLayers) =>
    setLayers((prev) => ({ ...prev, [key]: !prev[key] }));

  const updateTreeSettings = (patch: Partial<RvizTreeDisplaySettings>) =>
    setTreeSettings((prev) => ({ ...prev, ...patch }));

  const handleJointValue = (jointName: string, value: number) =>
    setJointValues((prev) =>
      resolveMimicJointValues(model, { ...prev, [jointName]: value }),
    );

  const bridgedCount = ros2State.bridgeRules.filter(
    (rule) => ros2State.isBridgeRunning && rule.enabled,
  ).length;

  const { scanReceived, odomReceived, tfReceived, imuReceived } =
    ros2State.bridgedTelemetry;

  return (
    <div
      data-testid="ros2-visualizer-pane"
      className="flex flex-col w-full h-full min-h-0 bg-[#0b1120] overflow-hidden"
    >
      {/* Compact RViz2 title bar: bridge status, sim time, view_frames, pane controls */}
      <header className="flex flex-wrap items-center justify-between gap-x-2 gap-y-1 px-2 py-1 min-h-8 bg-[#111827] border-b border-[#26334d] shrink-0">
        <div className="flex items-center gap-2 min-w-0">
          {!showDisplays && (
            <button
              type="button"
              onClick={() => setShowDisplays(true)}
              title="Show RViz2 Displays panel"
              aria-label="Show RViz2 Displays panel"
              className="text-slate-400 hover:text-white shrink-0"
            >
              <PanelLeftOpen className="w-3.5 h-3.5" />
            </button>
          )}
          <span className="flex items-center gap-1.5 text-[11px] font-semibold text-white whitespace-nowrap shrink-0">
            <Share2 className="w-3.5 h-3.5 text-teal-400" />
            <span>RViz2</span>
          </span>
          <span
            data-testid="ros2-bridge-status"
            title={
              ros2State.isBridgeRunning
                ? 'ros_gz_bridge is running'
                : 'ros_gz_bridge is stopped — start it from the Ubuntu terminal'
            }
            className={`inline-flex items-center gap-1 px-1.5 py-0.5 rounded border text-[10px] font-mono whitespace-nowrap shrink-0 ${
              ros2State.isBridgeRunning
                ? 'bg-emerald-500/10 border-emerald-500/40 text-emerald-300'
                : 'bg-amber-500/10 border-amber-500/40 text-amber-300'
            }`}
          >
            <Radio className="w-2.5 h-2.5" />
            <span>
              bridge {ros2State.isBridgeRunning ? 'ACTIVE' : 'STOPPED'} (
              {bridgedCount})
            </span>
          </span>
          <span
            data-testid="ros2-telemetry-strip"
            className="flex items-center gap-1.5 text-[10px] font-mono whitespace-nowrap shrink-0"
          >
            {!ros2State.isBridgeRunning ? (
              <span
                data-testid="rviz-bridge-stopped-warning"
                className="flex items-center gap-1 text-amber-400"
              >
                <AlertTriangle className="w-2.5 h-2.5" />
                <span>stopped</span>
              </span>
            ) : (
              (['scan', 'odom', 'tf'] as const).map((key) => {
                const live =
                  key === 'scan'
                    ? scanReceived
                    : key === 'odom'
                      ? odomReceived
                      : tfReceived;
                return (
                  <span key={key} className="flex items-center gap-1 text-slate-400">
                    <span
                      className={`w-1.5 h-1.5 rounded-full ${
                        live ? 'bg-emerald-400' : 'bg-slate-600'
                      }`}
                    />
                    <span>
                      /{key}
                      {key === 'tf' && live
                        ? ` ${ros2State.bridgedTelemetry.tfTree.length}`
                        : ''}
                    </span>
                  </span>
                );
              })
            )}
          </span>
          <button
            type="button"
            onClick={onToggleUseSimTime}
            data-testid="toggle-use-sim-time-btn"
            title="Toggle ROS 2 use_sim_time (/clock synchronization)"
            className={`inline-flex items-center gap-1 px-1.5 py-0.5 rounded border text-[10px] font-mono whitespace-nowrap shrink-0 ${
              ros2State.useSimTime
                ? 'bg-sky-500/10 border-sky-500/40 text-sky-300'
                : 'bg-slate-800/60 border-slate-600/60 text-slate-400'
            }`}
          >
            <Compass className="w-2.5 h-2.5" />
            <span>use_sim_time {ros2State.useSimTime ? 'True' : 'False'}</span>
          </button>
        </div>

        <div className="flex items-center gap-1 shrink-0">
          <button
            type="button"
            onClick={() => setViewFramesOpen(true)}
            title="Preview the tf2_tools view_frames tree"
            aria-label="Preview tf2_tools view_frames tree"
            className="px-1.5 py-0.5 rounded text-[10px] text-slate-300 hover:text-white hover:bg-slate-800 whitespace-nowrap"
          >
            view_frames
          </button>
          <button
            type="button"
            onClick={() => downloadViewFramesPdf(model)}
            title="Download the tf2_tools view_frames PDF"
            aria-label="Download view_frames PDF"
            className="flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] text-sky-300 hover:text-white bg-sky-500/15 hover:bg-sky-500/25 border border-sky-500/40 whitespace-nowrap"
          >
            <FileDown className="w-3 h-3" />
            <span>PDF</span>
          </button>
          {onMinimizePane && (
            <button
              type="button"
              onClick={onMinimizePane}
              data-testid="minimize-ros2-pane-btn"
              title="Minimize ROS 2 pane"
              aria-label="Minimize ROS 2 pane"
              className="text-slate-400 hover:text-white px-1"
            >
              <Minimize2 className="w-3.5 h-3.5" />
            </button>
          )}
          {onToggleMaximizePane && (
            <button
              type="button"
              onClick={onToggleMaximizePane}
              data-testid="maximize-ros2-pane-btn"
              title={isMaximized ? 'Restore ROS 2 pane' : 'Maximize ROS 2 pane'}
              aria-label={isMaximized ? 'Restore ROS 2 pane' : 'Maximize ROS 2 pane'}
              className="text-slate-400 hover:text-white px-1"
            >
              <Maximize2 className="w-3.5 h-3.5" />
            </button>
          )}
        </div>
      </header>

      <div className="flex flex-1 min-h-0 overflow-hidden">
        {showDisplays && (
          <Ros2DisplaysPanel
            model={model}
            layers={layers}
            onToggleLayer={toggleLayer}
            treeSettings={treeSettings}
            onUpdateTreeSettings={updateTreeSettings}
            selectedNode={selectedNode}
            onSelectNode={setSelectedNode}
            fixedFrame={fixedFrame}
            onFixedFrameChange={setFixedFrame}
            topics={topics}
            onTopicChange={(key, value) =>
              setTopics((prev) => ({ ...prev, [key]: value }))
            }
            bridgedRosTopics={bridgedRosTopics}
            received={{
              scan: scanReceived,
              odom: odomReceived,
              imu: imuReceived,
              tf: tfReceived,
              jointStates: ros2State.bridgedTelemetry.jointStatesReceived,
            }}
            onClose={() => setShowDisplays(false)}
          />
        )}

        <div className="flex-1 min-w-0 relative">
          <RvizViewport
            model={model}
            jointValues={jointValues}
            layers={layers}
            treeSettings={treeSettings}
            meshAssets={meshAssets}
            selectedNode={selectedNode}
            onSelectNode={setSelectedNode}
            worldTransforms={worldTransforms}
            sensors={sensors}
          >
            <BottomRvizDock
              model={model}
              layers={layers}
              onToggleLayer={toggleLayer}
              jointValues={jointValues}
              onChangeJointValue={handleJointValue}
              onZeroJoints={() => setJointValues(zeroJointValues(model))}
              onRandomizeJoints={() =>
                setJointValues(() => {
                  const next: Record<string, number> = {};
                  for (const joint of Object.values(model.joints) as UrdfJoint[]) {
                    if (joint.type === 'revolute' || joint.type === 'continuous') {
                      next[joint.name] = Number(
                        (
                          joint.limit.lower +
                          Math.random() *
                            (joint.limit.upper - joint.limit.lower || 2 * Math.PI)
                        ).toFixed(3),
                      );
                    } else if (joint.type === 'prismatic') {
                      next[joint.name] = Number(
                        (
                          joint.limit.lower +
                          Math.random() * (joint.limit.upper - joint.limit.lower)
                        ).toFixed(3),
                      );
                    }
                  }
                  return resolveMimicJointValues(model, next);
                })
              }
              onSelectJoint={(jointName) =>
                setSelectedNode({ type: 'joint', name: jointName })
              }
            />
          </RvizViewport>

        </div>
      </div>

      <ViewFramesModal
        open={viewFramesOpen}
        onClose={() => setViewFramesOpen(false)}
        model={model}
      />
    </div>
  );
};
