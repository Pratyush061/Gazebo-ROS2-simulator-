import React, { useMemo, useState } from 'react';
import {
  ChevronDown,
  ChevronRight,
  Layers,
  PanelLeftClose,
  Search,
} from 'lucide-react';
import {
  ParsedUrdfModel,
  RvizTreeDisplaySettings,
  ViewportLayers,
} from '../urdf/types';
import {
  buildDisplayTreeRows,
  collectTopicOptions,
  listModelLinks,
} from '../urdf/rvizDisplayTree';

/** ROS 2 topics bound to each RViz2 display. */
export interface Ros2DisplayTopics {
  robotDescription: string;
  tf: string;
  scan: string;
  odom: string;
  imu: string;
}

export const DEFAULT_ROS2_DISPLAY_TOPICS: Ros2DisplayTopics = {
  robotDescription: '/robot_description',
  tf: '/tf',
  scan: '/scan',
  odom: '/odom',
  imu: '/imu',
};

interface Ros2DisplaysPanelProps {
  model: ParsedUrdfModel;
  layers: ViewportLayers;
  onToggleLayer: (key: keyof ViewportLayers) => void;
  treeSettings: RvizTreeDisplaySettings;
  onUpdateTreeSettings: (patch: Partial<RvizTreeDisplaySettings>) => void;
  selectedNode: { type: 'link' | 'joint'; name: string } | null;
  onSelectNode: (node: { type: 'link' | 'joint'; name: string } | null) => void;
  fixedFrame: string;
  onFixedFrameChange: (frame: string) => void;
  topics: Ros2DisplayTopics;
  onTopicChange: (key: keyof Ros2DisplayTopics, value: string) => void;
  bridgedRosTopics: string[];
  received: {
    scan: boolean;
    odom: boolean;
    imu: boolean;
    tf: boolean;
    jointStates: boolean;
  };
  onClose: () => void;
}

type Status = 'ok' | 'warn' | 'off';

const statusDot: Record<Status, string> = {
  ok: 'bg-emerald-400',
  warn: 'bg-amber-400',
  off: 'bg-slate-600',
};

const statusLabel: Record<Status, string> = {
  ok: 'OK',
  warn: 'Warning',
  off: 'Disabled',
};

/** One collapsible display row: enable box, name, status, expander. */
const DisplayRow: React.FC<{
  name: string;
  enabled: boolean;
  status: Status;
  statusText: string;
  onToggle: () => void;
  expanded: boolean;
  onExpand: () => void;
  children?: React.ReactNode;
}> = ({ name, enabled, status, statusText, onToggle, expanded, onExpand, children }) => (
  <div className="border-b border-[#26334d]/60">
    <div className="flex items-center gap-1.5 px-1.5 py-1 hover:bg-[#1e293b]/60">
      <button
        type="button"
        onClick={onExpand}
        aria-label={`${expanded ? 'Collapse' : 'Expand'} ${name} properties`}
        className="text-slate-400 hover:text-white shrink-0"
      >
        {expanded ? (
          <ChevronDown className="w-3 h-3" />
        ) : (
          <ChevronRight className="w-3 h-3" />
        )}
      </button>
      <input
        type="checkbox"
        checked={enabled}
        onChange={onToggle}
        aria-label={`Enable ${name} display`}
        className="w-3 h-3 accent-sky-500 shrink-0"
      />
      <span
        className={`flex-1 truncate text-[11.5px] font-medium ${
          enabled ? 'text-slate-200' : 'text-slate-500'
        }`}
        title={`${name} — ${statusText}`}
      >
        {name}
      </span>
      <span
        className={`w-1.5 h-1.5 rounded-full shrink-0 ${statusDot[status]}`}
        title={statusText}
        aria-label={statusLabel[status]}
      />
    </div>
    {expanded && children ? (
      <div className="pl-6 pr-2 pb-1.5 space-y-1">{children}</div>
    ) : null}
  </div>
);

const PropRow: React.FC<{ label: string; children: React.ReactNode }> = ({
  label,
  children,
}) => (
  <div className="flex items-center justify-between gap-2 text-[11px]">
    <span className="text-slate-400 whitespace-nowrap">{label}</span>
    <span className="text-slate-200 shrink-0 max-w-[58%]">{children}</span>
  </div>
);

const TopicSelect: React.FC<{
  label: string;
  value: string;
  options: string[];
  onChange: (value: string) => void;
}> = ({ label, value, options, onChange }) => (
  <label className="flex items-center justify-between gap-2 text-[11px]">
    <span className="text-slate-400 whitespace-nowrap">{label}</span>
    <select
      value={value}
      onChange={(e) => onChange(e.target.value)}
      aria-label={label}
      className="max-w-[56%] bg-[#0f172a] text-sky-300 font-mono text-[10.5px] rounded border border-[#334155] px-1 py-0.5"
    >
      {options.map((opt) => (
        <option key={opt} value={opt}>
          {opt}
        </option>
      ))}
    </select>
  </label>
);

/**
 * RViz2 "Displays" panel: Global Options (Fixed Frame) plus one collapsible row
 * per display. Every 3D overlay from the previous version is reachable here:
 * visual, collision, wireframe, inertia boxes, centre of mass, TF axes, TF names,
 * joint axes and grid — plus the Links and TF frame trees with per-frame toggles.
 */
export const Ros2DisplaysPanel: React.FC<Ros2DisplaysPanelProps> = ({
  model,
  layers,
  onToggleLayer,
  treeSettings,
  onUpdateTreeSettings,
  selectedNode,
  onSelectNode,
  fixedFrame,
  onFixedFrameChange,
  topics,
  onTopicChange,
  bridgedRosTopics,
  received,
  onClose,
}) => {
  const [expanded, setExpanded] = useState<Record<string, boolean>>({
    'Global Options': true,
    RobotModel: true,
    TF: false,
    LaserScan: false,
    Odometry: false,
    Imu: false,
    Grid: false,
    JointAxes: false,
  });
  const [treeFilter, setTreeFilter] = useState('');

  const toggleExpanded = (name: string) =>
    setExpanded((prev) => ({ ...prev, [name]: !prev[name] }));

  const treeRows = useMemo(
    () => buildDisplayTreeRows(model, treeSettings),
    [model, treeSettings.treeStyle],
  );

  const frameOptions = useMemo(() => {
    const frames = ['map', 'odom', 'base_footprint', ...listModelLinks(model)];
    return Array.from(new Set(frames));
  }, [model]);

  const filteredRows = useMemo(() => {
    const q = treeFilter.trim().toLowerCase();
    return q ? treeRows.filter((r) => r.name.toLowerCase().includes(q)) : treeRows;
  }, [treeRows, treeFilter]);

  const scanOptions = collectTopicOptions(topics.scan, bridgedRosTopics);
  const odomOptions = collectTopicOptions(topics.odom, bridgedRosTopics);
  const imuOptions = collectTopicOptions(topics.imu, bridgedRosTopics);

  const renderTree = () => (
    <div className="space-y-0.5">
      <div className="flex items-center gap-1">
        <select
          value={treeSettings.treeStyle}
          onChange={(e) =>
            onUpdateTreeSettings({
              treeStyle: e.target.value as RvizTreeDisplaySettings['treeStyle'],
            })
          }
          aria-label="Kinematic tree style"
          className="flex-1 bg-[#0f172a] text-slate-200 text-[10.5px] rounded border border-[#334155] px-1 py-0.5"
        >
          <option value="hierarchy">Hierarchy</option>
          <option value="alphabetic">Alphabetic</option>
        </select>
        <label className="flex items-center gap-1 text-[10px] text-slate-400 whitespace-nowrap">
          <input
            type="checkbox"
            checked={treeSettings.expandDetails}
            onChange={(e) =>
              onUpdateTreeSettings({ expandDetails: e.target.checked })
            }
            className="w-3 h-3 accent-sky-500"
          />
          <span>Details</span>
        </label>
      </div>

      <div className="flex items-center gap-1 bg-[#0f172a] border border-[#334155] rounded px-1">
        <Search className="w-3 h-3 text-slate-500 shrink-0" />
        <input
          value={treeFilter}
          onChange={(e) => setTreeFilter(e.target.value)}
          placeholder="Filter frames"
          aria-label="Filter TF frames"
          className="w-full bg-transparent text-[10.5px] text-slate-200 py-0.5 focus:outline-none"
        />
      </div>

      <div className="max-h-64 overflow-y-auto pr-0.5" data-testid="rviz-tf-tree">
        {filteredRows.length === 0 ? (
          <div className="text-[10.5px] text-slate-500 py-1">
            No frames — compile a URDF to populate the tree.
          </div>
        ) : (
          filteredRows.map((row) => {
            const disabled =
              row.kind === 'link'
                ? treeSettings.disabledLinks[row.name]
                : treeSettings.disabledJoints[row.name];
            const isSelected =
              selectedNode?.type === row.kind && selectedNode.name === row.name;
            return (
              <div
                key={`${row.kind}-${row.name}`}
                className={`flex items-start gap-1 py-[1px] rounded ${
                  isSelected ? 'bg-sky-500/20' : 'hover:bg-[#1e293b]'
                }`}
                style={{ paddingLeft: `${row.depth * 9}px` }}
              >
                <input
                  type="checkbox"
                  checked={!disabled}
                  aria-label={`Toggle ${row.kind} ${row.name}`}
                  onChange={() => {
                    if (row.kind === 'link') {
                      onUpdateTreeSettings({
                        disabledLinks: {
                          ...treeSettings.disabledLinks,
                          [row.name]: !disabled,
                        },
                      });
                    } else {
                      onUpdateTreeSettings({
                        disabledJoints: {
                          ...treeSettings.disabledJoints,
                          [row.name]: !disabled,
                        },
                      });
                    }
                  }}
                  className="w-3 h-3 mt-[2px] accent-sky-500 shrink-0"
                />
                <button
                  type="button"
                  onClick={() =>
                    onSelectNode(
                      isSelected ? null : { type: row.kind, name: row.name },
                    )
                  }
                  className="text-left min-w-0 flex-1"
                  title={row.detail}
                >
                  <span
                    className={`font-mono text-[10.5px] truncate block ${
                      row.kind === 'joint'
                        ? 'text-amber-300/90'
                        : disabled
                          ? 'text-slate-500'
                          : 'text-slate-200'
                    }`}
                  >
                    {row.kind === 'joint' ? '⌐ ' : ''}
                    {row.name}
                  </span>
                  {treeSettings.expandDetails && row.detail ? (
                    <span className="block text-[9.5px] text-slate-500 truncate">
                      {row.detail}
                    </span>
                  ) : null}
                </button>
              </div>
            );
          })
        )}
      </div>
    </div>
  );

  return (
    <aside
      aria-label="RViz2 Displays"
      data-testid="rviz-displays-panel"
      className="w-[262px] shrink-0 h-full flex flex-col bg-[#111827] border-r border-[#26334d] overflow-hidden"
    >
      <div className="flex items-center justify-between px-2 h-7 bg-[#161f30] border-b border-[#26334d] shrink-0">
        <span className="flex items-center gap-1.5 text-[11px] font-semibold text-slate-200">
          <Layers className="w-3.5 h-3.5 text-sky-400" />
          <span>Displays</span>
        </span>
        <button
          type="button"
          onClick={onClose}
          title="Hide Displays panel"
          aria-label="Hide Displays panel"
          className="text-slate-400 hover:text-white"
        >
          <PanelLeftClose className="w-3.5 h-3.5" />
        </button>
      </div>

      <div className="flex-1 overflow-y-auto">
        {/* Global Options */}
        <DisplayRow
          name="Global Options"
          enabled
          status={model.valid ? 'ok' : 'warn'}
          statusText={
            model.valid
              ? `${model.robotName} loaded`
              : 'URDF has compile errors — see Problems'
          }
          onToggle={() => {}}
          expanded={expanded['Global Options']}
          onExpand={() => toggleExpanded('Global Options')}
        >
          <label className="flex items-center justify-between gap-2 text-[11px]">
            <span className="text-slate-400">Fixed Frame</span>
            <select
              value={fixedFrame}
              onChange={(e) => onFixedFrameChange(e.target.value)}
              aria-label="RViz2 Fixed Frame"
              className="max-w-[56%] bg-[#0f172a] text-sky-300 font-mono text-[10.5px] rounded border border-[#334155] px-1 py-0.5"
            >
              {frameOptions.map((f) => (
                <option key={f} value={f}>
                  {f}
                </option>
              ))}
            </select>
          </label>
          <PropRow label="Background">48; 48; 48</PropRow>
          <PropRow label="Frame Rate">30</PropRow>
          <PropRow label="Robot">
            <span className="font-mono">{model.robotName}</span>
          </PropRow>
        </DisplayRow>

        {/* RobotModel */}
        <DisplayRow
          name="RobotModel"
          enabled={layers.visual}
          status={layers.visual ? (model.valid ? 'ok' : 'warn') : 'off'}
          statusText={
            layers.visual
              ? `${Object.keys(model.links).length} links`
              : 'Disabled'
          }
          onToggle={() => onToggleLayer('visual')}
          expanded={expanded.RobotModel}
          onExpand={() => toggleExpanded('RobotModel')}
        >
          <TopicSelect
            label="Description Topic"
            value={topics.robotDescription}
            options={collectTopicOptions(
              topics.robotDescription,
              bridgedRosTopics,
            )}
            onChange={(v) => onTopicChange('robotDescription', v)}
          />
          <PropRow label="Visual Enabled">
            <input
              type="checkbox"
              checked={layers.visual}
              onChange={() => onToggleLayer('visual')}
              aria-label="Toggle visual geometry"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Collision Enabled">
            <input
              type="checkbox"
              checked={layers.collision}
              onChange={() => onToggleLayer('collision')}
              aria-label="Toggle collision geometry"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Wireframe">
            <input
              type="checkbox"
              checked={layers.wireframeVisual}
              onChange={() => onToggleLayer('wireframeVisual')}
              aria-label="Toggle wireframe rendering"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Inertia Boxes">
            <input
              type="checkbox"
              checked={layers.inertiaBoxes}
              onChange={() => onToggleLayer('inertiaBoxes')}
              aria-label="Toggle inertia boxes"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Center of Mass">
            <input
              type="checkbox"
              checked={layers.centerOfMass}
              onChange={() => onToggleLayer('centerOfMass')}
              aria-label="Toggle center of mass markers"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <div className="pt-1 text-[10px] uppercase tracking-wide text-slate-500">
            Links
          </div>
          {renderTree()}
        </DisplayRow>

        {/* TF */}
        <DisplayRow
          name="TF"
          enabled={layers.tfFrames}
          status={
            !layers.tfFrames ? 'off' : received.tf ? 'ok' : 'warn'
          }
          statusText={
            !layers.tfFrames
              ? 'Disabled'
              : received.tf
                ? `/${topics.tf.replace(/^\//, '')} live`
                : 'No /tf received (bridge stopped)'
          }
          onToggle={() => onToggleLayer('tfFrames')}
          expanded={expanded.TF}
          onExpand={() => toggleExpanded('TF')}
        >
          <TopicSelect
            label="Topic"
            value={topics.tf}
            options={collectTopicOptions(topics.tf, bridgedRosTopics)}
            onChange={(v) => onTopicChange('tf', v)}
          />
          <PropRow label="Show Axes">
            <input
              type="checkbox"
              checked={layers.tfFrames}
              onChange={() => onToggleLayer('tfFrames')}
              aria-label="Toggle TF axes"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Show Names">
            <input
              type="checkbox"
              checked={layers.tfLabels}
              onChange={() => onToggleLayer('tfLabels')}
              aria-label="Toggle TF frame names"
              className="w-3 h-3 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Marker Scale">
            <input
              type="range"
              min={0.2}
              max={3}
              step={0.1}
              value={treeSettings.tfMarkerScale}
              onChange={(e) =>
                onUpdateTreeSettings({ tfMarkerScale: Number(e.target.value) })
              }
              aria-label="TF marker scale"
              className="w-20 accent-sky-500"
            />
          </PropRow>
          <PropRow label="Frames">{treeRows.filter((r) => r.kind === 'link').length}</PropRow>
        </DisplayRow>

        {/* LaserScan */}
        <DisplayRow
          name="LaserScan"
          enabled={received.scan}
          status={!received.scan ? 'off' : 'ok'}
          statusText={received.scan ? `${topics.scan} streaming` : 'No scan'}
          onToggle={() => onToggleLayer('laserScan')}
          expanded={expanded.LaserScan}
          onExpand={() => toggleExpanded('LaserScan')}
        >
          <TopicSelect
            label="Topic"
            value={topics.scan}
            options={scanOptions}
            onChange={(v) => onTopicChange('scan', v)}
          />
          <PropRow label="Style">Points</PropRow>
          <PropRow label="Size (m)">0.09</PropRow>
          <PropRow label="Status">{received.scan ? 'OK' : 'No messages'}</PropRow>
        </DisplayRow>

        {/* Odometry */}
        <DisplayRow
          name="Odometry"
          enabled={received.odom}
          status={!received.odom ? 'off' : 'ok'}
          statusText={received.odom ? `${topics.odom} streaming` : 'No odometry'}
          onToggle={() => onToggleLayer('odometry')}
          expanded={expanded.Odometry}
          onExpand={() => toggleExpanded('Odometry')}
        >
          <TopicSelect
            label="Topic"
            value={topics.odom}
            options={odomOptions}
            onChange={(v) => onTopicChange('odom', v)}
          />
          <PropRow label="Keep">100</PropRow>
          <PropRow label="Position Tolerance">0.1</PropRow>
        </DisplayRow>

        {/* Imu */}
        <DisplayRow
          name="Imu"
          enabled={received.imu}
          status={!received.imu ? 'off' : 'ok'}
          statusText={received.imu ? `${topics.imu} streaming` : 'No IMU'}
          onToggle={() => onToggleLayer('imu')}
          expanded={expanded.Imu}
          onExpand={() => toggleExpanded('Imu')}
        >
          <TopicSelect
            label="Topic"
            value={topics.imu}
            options={imuOptions}
            onChange={(v) => onTopicChange('imu', v)}
          />
          <PropRow label="Status">{received.imu ? 'OK' : 'No messages'}</PropRow>
        </DisplayRow>

        {/* Joint axes */}
        <DisplayRow
          name="Joint Axes"
          enabled={layers.jointAxes}
          status={layers.jointAxes ? 'ok' : 'off'}
          statusText="Joint rotation/translation axes"
          onToggle={() => onToggleLayer('jointAxes')}
          expanded={expanded.JointAxes}
          onExpand={() => toggleExpanded('JointAxes')}
        >
          <PropRow label="Axis Length">0.12 m</PropRow>
          <PropRow label="Radius">0.006 m</PropRow>
        </DisplayRow>

        {/* Grid */}
        <DisplayRow
          name="Grid"
          enabled={layers.grid}
          status={layers.grid ? 'ok' : 'off'}
          statusText="Ground reference grid"
          onToggle={() => onToggleLayer('grid')}
          expanded={expanded.Grid}
          onExpand={() => toggleExpanded('Grid')}
        >
          <PropRow label="Cell Size">1</PropRow>
          <PropRow label="Plane Cell Count">12</PropRow>
          <PropRow label="Color">160; 160; 164</PropRow>
        </DisplayRow>
      </div>
    </aside>
  );
};
