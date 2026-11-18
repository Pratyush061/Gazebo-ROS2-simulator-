import {
  CompilerDiagnostic,
  ParsedWorld,
  Pose6D,
  Ros2BridgedTelemetry,
  Ros2RuntimeState,
  Ros2TfTransform,
  RosGzBridgeDirection,
  RosGzBridgeRule,
  RosGzQosProfile,
  SimModel,
} from '../types/simulation';
import { computeModelLinkPoses, computeModelLinkWorldPoses } from './physicsEngine';

/**
 * Official directional symbol mapping for `ros2 run ros_gz_bridge parameter_bridge`:
 * - `[` : GZ_TO_ROS (Gazebo Transport -> ROS 2 DDS)
 * - `]` : ROS_TO_GZ (ROS 2 DDS -> Gazebo Transport)
 * - `@` : BIDIRECTIONAL (Both directions)
 * Reference: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
 */
export function directionToCliSymbol(direction: RosGzBridgeDirection): '[' | ']' | '@' {
  if (direction === 'GZ_TO_ROS') return '[';
  if (direction === 'ROS_TO_GZ') return ']';
  return '@';
}

export function cliSymbolToDirection(sym: string): RosGzBridgeDirection {
  if (sym === '[') return 'GZ_TO_ROS';
  if (sym === ']') return 'ROS_TO_GZ';
  return 'BIDIRECTIONAL';
}

/**
 * Formats a `RosGzBridgeRule` into an official `parameter_bridge` CLI argument:
 * `<topic>@<ros_type><direction_symbol><gz_type>`
 */
export function formatParameterBridgeCliArg(rule: RosGzBridgeRule): string {
  const sym = directionToCliSymbol(rule.direction);
  return `${rule.gzTopic}@${rule.rosType}${sym}${rule.gzType}`;
}

/**
 * Parses an official `parameter_bridge` CLI argument string such as:
 * `/model/ros2_urdf_rover/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan`
 */
export function parseParameterBridgeCliArg(
  arg: string,
  index = 0
): RosGzBridgeRule | null {
  const trimmed = arg.trim();
  // Match <topic>@<ros_type>([|]|@)<gz_type>
  const match = trimmed.match(/^([^@]+)@([^[\]@]+)([[\]@])(.+)$/);
  if (!match) return null;

  const [, rawTopic, rosType, dirSym, gzType] = match;
  const direction = cliSymbolToDirection(dirSym);
  const gzTopic = rawTopic.trim();
  const shortRosTopic = gzTopic.startsWith('/model/')
    ? '/' + gzTopic.split('/').slice(3).join('/')
    : gzTopic.includes('/joint_state')
    ? '/joint_states'
    : gzTopic;

  const isSensor =
    rosType.includes('LaserScan') ||
    rosType.includes('Imu') ||
    rosType.includes('JointState');
  const qosProfile: RosGzQosProfile =
    gzTopic === '/clock' ? 'clock' : isSensor ? 'sensor_data' : 'default';

  return {
    id: `cli-rule-${index}-${ shortRosTopic.replace(/\W+/g, '_') }`,
    rosTopic: shortRosTopic || gzTopic,
    gzTopic,
    rosType: rosType.trim(),
    gzType: gzType.trim(),
    direction,
    qosProfile,
    enabled: true,
    msgCount: 0,
    rateHz: gzTopic === '/clock' ? 1000 : isSensor ? 50 : 30,
    lastPayloadSummary: 'Awaiting bridge traffic...',
  };
}

/**
 * Builds the default official `RosGzBridgeRule[]` list for a parsed Gazebo world.
 */
export function buildDefaultBridgeRulesForWorld(
  world: ParsedWorld | null
): RosGzBridgeRule[] {
  const heroModel: SimModel | undefined =
    world?.models.find((m) => m.name !== 'ground_plane') || world?.models[0];
  const modelName = heroModel?.name || 'ros2_urdf_rover';
  const worldName = world?.name || 'default';

  return [
    {
      id: 'bridge-clock',
      rosTopic: '/clock',
      gzTopic: '/clock',
      rosType: 'rosgraph_msgs/msg/Clock',
      gzType: 'gz.msgs.Clock',
      direction: 'GZ_TO_ROS',
      qosProfile: 'clock',
      enabled: true,
      msgCount: 0,
      rateHz: 1000,
      lastPayloadSummary: 'sec: 0, nanosec: 0',
    },
    {
      id: 'bridge-cmd-vel',
      rosTopic: '/cmd_vel',
      gzTopic: `/model/${modelName}/cmd_vel`,
      rosType: 'geometry_msgs/msg/Twist',
      gzType: 'gz.msgs.Twist',
      direction: 'ROS_TO_GZ',
      qosProfile: 'default',
      enabled: true,
      msgCount: 0,
      rateHz: 20,
      lastPayloadSummary: 'linear.x: 0.00, angular.z: 0.00',
    },
    {
      id: 'bridge-odom',
      rosTopic: '/odom',
      gzTopic: `/model/${modelName}/odometry`,
      rosType: 'nav_msgs/msg/Odometry',
      gzType: 'gz.msgs.Odometry',
      direction: 'GZ_TO_ROS',
      qosProfile: 'default',
      enabled: true,
      msgCount: 0,
      rateHz: 50,
      lastPayloadSummary: 'pose: [0.00, 0.00, 0.00]',
    },
    {
      id: 'bridge-tf',
      rosTopic: '/tf',
      gzTopic: `/model/${modelName}/tf`,
      rosType: 'tf2_msgs/msg/TFMessage',
      gzType: 'gz.msgs.Pose_V',
      direction: 'GZ_TO_ROS',
      qosProfile: 'default',
      enabled: true,
      msgCount: 0,
      rateHz: 60,
      lastPayloadSummary: 'transforms: odom -> base_footprint',
    },
    {
      id: 'bridge-joint-states',
      rosTopic: '/joint_states',
      gzTopic: `/world/${worldName}/model/${modelName}/joint_state`,
      rosType: 'sensor_msgs/msg/JointState',
      gzType: 'gz.msgs.Model',
      direction: 'GZ_TO_ROS',
      qosProfile: 'sensor_data',
      enabled: true,
      msgCount: 0,
      rateHz: 50,
      lastPayloadSummary: 'joints: []',
    },
    {
      id: 'bridge-scan',
      rosTopic: '/scan',
      gzTopic: `/model/${modelName}/scan`,
      rosType: 'sensor_msgs/msg/LaserScan',
      gzType: 'gz.msgs.LaserScan',
      direction: 'GZ_TO_ROS',
      qosProfile: 'sensor_data',
      enabled: true,
      msgCount: 0,
      rateHz: 20,
      lastPayloadSummary: '36 rays, range: [0.12m..8.00m]',
    },
    {
      id: 'bridge-imu',
      rosTopic: '/imu',
      gzTopic: `/model/${modelName}/imu`,
      rosType: 'sensor_msgs/msg/Imu',
      gzType: 'gz.msgs.IMU',
      direction: 'GZ_TO_ROS',
      qosProfile: 'sensor_data',
      enabled: true,
      msgCount: 0,
      rateHz: 100,
      lastPayloadSummary: 'accel: [0.00, 0.00, 9.81] m/s²',
    },
    {
      id: 'bridge-joint-cmd',
      rosTopic: '/joint_cmd',
      gzTopic: `/model/${modelName}/joint_cmd`,
      rosType: 'std_msgs/msg/Float64',
      gzType: 'gz.msgs.Double',
      direction: 'BIDIRECTIONAL',
      qosProfile: 'default',
      enabled: true,
      msgCount: 0,
      rateHz: 30,
      lastPayloadSummary: 'data: 0.00 rad',
    },
  ];
}

/**
 * Retargets an existing list of `RosGzBridgeRule[]` when the user switches to a different Gazebo world/model
 * while preserving the user's enabled/disabled and directional preferences!
 */
export function retargetBridgeRulesToModel(
  rules: RosGzBridgeRule[],
  newModelName: string,
  worldName = 'default'
): RosGzBridgeRule[] {
  return rules.map((r) => {
    if (r.rosTopic === '/clock') return r;
    const nextGzTopic = r.gzTopic
      .replace(/\/model\/[^/]+\//, `/model/${newModelName}/`)
      .replace(/\/world\/[^/]+\//, `/world/${worldName}/`);
    return {
      ...r,
      gzTopic: nextGzTopic,
    };
  });
}

/**
 * Parses a `ros_gz_bridge.yaml` configuration string into `RosGzBridgeRule[]` and returns
 * any syntax or schema diagnostics.
 */
export function parseRosGzBridgeYaml(yamlText: string): {
  rules: RosGzBridgeRule[];
  diagnostics: CompilerDiagnostic[];
} {
  const diagnostics: CompilerDiagnostic[] = [];
  const rules: RosGzBridgeRule[] = [];
  const lines = yamlText.split(/\r?\n/);

  let currentItem: Partial<RosGzBridgeRule> & { startLine?: number } | null = null;

  const commitCurrent = () => {
    if (!currentItem) return;
    const line = currentItem.startLine || 1;
    if (!currentItem.rosTopic || !currentItem.gzTopic) {
      diagnostics.push({
        line,
        column: 1,
        severity: 'error',
        code: 'ROS_GZ_YAML_MISSING_TOPIC',
        message:
          'Bridge entry must specify both `ros_topic_name` and `gz_topic_name`.',
        suggestion:
          'Add `ros_topic_name: "/scan"` and `gz_topic_name: "/model/robot/scan"`.',
      });
    } else if (!currentItem.rosType || !currentItem.gzType) {
      diagnostics.push({
        line,
        column: 1,
        severity: 'error',
        code: 'ROS_GZ_YAML_MISSING_TYPE',
        message: `Bridge entry for '${currentItem.rosTopic}' is missing 'ros_type_name' or 'gz_type_name'.`,
        suggestion:
          'Specify valid ROS 2 and Gazebo message types (e.g. sensor_msgs/msg/LaserScan and gz.msgs.LaserScan).',
      });
    } else {
      const dir = currentItem.direction || 'BIDIRECTIONAL';
      const qos = currentItem.qosProfile || 'default';
      rules.push({
        id: `yaml-rule-${rules.length}-${currentItem.rosTopic.replace(/\W+/g, '_')}`,
        rosTopic: currentItem.rosTopic,
        gzTopic: currentItem.gzTopic,
        rosType: currentItem.rosType,
        gzType: currentItem.gzType,
        direction: dir,
        qosProfile: qos,
        enabled: currentItem.enabled !== undefined ? currentItem.enabled : true,
        msgCount: 0,
        rateHz: currentItem.rosTopic === '/clock' ? 1000 : 30,
        lastPayloadSummary: 'Configured via ros_gz_bridge.yaml',
      });
    }
    currentItem = null;
  };

  for (let i = 0; i < lines.length; i++) {
    const rawLine = lines[i];
    const stripped = rawLine.replace(/#.*$/, '').trim();
    if (!stripped) continue;

    if (stripped.startsWith('-')) {
      commitCurrent();
      currentItem = { startLine: i + 1, enabled: true };
      const afterDash = stripped.slice(1).trim();
      if (afterDash) {
        parseKeyValIntoItem(afterDash, i + 1, currentItem, diagnostics);
      }
    } else if (currentItem) {
      parseKeyValIntoItem(stripped, i + 1, currentItem, diagnostics);
    }
  }
  commitCurrent();

  return { rules, diagnostics };
}

function parseKeyValIntoItem(
  lineText: string,
  lineNumber: number,
  item: Partial<RosGzBridgeRule>,
  diagnostics: CompilerDiagnostic[]
): void {
  const colonIdx = lineText.indexOf(':');
  if (colonIdx === -1) {
    diagnostics.push({
      line: lineNumber,
      column: 1,
      severity: 'warning',
      code: 'ROS_GZ_YAML_SYNTAX',
      message: `Expected 'key: value' pair in ros_gz_bridge YAML on line ${lineNumber}.`,
    });
    return;
  }

  const key = lineText.slice(0, colonIdx).trim();
  const val = lineText
    .slice(colonIdx + 1)
    .trim()
    .replace(/^["']|["']$/g, '');

  switch (key) {
    case 'ros_topic_name':
      item.rosTopic = val;
      break;
    case 'gz_topic_name':
      item.gzTopic = val;
      break;
    case 'ros_type_name':
      item.rosType = val;
      break;
    case 'gz_type_name':
      item.gzType = val;
      break;
    case 'direction':
      if (
        val === 'GZ_TO_ROS' ||
        val === 'ROS_TO_GZ' ||
        val === 'BIDIRECTIONAL'
      ) {
        item.direction = val;
      } else {
        diagnostics.push({
          line: lineNumber,
          column: colonIdx + 2,
          severity: 'error',
          code: 'ROS_GZ_INVALID_DIRECTION',
          message: `Invalid bridge direction '${val}'. Allowed values: GZ_TO_ROS, ROS_TO_GZ, BIDIRECTIONAL.`,
        });
      }
      break;
    case 'qos_profile':
      if (
        val === 'sensor_data' ||
        val === 'default' ||
        val === 'tf_static' ||
        val === 'clock'
      ) {
        item.qosProfile = val;
      }
      break;
    case 'enabled':
    case 'lazy':
      if (key === 'enabled') {
        item.enabled = val !== 'false';
      }
      break;
    default:
      break;
  }
}

/**
 * Serializes `RosGzBridgeRule[]` into an official `ros_gz_bridge.yaml` configuration file.
 */
export function serializeRosGzBridgeYaml(rules: RosGzBridgeRule[]): string {
  const lines: string[] = [
    '# Official ROS 2 Jazzy <-> Gazebo Harmonic (gz-sim8) parameter_bridge configuration',
    '# Reference: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge',
    '# CLI: ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=ros2_ws/ros_gz_bridge.yaml',
    '',
  ];

  for (const r of rules) {
    lines.push(`- ros_topic_name: "${r.rosTopic}"`);
    lines.push(`  gz_topic_name: "${r.gzTopic}"`);
    lines.push(`  ros_type_name: "${r.rosType}"`);
    lines.push(`  gz_type_name: "${r.gzType}"`);
    lines.push(`  direction: ${r.direction}`);
    lines.push(`  qos_profile: ${r.qosProfile}`);
    if (!r.enabled) {
      lines.push(`  enabled: false`);
    }
    lines.push('');
  }

  return lines.join('\n');
}

function createEmptyBridgedTelemetry(): Ros2BridgedTelemetry {
  return {
    clockReceived: false,
    scanReceived: false,
    imuReceived: false,
    odomReceived: false,
    jointStatesReceived: false,
    tfReceived: false,
    scan: null,
    imu: null,
    odom: null,
    jointStates: [],
    tfTree: [],
  };
}

/**
 * Creates a fresh `Ros2RuntimeState` for the given Gazebo world and populates initial bridged telemetry.
 */
export function createDefaultRos2RuntimeState(
  world: ParsedWorld | null
): Ros2RuntimeState {
  const heroModel =
    world?.models.find((m) => m.name !== 'ground_plane') || world?.models[0];
  const state: Ros2RuntimeState = {
    // Clean unconnected state: the bridge is started by the user from the Ubuntu
    // terminal (ros2 run ros_gz_bridge parameter_bridge / ros2 launch ...), exactly
    // like a real ROS 2 session.
    isBridgeRunning: false,
    useSimTime: true,
    rosClockSec: 0,
    targetModelName: heroModel?.name || 'ros2_urdf_rover',
    bridgeRules: buildDefaultBridgeRulesForWorld(world),
    nodes: {
      robotStatePublisher: true,
      lidarObstacleAvoidance: false,
      teleopTwistRos2: true,
      jointTrajectoryPublisher: false,
    },
    pendingRosCmdVel: {
      linearX: 0,
      linearY: 0,
      angularZ: 0,
      active: false,
    },
    pendingRosJointCmds: {},
    bridgedTelemetry: createEmptyBridgedTelemetry(),
    odometryTrail: [],
    lastAvoidanceStatus: 'Idle (Node standby)',
  };

  if (world) {
    stepRosGzBridgeAndNodes(world, state, 0, 0.016);
  }

  return state;
}

/**
 * Steps the official `ros_gz_bridge` (`parameter_bridge`) and active ROS 2 Jazzy nodes:
 * 1. Enforces `ros2State.isBridgeRunning` and each rule's `enabled` + `direction` (`GZ_TO_ROS`, `ROS_TO_GZ`, `BIDIRECTIONAL`).
 * 2. Transfers Gazebo Transport telemetry (`/clock`, `/scan`, `/imu`, `/odom`, `/joint_states`, `/tf`) to ROS 2 DDS
 *    ONLY when bridged in `GZ_TO_ROS` (`[`) or `BIDIRECTIONAL` (`@`).
 * 3. Executes active ROS 2 nodes (`robot_state_publisher`, `lidar_obstacle_avoidance`, `joint_trajectory_publisher`).
 * 4. Transfers ROS 2 actuation commands (`/cmd_vel`, `/joint_cmd`) to Gazebo Transport
 *    ONLY when bridged in `ROS_TO_GZ` (`]`) or `BIDIRECTIONAL` (`@`).
 */
export function stepRosGzBridgeAndNodes(
  world: ParsedWorld,
  ros2State: Ros2RuntimeState,
  simTimeSec: number,
  dt: number
): void {
  const heroModel =
    world.models.find((m) => m.name === ros2State.targetModelName) ||
    world.models.find((m) => m.name !== 'ground_plane') ||
    world.models[0];

  if (!heroModel) return;

  // Keep targetModelName and bridge rule topics aligned with the active robot model
  if (ros2State.targetModelName !== heroModel.name) {
    ros2State.targetModelName = heroModel.name;
    ros2State.bridgeRules = retargetBridgeRulesToModel(
      ros2State.bridgeRules,
      heroModel.name,
      world.name
    );
    ros2State.odometryTrail = [];
  }

  const canGzToRos = (rosTopicPrefix: string): RosGzBridgeRule | undefined => {
    if (!ros2State.isBridgeRunning) return undefined;
    return ros2State.bridgeRules.find(
      (r) =>
        r.enabled &&
        (r.direction === 'GZ_TO_ROS' || r.direction === 'BIDIRECTIONAL') &&
        (r.rosTopic === rosTopicPrefix || r.rosTopic.endsWith(rosTopicPrefix))
    );
  };

  const canRosToGz = (rosTopicPrefix: string): RosGzBridgeRule | undefined => {
    if (!ros2State.isBridgeRunning) return undefined;
    return ros2State.bridgeRules.find(
      (r) =>
        r.enabled &&
        (r.direction === 'ROS_TO_GZ' || r.direction === 'BIDIRECTIONAL') &&
        (r.rosTopic === rosTopicPrefix || r.rosTopic.endsWith(rosTopicPrefix))
    );
  };

  // 1. /clock synchronization (`use_sim_time`)
  const clockRule = canGzToRos('/clock');
  if (clockRule) {
    ros2State.bridgedTelemetry.clockReceived = true;
    ros2State.rosClockSec = simTimeSec;
    const sec = Math.floor(simTimeSec);
    const nsec = Math.floor((simTimeSec - sec) * 1e9);
    clockRule.msgCount += 1;
    clockRule.lastPayloadSummary = `sec: ${sec}, nanosec: ${nsec}`;
  } else {
    ros2State.bridgedTelemetry.clockReceived = false;
    if (!ros2State.useSimTime) {
      ros2State.rosClockSec += dt;
    }
  }

  const stampSec = ros2State.rosClockSec;

  // 2. /odom (GZ_TO_ROS: gz.msgs.Odometry -> nav_msgs/msg/Odometry)
  const odomRule = canGzToRos('/odom');
  if (odomRule) {
    ros2State.bridgedTelemetry.odomReceived = true;
    const pose: Pose6D = [...heroModel.pose];
    ros2State.bridgedTelemetry.odom = {
      frameId: 'odom',
      childFrameId: heroModel.virtualRootLink || heroModel.canonicalLink,
      pose,
      linearTwist: [...heroModel.linearVelocity],
      angularTwist: [...heroModel.angularVelocity],
    };
    odomRule.msgCount += 1;
    odomRule.lastPayloadSummary = `x: ${pose[0].toFixed(2)}m, y: ${pose[1].toFixed(2)}m, yaw: ${pose[5].toFixed(2)}rad`;

    // Record odometry trail point if robot moved > 4 cm
    const lastPt = ros2State.odometryTrail[ros2State.odometryTrail.length - 1];
    if (
      !lastPt ||
      Math.hypot(pose[0] - lastPt[0], pose[1] - lastPt[1], pose[2] - lastPt[2]) >
        0.04
    ) {
      ros2State.odometryTrail.push([pose[0], pose[1], pose[2]]);
      if (ros2State.odometryTrail.length > 160) {
        ros2State.odometryTrail.shift();
      }
    }
  } else {
    ros2State.bridgedTelemetry.odomReceived = false;
  }

  // 3. /joint_states (GZ_TO_ROS: gz.msgs.Model -> sensor_msgs/msg/JointState)
  const jointStatesRule = canGzToRos('/joint_states');
  if (jointStatesRule) {
    ros2State.bridgedTelemetry.jointStatesReceived = true;
    const movableJoints = heroModel.joints.filter((j) => j.type !== 'fixed');
    ros2State.bridgedTelemetry.jointStates = movableJoints.map((j) => ({
      name: j.name,
      position: j.position,
      velocity: j.velocity,
      effort: j.appliedEffort,
    }));
    jointStatesRule.msgCount += 1;
    jointStatesRule.lastPayloadSummary =
      movableJoints.length > 0
        ? movableJoints
            .slice(0, 3)
            .map((j) => `${j.name}: ${j.position.toFixed(2)}`)
            .join(', ')
        : '0 movable joints';
  } else {
    ros2State.bridgedTelemetry.jointStatesReceived = false;
  }

  // 4. /tf (GZ_TO_ROS: gz.msgs.Pose_V + robot_state_publisher -> tf2_msgs/msg/TFMessage)
  const tfRule = canGzToRos('/tf');
  if (tfRule || (ros2State.nodes.robotStatePublisher && jointStatesRule)) {
    ros2State.bridgedTelemetry.tfReceived = true;
    const tfTree: Ros2TfTransform[] = [];
    const rootFrame = heroModel.virtualRootLink || heroModel.canonicalLink || 'base_link';

    if (tfRule) {
      tfTree.push({
        parentFrame: 'odom',
        childFrame: rootFrame,
        translation: [heroModel.pose[0], heroModel.pose[1], heroModel.pose[2]],
        rotationRpy: [heroModel.pose[3], heroModel.pose[4], heroModel.pose[5]],
        timestampSec: stampSec,
      });
      tfRule.msgCount += 1;
      tfRule.lastPayloadSummary = `odom -> ${rootFrame} (${heroModel.links.length} links)`;
    }

    if (ros2State.nodes.robotStatePublisher) {
      const linkLocalPoses = computeModelLinkPoses(heroModel);
      for (const link of heroModel.links) {
        if (link.name === rootFrame) continue;
        const lPose = linkLocalPoses.get(link.name) || link.pose;
        tfTree.push({
          parentFrame: rootFrame,
          childFrame: link.name,
          translation: [lPose[0], lPose[1], lPose[2]],
          rotationRpy: [lPose[3], lPose[4], lPose[5]],
          timestampSec: stampSec,
        });
      }
    }

    ros2State.bridgedTelemetry.tfTree = tfTree;
  } else {
    ros2State.bridgedTelemetry.tfReceived = false;
  }

  // 5. /scan (GZ_TO_ROS: gz.msgs.LaserScan -> sensor_msgs/msg/LaserScan)
  const scanRule = canGzToRos('/scan');
  let foundLidar = false;
  if (scanRule) {
    const linkWorldPoses = computeModelLinkWorldPoses(heroModel);
    for (const link of heroModel.links) {
      const lidarSensor = (link.sensors || []).find((s) => s.type === 'gpu_lidar');
      if (lidarSensor && lidarSensor.lidar) {
        foundLidar = true;
        const linkPose = linkWorldPoses.get(link.name) || heroModel.pose;
        ros2State.bridgedTelemetry.scanReceived = true;
        ros2State.bridgedTelemetry.scan = {
          frameId: link.name,
          minAngle: lidarSensor.lidar.minAngle,
          maxAngle: lidarSensor.lidar.maxAngle,
          minRange: lidarSensor.lidar.minRange,
          maxRange: lidarSensor.lidar.maxRange,
          ranges: [...lidarSensor.lidar.ranges],
          sensorWorldPose: [
            linkPose[0] + lidarSensor.pose[0],
            linkPose[1] + lidarSensor.pose[1],
            linkPose[2] + lidarSensor.pose[2],
            linkPose[3],
            linkPose[4],
            linkPose[5],
          ],
        };
        const minHit =
          lidarSensor.lidar.ranges.length > 0
            ? Math.min(...lidarSensor.lidar.ranges)
            : lidarSensor.lidar.maxRange;
        scanRule.msgCount += 1;
        scanRule.lastPayloadSummary = `frame: ${link.name}, min_range: ${minHit.toFixed(2)}m (${lidarSensor.lidar.ranges.length} rays)`;
        break;
      }
    }
  }
  if (!scanRule || !foundLidar) {
    ros2State.bridgedTelemetry.scanReceived = false;
    if (!scanRule) {
      ros2State.bridgedTelemetry.scan = null;
    }
  }

  // 6. /imu (GZ_TO_ROS: gz.msgs.IMU -> sensor_msgs/msg/Imu)
  const imuRule = canGzToRos('/imu');
  let foundImu = false;
  if (imuRule) {
    for (const link of heroModel.links) {
      const imuSensor = (link.sensors || []).find((s) => s.type === 'imu');
      if (imuSensor && imuSensor.imu) {
        foundImu = true;
        ros2State.bridgedTelemetry.imuReceived = true;
        ros2State.bridgedTelemetry.imu = {
          frameId: link.name,
          orientation: [...imuSensor.imu.orientation],
          angularVelocity: [...imuSensor.imu.angularVelocity],
          linearAcceleration: [...imuSensor.imu.linearAcceleration],
        };
        imuRule.msgCount += 1;
        imuRule.lastPayloadSummary = `acc_z: ${imuSensor.imu.linearAcceleration[2].toFixed(2)} m/s², gyro_z: ${imuSensor.imu.angularVelocity[2].toFixed(2)} rad/s`;
        break;
      }
    }
  }
  if (!imuRule || !foundImu) {
    ros2State.bridgedTelemetry.imuReceived = false;
  }

  // 7. Execute Closed-Loop ROS 2 Nodes (only when clock is advancing!)
  const simClockActive = !ros2State.useSimTime || ros2State.bridgedTelemetry.clockReceived;

  if (ros2State.nodes.lidarObstacleAvoidance) {
    if (!simClockActive) {
      ros2State.lastAvoidanceStatus =
        'Blocked: Waiting for /clock (use_sim_time=true)';
    } else if (!ros2State.bridgedTelemetry.scanReceived || !ros2State.bridgedTelemetry.scan) {
      ros2State.lastAvoidanceStatus =
        'Waiting for bridged /scan (sensor_msgs/msg/LaserScan)...';
    } else {
      const ranges = ros2State.bridgedTelemetry.scan.ranges;
      const n = ranges.length;
      const third = Math.max(1, Math.floor(n / 3));
      const rightSector = ranges.slice(0, third);
      const frontSector = ranges.slice(third, n - third);
      const leftSector = ranges.slice(n - third);

      const minFront = frontSector.length > 0 ? Math.min(...frontSector) : 8.0;
      const avgLeft =
        leftSector.reduce((s, v) => s + v, 0) / Math.max(1, leftSector.length);
      const avgRight =
        rightSector.reduce((s, v) => s + v, 0) / Math.max(1, rightSector.length);

      if (minFront < 1.75) {
        const turnDir = avgLeft >= avgRight ? 1.15 : -1.15;
        ros2State.pendingRosCmdVel = {
          linearX: 0.18,
          linearY: 0,
          angularZ: turnDir,
          active: true,
        };
        ros2State.lastAvoidanceStatus = `Obstacle at ${minFront.toFixed(2)}m -> Turning ${turnDir > 0 ? 'LEFT' : 'RIGHT'} (ω=${turnDir.toFixed(2)} rad/s)`;
      } else {
        ros2State.pendingRosCmdVel = {
          linearX: 0.95,
          linearY: 0,
          angularZ: 0.15,
          active: true,
        };
        ros2State.lastAvoidanceStatus = `Clear (${minFront.toFixed(2)}m) -> Cruising forward (v=0.95 m/s)`;
      }
    }
  }

  if (ros2State.nodes.jointTrajectoryPublisher && simClockActive) {
    const movableJoints = heroModel.joints.filter(
      (j) => j.type !== 'fixed' && !j.name.toLowerCase().includes('wheel')
    );
    movableJoints.forEach((j, idx) => {
      const amp = Math.min(1.1, (j.axis.upper - j.axis.lower) * 0.35 || 0.8);
      ros2State.pendingRosJointCmds[j.name] =
        Math.sin(ros2State.rosClockSec * 1.6 + idx * 0.9) * amp;
    });
  }

  // 8. ROS_TO_GZ Actuation across `parameter_bridge` (`]` or `@`)
  const cmdVelRule = canRosToGz('/cmd_vel');
  if (ros2State.pendingRosCmdVel.active) {
    if (cmdVelRule) {
      heroModel.cmdVel = {
        linearX: ros2State.pendingRosCmdVel.linearX,
        linearY: ros2State.pendingRosCmdVel.linearY,
        angularZ: ros2State.pendingRosCmdVel.angularZ,
      };
      cmdVelRule.msgCount += 1;
      cmdVelRule.lastPayloadSummary = `linear.x: ${ros2State.pendingRosCmdVel.linearX.toFixed(2)}, angular.z: ${ros2State.pendingRosCmdVel.angularZ.toFixed(2)}`;
    } else if (ros2State.nodes.lidarObstacleAvoidance) {
      ros2State.lastAvoidanceStatus =
        'Blocked: /cmd_vel is not bridged in ROS_TO_GZ (]) direction!';
      heroModel.cmdVel = { linearX: 0, linearY: 0, angularZ: 0 };
    }
  }

  const jointCmdRule = canRosToGz('/joint_cmd');
  const pendingJointEntries = Object.entries(ros2State.pendingRosJointCmds);
  if (pendingJointEntries.length > 0 && jointCmdRule) {
    for (const [jName, targetVal] of pendingJointEntries) {
      const joint = heroModel.joints.find((j) => j.name === jName);
      if (joint && joint.type !== 'fixed') {
        joint.controlMode = 'position';
        joint.targetPosition = targetVal;
      }
    }
    jointCmdRule.msgCount += 1;
    const [firstJoint, firstVal] = pendingJointEntries[0];
    jointCmdRule.lastPayloadSummary = `${firstJoint} -> ${firstVal.toFixed(2)} rad`;
  }
}
