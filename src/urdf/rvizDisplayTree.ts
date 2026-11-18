import {
  KinematicTreeNode,
  ParsedUrdfModel,
  RvizTreeDisplaySettings,
  UrdfJoint,
  UrdfLink,
} from './types';

/**
 * One row of the RViz2 Displays tree.
 * Hierarchy mode walks the kinematic tree; alphabetic mode flattens links by name.
 */
export interface DisplayTreeRow {
  kind: 'link' | 'joint';
  name: string;
  depth: number;
  jointType?: UrdfJoint['type'];
  detail?: string;
}

function jointDetail(joint: UrdfJoint): string {
  const axis = `[${joint.axis.join(', ')}]`;
  if (joint.type === 'revolute' || joint.type === 'prismatic') {
    return `${joint.type} · axis ${axis} · limit ${joint.limit.lower.toFixed(
      2,
    )}..${joint.limit.upper.toFixed(2)}`;
  }
  return `${joint.type} · axis ${axis}`;
}

function linkDetail(link: UrdfLink): string {
  const mass = link.inertial ? `${link.inertial.mass.toFixed(3)} kg` : 'no inertial';
  return `${link.visuals.length} visual · ${link.collisions.length} collision · ${mass}`;
}

function walkHierarchy(
  node: KinematicTreeNode,
  depth: number,
  rows: DisplayTreeRow[],
): void {
  if (node.joint) {
    rows.push({
      kind: 'joint',
      name: node.joint.name,
      depth,
      jointType: node.joint.type,
      detail: jointDetail(node.joint),
    });
    depth += 1;
  }
  rows.push({
    kind: 'link',
    name: node.linkName,
    depth,
    detail: linkDetail(node.link),
  });
  for (const child of node.children) {
    walkHierarchy(child, depth + 1, rows);
  }
}

/**
 * Flattens a compiled URDF model into RViz2 Displays tree rows.
 * Returns [] when the model has no root link (nothing to display).
 */
export function buildDisplayTreeRows(
  model: ParsedUrdfModel,
  settings: Pick<RvizTreeDisplaySettings, 'treeStyle'>,
): DisplayTreeRow[] {
  if (!model.valid && !model.rootLink) return [];

  if (settings.treeStyle === 'alphabetic') {
    return (Object.values(model.links) as UrdfLink[])
      .slice()
      .sort((a, b) => a.name.localeCompare(b.name))
      .map((link) => {
        const parentJoint = (Object.values(model.joints) as UrdfJoint[]).find(
          (j) => j.child === link.name,
        );
        return {
          kind: 'link' as const,
          name: link.name,
          depth: 0,
          jointType: parentJoint?.type,
          detail: linkDetail(link),
        };
      });
  }

  const rows: DisplayTreeRow[] = [];
  if (model.tree) walkHierarchy(model.tree, 0, rows);
  return rows;
}

/** Links in kinematic order, used for Fixed Frame options and link toggles. */
export function listModelLinks(model: ParsedUrdfModel): string[] {
  const rows = buildDisplayTreeRows(model, { treeStyle: 'hierarchy' });
  const links = rows.filter((r) => r.kind === 'link').map((r) => r.name);
  return links.length > 0
    ? links
    : (Object.values(model.links) as UrdfLink[]).map((l) => l.name);
}

/**
 * Topic dropdown options for a display, built from the live bridge rules plus
 * the conventional ROS 2 topic name so the list is never empty.
 */
export function collectTopicOptions(
  rosTopic: string,
  bridgeRosTopics: string[],
): string[] {
  const set = new Set<string>([rosTopic, ...bridgeRosTopics]);
  return Array.from(set);
}
