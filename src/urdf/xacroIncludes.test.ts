import test from 'node:test';
import assert from 'node:assert/strict';
import {
  resolveIncludeTarget,
  resolveXacroIncludes,
  stripRobotWrapper,
} from './xacroIncludes';

const PKG = 'ros2_ws/src/my_robot_description';
const ROBOT = `${PKG}/urdf/my_robot.urdf.xacro`;

const files = [
  {
    path: ROBOT,
    content: `<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="common/inertia.xacro"/>
  <link name="base_link"/>
</robot>`,
  },
  {
    path: `${PKG}/urdf/common/inertia.xacro`,
    content: `<robot name="m" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="box_inertia" params="mass">
    <inertial><mass value="\${mass}"/></inertial>
  </xacro:macro>
</robot>`,
  },
];

test('relative and $(find pkg) includes resolve to workspace paths', () => {
  assert.equal(
    resolveIncludeTarget('common/inertia.xacro', ROBOT),
    `${PKG}/urdf/common/inertia.xacro`,
  );
  assert.equal(
    resolveIncludeTarget('$(find my_robot_description)/urdf/common/inertia.xacro', ROBOT),
    `${PKG}/urdf/common/inertia.xacro`,
  );
  assert.equal(resolveIncludeTarget('../other/x.xacro', ROBOT), `${PKG}/other/x.xacro`);
});

test('include fragments are inlined without nesting their <robot> wrapper', () => {
  const result = resolveXacroIncludes(files[0].content, ROBOT, files);

  assert.deepEqual(result.diagnostics, []);
  assert.deepEqual(result.resolved, [`${PKG}/urdf/common/inertia.xacro`]);
  assert.equal(result.xml.match(/<robot/g)?.length, 1, 'only the host <robot> survives');
  assert.ok(result.xml.includes('<xacro:macro name="box_inertia"'));
  assert.ok(result.xml.includes('<link name="base_link"/>'));
});

test('missing includes and cycles become diagnostics, not exceptions', () => {
  const missing = resolveXacroIncludes(
    '<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="nope.xacro"/></robot>',
    ROBOT,
    files,
  );
  assert.equal(missing.diagnostics.length, 1);
  assert.match(missing.diagnostics[0].message, /not found/);

  const loopFiles = [
    { path: 'a.xacro', content: '<robot xmlns:xacro="x"><xacro:include filename="b.xacro"/></robot>' },
    { path: 'b.xacro', content: '<robot xmlns:xacro="x"><xacro:include filename="a.xacro"/></robot>' },
  ];
  const cycle = resolveXacroIncludes(loopFiles[0].content, 'a.xacro', loopFiles);
  assert.equal(cycle.diagnostics.length, 1);
  assert.match(cycle.diagnostics[0].message, /Circular/);
});

test('stripRobotWrapper is a no-op for fragments without a robot root', () => {
  assert.equal(stripRobotWrapper('<link name="a"/>'), '<link name="a"/>');
  assert.equal(
    stripRobotWrapper('<robot name="x"><link name="a"/></robot>').trim(),
    '<link name="a"/>',
  );
});
