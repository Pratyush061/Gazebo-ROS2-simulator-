import test from 'node:test';
import assert from 'node:assert/strict';
import { compileUrdf } from './urdfCompiler';
import { STARTER_TEMPLATES } from './templates';
import {
  buildDisplayTreeRows,
  collectTopicOptions,
  listModelLinks,
} from './rvizDisplayTree';

const robotTemplate =
  STARTER_TEMPLATES.find((t) => !t.isXacro) || STARTER_TEMPLATES[0];

test('hierarchy tree rows start at the root link with depth 0', () => {
  const model = compileUrdf(robotTemplate.urdfXml);
  const rows = buildDisplayTreeRows(model, { treeStyle: 'hierarchy' });

  assert.ok(rows.length > 0, 'expected tree rows for a valid URDF');
  assert.equal(rows[0].kind, 'link');
  assert.equal(rows[0].name, model.rootLink);
  assert.equal(rows[0].depth, 0);

  // Every joint row is immediately followed by its child link row.
  rows.forEach((row, i) => {
    if (row.kind === 'joint') {
      assert.equal(rows[i + 1]?.kind, 'link');
      assert.equal(rows[i + 1].depth, row.depth + 1);
    }
  });
});

test('alphabetic mode lists every link exactly once, sorted', () => {
  const model = compileUrdf(robotTemplate.urdfXml);
  const rows = buildDisplayTreeRows(model, { treeStyle: 'alphabetic' });
  const names = rows.map((r) => r.name);
  const linkCount = Object.keys(model.links).length;

  assert.equal(names.length, linkCount);
  assert.equal(new Set(names).size, linkCount);
  assert.deepEqual(names, [...names].sort((a, b) => a.localeCompare(b)));
  assert.ok(rows.every((r) => r.kind === 'link' && r.depth === 0));
});

test('listModelLinks returns root-first kinematic order', () => {
  const model = compileUrdf(robotTemplate.urdfXml);
  const links = listModelLinks(model);
  assert.equal(links[0], model.rootLink);
  assert.equal(links.length, Object.keys(model.links).length);
});

test('topic options always include the display default and dedupe bridged topics', () => {
  assert.deepEqual(collectTopicOptions('/scan', ['/scan', '/odom']), [
    '/scan',
    '/odom',
  ]);
  assert.deepEqual(collectTopicOptions('/imu', []), ['/imu']);
});
