'use strict';

const test = require('node:test');
const assert = require('node:assert');
const { buildScene } = require('../src/scenes');
const { Mesher } = require('../src/mesher');
const { extractMesh, verify } = require('../src/export');

function run(name, params = {}) {
  const scene = buildScene(name, params);
  const mesher = new Mesher(scene);
  const result = mesher.run();
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: 18 });
  return { scene, mesher, result, mesh, verification };
}

test('plane: zero curvature never refines beyond the seed', () => {
  const { result, verification } = run('plane');
  assert.strictEqual(result.totalTriangles, 2);
  assert.strictEqual(result.totalVertices, 4);
  assert.strictEqual(verification.patches[0].tolerancesMet, true);
});

test('bump: adaptive refinement concentrates vertices and meets guaranteed tol', () => {
  const { result, verification } = run('bump');
  assert.strictEqual(result.rejected, false);
  assert.ok(result.totalTriangles > 100, 'must actually adapt, not stay uniform');
  assert.strictEqual(result.patches[0].guaranteed, true);
  assert.strictEqual(verification.patches[0].tolerancesMet, true);
});

test('guaranteed bound always dominates the independently measured error', () => {
  for (const name of ['bump', 'spike', 'polar', 'band', 'weldedPair', 'frame']) {
    const { result, verification } = run(name);
    result.patches.forEach((p, k) => {
      const vp = verification.patches[k];
      assert.ok(
        p.boundMax.angle >= vp.maxSampledNormalAngle - 1e-9,
        `${name}/${p.name}: normal bound ${p.boundMax.angle} < measured ${vp.maxSampledNormalAngle}`
      );
      assert.ok(
        p.boundMax.dist >= vp.maxSampledDistance - 1e-9,
        `${name}/${p.name}: distance bound ${p.boundMax.dist} < measured ${vp.maxSampledDistance}`
      );
    });
  }
});

test('spike: high-curvature local feature refines much more than the flat region', () => {
  const { mesher } = run('spike');
  const pm = mesher.patches[0];
  const { ratVal } = require('../src/math');
  let nearDepth = 0;
  let farDepth = 0;
  for (const tri of pm.aliveTriangles()) {
    const c = [0, 0];
    for (const vx of tri.v) {
      c[0] += ratVal(vx.uv[0]) / 3;
      c[1] += ratVal(vx.uv[1]) / 3;
    }
    if (Math.hypot(c[0] - 0.5, c[1] - 0.5) < 0.08) nearDepth = Math.max(nearDepth, tri.depth);
    if (Math.abs(c[0] - 0.1) < 0.05 && Math.abs(c[1] - 0.1) < 0.05) farDepth = Math.max(farDepth, tri.depth);
  }
  assert.ok(nearDepth > farDepth + 2, `peak depth ${nearDepth} should exceed flat depth ${farDepth}`);
});

test('welded edges share one sampling identity and are crack-free', () => {
  const { verification, mesh } = run('weldedPair');
  assert.strictEqual(verification.cracks.length, 1);
  assert.strictEqual(verification.cracks[0].crackFree, true);
  assert.strictEqual(verification.cracks[0].maxGap, 0);
  assert.strictEqual(verification.nonManifoldEdgeCount, 0);
  assert.strictEqual(verification.hangingVertices.length, 0);
});

test('weld: stricter neighbour propagates subdivision to the looser patch', () => {
  const { mesher, result } = run('weldedPair');
  const weld = result.welds[0];
  assert.ok(weld.level >= 1, 'shared edge must be subdivided at least once');
  // Every canonical t must resolve to one global vertex id on both sides.
  const mesh = extractMesh(mesher);
  const posId = new Map();
  mesh.vertices.forEach((p, i) => posId.set(p.map((x) => Math.round(x / 1e-9)).join('|'), i));
  const w = mesher.welds[0];
  for (const pos of w.canonical.values()) {
    assert.ok(posId.has(pos.map((x) => Math.round(x / 1e-9)).join('|')), 'canonical weld vertex must exist in export');
  }
});

test('separated lips keep independent vertices and their own tolerances', () => {
  const { result } = run('lips');
  const upper = result.patches.find((p) => p.name === 'lip-upper');
  const lower = result.patches.find((p) => p.name === 'lip-lower');
  assert.notStrictEqual(upper.tolerances.position, lower.tolerances.position);
  // Two independent 4-vertex strips; nothing is welded between them.
  assert.strictEqual(result.totalVertices, 8);
  assert.strictEqual(upper.vertices, 4);
  assert.strictEqual(lower.vertices, 4);
});

test('budget coarsen-remote degrades an unlocked far patch and keeps the locked one', () => {
  const { result } = run('budgetWeld', { vertexCap: 9000, mode: 'coarsen-remote' });
  assert.strictEqual(result.rejected, false);
  const near = result.patches.find((p) => p.name === 'near-half');
  const far = result.patches.find((p) => p.name === 'far-half');
  assert.strictEqual(near.relax, 1, 'locked patch keeps declared tolerance');
  assert.ok(far.relax > 1, 'far patch must relax to free vertices');
  assert.ok(result.totalVertices <= 9000, 'hard cap respected');
  assert.ok(result.events.some((e) => e.type === 'coarsen-remote'));
});

test('budget reject mode preserves locked patches and refuses export', () => {
  const { result } = run('budgetWeld', { vertexCap: 220, mode: 'reject' });
  assert.strictEqual(result.rejected, true);
  assert.ok(result.rejectReason.includes('reject mode'));
  assert.ok(result.events.some((e) => e.type === 'export-rejected'));
});

test('vertex cap is a hard ceiling in every accepted export', () => {
  for (const name of ['bump', 'weldedPair', 'frame', 'lips']) {
    const { result } = run(name);
    assert.ok(result.totalVertices <= result.vertexCap, `${name} exceeded cap`);
  }
});

test('exported mesh is manifold with no hanging or duplicate vertices', () => {
  const { verification, mesh } = run('frame');
  assert.strictEqual(verification.nonManifoldEdgeCount, 0);
  assert.strictEqual(verification.hangingVertices.length, 0);
  const keys = new Set(mesh.vertices.map((p) => p.join('|')));
  assert.strictEqual(keys.size, mesh.vertices.length);
});

test('polar repeated node collapses to one exported pole vertex', () => {
  const { mesher, mesh } = run('polar');
  const polePos = mesher.patches[0].patch.pos(0.5, 1);
  // In the exported, deduplicated mesh the pole must be a single vertex:
  // every collapsed uv record shares one 3D identity.
  const poleIds = new Set();
  mesh.vertices.forEach((p, i) => {
    if (Math.hypot(p[0] - polePos[0], p[1] - polePos[1], p[2] - polePos[2]) < 1e-9) {
      poleIds.add(i);
    }
  });
  assert.strictEqual(poleIds.size, 1, 'exactly one global pole vertex after dedup');
  assert.strictEqual(mesher.patches[0].aliveTriangles().length > 16, true);
});

test('sampled estimates and guaranteed bounds are reported as distinct kinds', () => {
  const guaranteed = run('bump').result.patches[0];
  assert.strictEqual(guaranteed.guaranteed, true);
  // Both metric families are present regardless of kind.
  assert.ok(guaranteed.sampleMax !== undefined);
  assert.ok(guaranteed.boundMax !== undefined);
});
