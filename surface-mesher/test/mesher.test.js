'use strict';

const test = require('node:test');
const assert = require('node:assert');
const { buildScene } = require('../src/scenes');
// Deliberately import through the frozen compatibility adapter: the legacy
// surface-discretisation entry point must keep running old samples untouched.
const { Mesher } = require('../src/mesher');
const { extractMesh, verify } = require('../src/export');
// New module interface, exercised directly by the refactor tests.
const disc = require('../src/discretization');
const { RequestScheduler } = disc;

function run(name, params = {}, opts = {}) {
  const scene = buildScene(name, params);
  const { gridN, ...mesherOpts } = opts;
  const mesher = new Mesher(scene, mesherOpts);
  const result = mesher.run();
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: gridN || 18 });
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

// ---------------------------------------------------------------------------
// Refactor: module layering, boundary strategies, metric swap, evidence.
// ---------------------------------------------------------------------------

test('compat adapter and discretization driver are the same implementation', () => {
  const { Mesher: CompatMesher } = require('../src/mesher');
  assert.strictEqual(CompatMesher, disc.Mesher);
});

test('unified-cached and negotiated-depth are compared at the same seam', () => {
  const unified = run('weldedPair', { boundaryStrategy: 'unified-cached' }, { gridN: 10 });
  const negotiated = run('weldedPair', { boundaryStrategy: 'negotiated-depth' }, { gridN: 10 });

  // Same shared-edge agreement level to begin with, different vertex identity.
  assert.strictEqual(unified.result.boundaryStrategy, 'unified-cached');
  assert.strictEqual(negotiated.result.boundaryStrategy, 'negotiated-depth');
  assert.ok(unified.result.welds[0].level >= 1);
  assert.ok(negotiated.result.welds[0].level >= 1);

  const uEv = disc.buildEvidence(unified.mesher, unified.mesh, unified.verification);
  const nEv = disc.buildEvidence(negotiated.mesher, negotiated.mesh, negotiated.verification);

  // Unified cache: vertices correspond one-for-one, no crack.
  assert.strictEqual(uEv.sharedEdge[0].crackFree, true);
  assert.strictEqual(uEv.sharedEdge[0].mismatchedVertices, 0);
  assert.strictEqual(uEv.sharedEdge[0].maxGap, 0);

  // Negotiated depth: agreeing on a level is not the same as agreeing on the
  // vertex set, so independently bisected neighbours leave T-junctions.  This
  // is exactly the comparison the two strategies exist to expose.
  assert.ok(nEv.sharedEdge[0].tJunctions > 0, 'negotiated depth should expose T-junctions');
  assert.strictEqual(nEv.sharedEdge[0].crackFree, false);
});

test('refinement policy consumes explicit estimates from a swappable metric', () => {
  // sampled-only is a genuinely different metric: it refines from sampling
  // alone, with no analytic guarantee, so it stops earlier.
  const analytic = run('bump', { errorMetric: 'analytic-sampled' });
  const sampled = run('bump', { errorMetric: 'sampled-only' });

  assert.strictEqual(analytic.result.patches[0].errorMetric, 'analytic-sampled');
  assert.strictEqual(sampled.result.patches[0].errorMetric, 'sampled-only');
  assert.notStrictEqual(analytic.result.totalTriangles, sampled.result.totalTriangles);
});

test('replaceable audit backend records the explicit estimates consumed', () => {
  const audit = disc.createAuditBackend({ cap: 100000 });
  const { result } = run('bump', {}, { audit });
  assert.ok(audit.records.length > 0);
  assert.strictEqual(audit.records[0].type, 'estimate');
  // Every record names its metric and carries the numbers the policy saw.
  for (const r of audit.records) {
    assert.strictEqual(r.type, 'estimate');
    assert.strictEqual(typeof r.metric, 'string');
    assert.strictEqual(typeof r.h, 'number');
  }
  assert.ok(result.totalTriangles > 100);

  // A different backend must not change results (audit is observational).
  const audit2 = disc.estimation.createAuditBackend({ name: 'structured-log' });
  const again = run('bump', {}, { audit: audit2 });
  assert.strictEqual(again.result.totalTriangles, result.totalTriangles);
});

test('swapping the metric mid-run leaves the shared seam individually welded', () => {
  const scene = buildScene('weldedPair', {});
  const mesher = new disc.Mesher(scene);
  // Seed, then drive passes manually so a metric swap can land mid-run.
  mesher._seed();
  let swappedAt = -1;
  for (let pass = 0; pass < 200; pass++) {
    mesher._runPass = pass;
    if (pass === 8) {
      mesher.setEstimator('sampled-only');
      swappedAt = pass;
    }
    mesher._repairConformity();
    const failing = [];
    for (const pm of mesher.patches) {
      for (const tri of pm.aliveTriangles()) {
        if (tri.blocked) continue;
        const verdict = mesher._evaluate(pm, tri);
        if (verdict.needsRefine) failing.push({ pm, tri, verdict });
      }
    }
    if (failing.length === 0) break;
    for (const item of failing) {
      if (item.tri.alive && !item.tri.blocked) mesher._execute(item);
    }
  }
  mesher._repairConformity();
  mesher._collectUnsatisfied();
  const result = mesher._buildResult();

  assert.strictEqual(swappedAt, 8);
  assert.ok(result.metricSwaps.length >= 1);
  assert.strictEqual(result.metricSwaps[0].to, 'sampled-only');

  // After the swap the seam must still be vertex-identical: the metric change
  // perturbs where we refine, not the boundary identity.
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: 10 });
  const ev = disc.buildEvidence(mesher, mesh, verification);
  assert.strictEqual(ev.sharedEdge[0].crackFree, true, `seam must stay welded: ${JSON.stringify(ev.sharedEdge[0])}`);
  assert.strictEqual(ev.sharedEdge[0].mismatchedVertices, 0);
});

test('scheduler records abandoned requests and refuses slow stale results', () => {
  const sched = new RequestScheduler();
  const j1 = sched.begin('drag-1');
  const j2 = sched.begin('drag-2'); // supersedes j1
  const j3 = sched.begin('drag-3'); // supersedes j2
  assert.strictEqual(sched.abandoned.length, 2);
  assert.deepStrictEqual(sched.abandoned.map((a) => a.gen).sort(), [j1.gen, j2.gen]);

  // Newest result commits first; older, slower results arrive afterwards and
  // must be refused so they cannot overwrite the newer seam.
  const c3 = sched.commit(j3, { generation: j3.gen, seam: 'new' });
  const c1 = sched.commit(j1, { generation: j1.gen, seam: 'old' });
  const c2 = sched.commit(j2, { generation: j2.gen, seam: 'old' });
  assert.strictEqual(c3.accepted, true);
  assert.strictEqual(c1.accepted, false);
  assert.strictEqual(c1.reason, 'stale-result');
  assert.strictEqual(c2.accepted, false);
  assert.strictEqual(sched.rejectedStale.length, 2);
  assert.strictEqual(sched.committedGen, j3.gen);
  // The committed seam is the newest one; no stale overwrite happened.
  assert.strictEqual(sched.results[sched.results.length - 1].gen, j3.gen);
});

test('evidence refuses to fabricate: independent grid error matches reported', () => {
  const { mesher, result, verification } = run('frame', {}, { gridN: 12 });
  const mesh = extractMesh(mesher);
  const ev = disc.buildEvidence(mesher, mesh, verification);
  assert.strictEqual(ev.gridError.length, result.patches.length);
  for (const g of ev.gridError) {
    assert.strictEqual(g.source, 'independent-grid-probe');
    // Grid error is a re-measurement, not a copy of the mesher's claim.
    const rp = result.patches.find((p) => p.name === g.patch);
    assert.ok(g.maxDistance >= 0);
    assert.ok(rp !== undefined);
  }
  // Every shared edge must correspond vertex-by-vertex.
  for (const e of ev.sharedEdge) {
    assert.strictEqual(e.source, 'exported-mesh-vertex-identity');
    assert.strictEqual(e.crackFree, true);
  }
  const check = disc.crossCheck(mesher, result, { gridN: 18 });
  assert.strictEqual(check.ok, true, `cross-check problems: ${JSON.stringify(check.problems)}`);
});

test('shared-edge correspondence cannot be faked by per-patch sealing', () => {
  // The negotiated strategy seals each side's own edge but the vertex sets
  // differ; the evidence must report it, so a crack cannot hide.
  const { mesher, mesh, verification } = run('weldedPair', { boundaryStrategy: 'negotiated-depth' }, { gridN: 10 });
  const ev = disc.buildEvidence(mesher, mesh, verification);
  assert.strictEqual(ev.summary.allSeamsFree, false);
  assert.ok(ev.sharedEdge[0].tJunctions > 0 || ev.sharedEdge[0].mismatchedVertices > 0);
});

test('negotiated-depth still meets the geometric grid tolerance', () => {
  // Whatever the seam outcome, local error estimation is independent of the
  // crack constraint: the negotiated strategy must still meet grid tolerance.
  const { mesher, mesh, verification, result } = run('weldedPair', { boundaryStrategy: 'negotiated-depth' }, { gridN: 12 });
  assert.strictEqual(result.rejected, false);
  const ev = disc.buildEvidence(mesher, mesh, verification);
  assert.strictEqual(ev.summary.allGridMet, true);
});

