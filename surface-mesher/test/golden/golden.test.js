'use strict';

// Golden before/after regression on adjacent patches and high-curvature scenes.
//
// baseline.json was generated from the pre-refactor implementation (see
// test/golden/gen.js).  Here we run the *refactored default path* and diff:
//
//   - mesh density and export size must match exactly,
//   - independent grid error per patch must match within float tolerance,
//   - shared-edge welding (maxGap / mismatchedVertices) must match exactly.
//
// This is the equivalence harness that ties grid error and shared-edge
// correspondence to committed execution evidence, rather than to hand-edited
// data.  It runs against a working tree only: no network, no manual fixtures.

const test = require('node:test');
const assert = require('node:assert');
const fs = require('fs');
const path = require('path');
const { buildScene } = require('../../src/scenes');
const { Mesher } = require('../../src/discretization');
const { extractMesh, verify } = require('../../src/export');
const { buildEvidence } = require('../../src/discretization/evidence');

const baseline = JSON.parse(fs.readFileSync(path.join(__dirname, 'baseline.json'), 'utf8'));

function runDefault(sceneName) {
  const scene = buildScene(sceneName, {});
  const mesher = new Mesher(scene);
  const result = mesher.run();
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: 24 });
  const evidence = buildEvidence(mesher, mesh, verification);
  return { result, mesh, verification, evidence };
}

for (const sceneName of Object.keys(baseline)) {
  test(`golden: ${sceneName} matches pre-refactor behaviour`, () => {
    const want = baseline[sceneName];
    const { result, mesh, verification, evidence } = runDefault(sceneName);

    // Density and export size: exact.
    assert.strictEqual(result.totalVertices, want.totalVertices, 'vertex count drifted');
    assert.strictEqual(result.totalTriangles, want.totalTriangles, 'triangle count drifted');
    assert.strictEqual(mesh.vertices.length, want.globalVertices, 'global vertex count drifted');
    assert.strictEqual(mesh.faces.length, want.faces, 'face count drifted');

    // Grid error: match per patch within float tolerance.
    assert.strictEqual(evidence.gridError.length, want.gridError.length);
    want.gridError.forEach((w, i) => {
      const g = evidence.gridError[i];
      assert.strictEqual(g.patch, w.name);
      assert.ok(Math.abs(g.maxDistance - w.maxSampledDistance) <= 1e-12 + 1e-6 * Math.abs(w.maxSampledDistance),
        `${sceneName}/${w.name}: grid distance drifted`);
      assert.ok(Math.abs(g.maxNormalAngle - w.maxSampledNormalAngle) <= 1e-12 + 1e-6 * Math.abs(w.maxSampledNormalAngle),
        `${sceneName}/${w.name}: grid angle drifted`);
      assert.strictEqual(g.met, w.tolerancesMet, `${sceneName}/${w.name}: tolerance verdict drifted`);
    });

    // Shared-edge correspondence: match per weld, exact.
    assert.strictEqual(evidence.sharedEdge.length, want.sharedEdge.length);
    want.sharedEdge.forEach((w, i) => {
      const e = evidence.sharedEdge[i];
      assert.strictEqual(e.weld, w.weld);
      assert.strictEqual(e.maxGap, w.maxGap, `${sceneName}/${w.weld}: maxGap drifted`);
      assert.strictEqual(e.mismatchedVertices, w.mismatchedVertices, `${sceneName}/${w.weld}: mismatches drifted`);
      assert.strictEqual(e.crackFree, w.crackFree, `${sceneName}/${w.weld}: crack verdict drifted`);
    });

    // Nothing was rejected by the default path.
    assert.strictEqual(result.rejected, false);
    // Verification agrees with the evidence summarisation.
    assert.strictEqual(verification.nonManifoldEdgeCount, 0);
  });
}
