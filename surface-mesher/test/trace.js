'use strict';

// Legacy surface-discretisation sample, adapted to the refactored interface.
//
// It traces a single patch's refinement pass by pass and runs to completion.
// Each pass evaluates every leaf once (consuming the explicit estimates the
// error module produces), splits the failing leaves, and reports the new
// density and worst estimate.  The old version called an internal
// `_nextAction()` that the driver never exposed; this drives the real driver
// through its public pass mechanics instead.

const { buildScene } = require('../src/scenes');
const { Mesher } = require('../src/discretization');

const scene = buildScene(process.argv[2] || 'bump', {});
const mesher = new Mesher(scene);
const pm = mesher.patches[0];

mesher._seed();
console.log('seeded, tris', pm.aliveTriangles().length, 'metric', mesher.estimator.name);

let maxDepth = 0;
let pass = 0;
for (; pass < 80; pass++) {
  mesher._runPass = pass;
  mesher._repairConformity();
  const failing = [];
  for (const tri of pm.aliveTriangles()) {
    if (tri.blocked) continue;
    const verdict = mesher._evaluate(pm, tri);
    if (verdict.needsRefine) failing.push({ pm, tri, verdict });
  }
  if (failing.length === 0) {
    console.log('CONVERGED pass', pass);
    break;
  }
  failing.sort((a, b) => b.verdict.metrics.h - a.verdict.metrics.h);
  const worst = failing[0].verdict;
  maxDepth = Math.max(maxDepth, failing[0].tri.depth);
  if (pass % 5 === 0) {
    console.log('pass', pass, 'tris', pm.aliveTriangles().length,
      'failing', failing.length, 'worstDepth', failing[0].tri.depth,
      'posBound', worst.metrics.posBound.toExponential(2),
      'sampled', worst.metrics.sampledDist.toExponential(2),
      'h', worst.metrics.h.toExponential(2));
  }
  for (const item of failing) {
    if (item.tri.alive && !item.tri.blocked) mesher._execute(item);
  }
}

console.log('final tris', pm.aliveTriangles().length, 'verts', pm.vertices.size, 'maxDepth', maxDepth);
console.log('completed', pass < 80 ? 'cleanly' : 'at pass cap');
