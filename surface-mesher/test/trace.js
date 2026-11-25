'use strict';
const { buildScene } = require('../src/scenes');
const { Mesher } = require('../src/mesher');
const scene = buildScene(process.argv[2] || 'bump', {});
const m = new Mesher(scene);
m._seed();
const pm = m.patches[0];
console.log('seeded, tris', pm.aliveTriangles().length);
let maxDepth = 0;
for (let pass = 0; pass < 6000; pass++) {
  const action = m._nextAction();
  if (!action) {
    console.log('CONVERGED pass', pass);
    break;
  }
  maxDepth = Math.max(maxDepth, action.tri.depth);
  if (pass % 200 === 0) {
    console.log('pass', pass, 'tris', pm.aliveTriangles().length,
      'depth', action.tri.depth, 'maxD', maxDepth,
      'posBound', action.verdict.metrics.posBound.toExponential(2),
      'sampled', action.verdict.metrics.sampledDist.toExponential(2),
      'h', action.verdict.metrics.h.toExponential(2));
  }
  m._execute(action);
  if (pass === 5999) console.log('NO CONVERGE maxDepth', maxDepth);
}
console.log('final tris', pm.aliveTriangles().length, 'verts', pm.vertices.size);
