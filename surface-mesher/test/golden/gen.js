'use strict';

// Regenerate test/golden/baseline.json from the pre-refactor implementation.
//
// The baseline freezes the *externally observable* behaviour of the old
// surface-discretisation code (vertex/triangle counts, deduplicated export
// size, independent grid error, and shared-edge welding) for the adjacent-patch
// and high-curvature scenes.  golden.test.js diffs the refactored default path
// against this file, so behaviour cannot silently drift.
//
// The old sources are recovered from git HEAD into a throwaway directory, so
// this script never depends on a hand-made copy and can be re-run any time:
//
//   node test/golden/gen.js
//
// It must be run from a checkout where HEAD still contains the original
// src/{mesher,error,export,scenes,surfaces,math}.js.

const { execFileSync } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const repoRoot = execFileSync('git', ['rev-parse', '--show-toplevel'], { encoding: 'utf8' }).trim();
const srcPrefix = path.relative(repoRoot, path.join(__dirname, '..', '..', 'src')).split(path.sep).join('/');

const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'surface-golden-'));
const files = ['mesher.js', 'error.js', 'export.js', 'scenes.js', 'surfaces.js', 'math.js'];
for (const f of files) {
  const blob = execFileSync('git', ['show', `HEAD:${srcPrefix}/${f}`], { cwd: repoRoot, encoding: 'utf8' });
  fs.writeFileSync(path.join(tmp, f), blob);
}

const { buildScene } = require(path.join(tmp, 'scenes.js'));
const { Mesher } = require(path.join(tmp, 'mesher.js'));
const { extractMesh, verify } = require(path.join(tmp, 'export.js'));

const cases = [
  { scene: 'weldedPair', params: {} },
  { scene: 'frame', params: {} },
  { scene: 'budgetWeld', params: {} },
  { scene: 'lips', params: {} },
  { scene: 'spike', params: {} },
  { scene: 'band', params: {} },
];
const out = {};
for (const c of cases) {
  const scene = buildScene(c.scene, c.params);
  const m = new Mesher(scene);
  const r = m.run();
  const mesh = extractMesh(m);
  const v = verify(m, mesh, { gridN: 24 });
  out[c.scene] = {
    totalVertices: r.totalVertices,
    totalTriangles: r.totalTriangles,
    globalVertices: mesh.vertices.length,
    faces: mesh.faces.length,
    gridError: v.patches.map((p) => ({
      name: p.name,
      maxSampledDistance: p.maxSampledDistance,
      maxSampledNormalAngle: p.maxSampledNormalAngle,
      tolerancesMet: p.tolerancesMet,
    })),
    sharedEdge: v.cracks.map((c2) => ({
      weld: c2.weld, maxGap: c2.maxGap, mismatchedVertices: c2.mismatchedVertices, crackFree: c2.crackFree,
    })),
  };
}

const dest = path.join(__dirname, 'baseline.json');
fs.writeFileSync(dest, JSON.stringify(out, null, 2) + '\n');
fs.rmSync(tmp, { recursive: true, force: true });
console.log('wrote', dest);
