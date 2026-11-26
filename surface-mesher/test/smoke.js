'use strict';

// Command-line helper for surface discretisation.
//
// The graphical entry point in the surface-editing viewport (web/) is the
// primary interface; this CLI exists only to drive the same discretisation
// from a terminal for scripting and spot checks.  It never invents data: every
// number comes from a real Mesher run followed by independent verification.

const { buildScene } = require('../src/scenes');
const { Mesher } = require('../src/discretization');
const { extractMesh, verify } = require('../src/export');
const { buildEvidence } = require('../src/discretization/evidence');

const sceneName = process.argv[2] || 'bump';
const params = {};
if (process.argv[3]) params.vertexCap = Number(process.argv[3]);
if (process.argv[4]) params.mode = process.argv[4];
if (process.argv[5]) params.errorMetric = process.argv[5];
if (process.argv[6]) params.boundaryStrategy = process.argv[6];

const scene = buildScene(sceneName, params);
const mesher = new Mesher(scene);
const result = mesher.run();
const mesh = extractMesh(mesher);
const report = verify(mesher, mesh);
const evidence = buildEvidence(mesher, mesh, report);

console.log(JSON.stringify({
  scene: sceneName,
  rejected: result.rejected,
  rejectReason: result.rejectReason,
  errorMetric: result.errorMetric,
  boundaryStrategy: result.boundaryStrategy,
  totalVertices: result.totalVertices,
  globalMeshVertices: mesh.vertices.length,
  totalTriangles: result.totalTriangles,
  cap: result.vertexCap,
  welds: result.welds,
  events: result.events.slice(0, 12),
  patches: result.patches.map((p) => ({
    name: p.name,
    verts: p.vertices,
    tris: p.triangles,
    guaranteed: p.guaranteed,
    errorMetric: p.errorMetric,
    relax: p.relax,
    sampleMax: {
      dist: p.sampleMax.dist.toExponential(3),
      angleDeg: (p.sampleMax.angle * 180 / Math.PI).toFixed(3),
    },
    boundMax: {
      dist: p.boundMax.dist.toExponential(3),
      angleDeg: (p.boundMax.angle * 180 / Math.PI).toFixed(3),
    },
    unsatisfied: p.unsatisfied.length,
    reasons: p.reasonCounts,
  })),
  // Execution evidence: grid error and shared-edge correspondence, recomputed
  // independently from the exported mesh (never hand-edited data).
  evidence: {
    summary: evidence.summary,
    gridError: evidence.gridError.map((g) => ({
      patch: g.patch,
      maxDist: g.maxDistance.toExponential(3),
      maxAngleDeg: (g.maxNormalAngle * 180 / Math.PI).toFixed(3),
      met: g.met,
    })),
    sharedEdge: evidence.sharedEdge.map((e) => ({
      weld: e.weld,
      boundaryVertices: e.boundaryVertices,
      maxGap: e.maxGap,
      mismatchedVertices: e.mismatchedVertices,
      tJunctions: e.tJunctions,
      crackFree: e.crackFree,
    })),
  },
  verification: {
    cracks: report.cracks,
    nonManifold: report.nonManifoldEdgeCount,
    hanging: report.hangingVertices.length,
    boundaryEdges: report.boundaryEdgeCount,
    perPatch: report.patches.map((p) => ({
      name: p.name,
      maxDist: p.maxSampledDistance.toExponential(3),
      maxAngleDeg: (p.maxSampledNormalAngle * 180 / Math.PI).toFixed(3),
      met: p.tolerancesMet,
    })),
  },
}, null, 2));
