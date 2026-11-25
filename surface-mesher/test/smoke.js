'use strict';

const { buildScene } = require('../src/scenes');
const { Mesher } = require('../src/mesher');
const { extractMesh, verify } = require('../src/export');

const sceneName = process.argv[2] || 'bump';
const params = {};
if (process.argv[3]) params.vertexCap = Number(process.argv[3]);
if (process.argv[4]) params.mode = process.argv[4];

const scene = buildScene(sceneName, params);
const mesher = new Mesher(scene);
const result = mesher.run();
const mesh = extractMesh(mesher);
const report = verify(mesher, mesh);

console.log(JSON.stringify({
  scene: sceneName,
  rejected: result.rejected,
  rejectReason: result.rejectReason,
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
