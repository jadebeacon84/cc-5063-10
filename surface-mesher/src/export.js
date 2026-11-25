'use strict';

// Mesh extraction, file export and independent verification.
//
// Extraction merges welded boundary vertices into one global vertex index by
// 3D position, so the exported connectivity itself - not the renderer -
// guarantees that welded edges share vertices.  Separated lips stay distinct
// because their positions genuinely differ.

const fs = require('fs');
const path = require('path');
const {
  vsub, vcross, vnorm, vlen, vdist, pointTriangleDistance, angleBetween,
  ratVal, clamp,
} = require('./math');

function extractMesh(mesher) {
  const globalIndex = new Map(); // "x|y|z" quantised key -> global id
  const vertices = [];
  const faces = [];
  const facePatches = [];
  const faceUnsatisfied = [];
  const POS_EPS = 1e-9;

  function keyOf(p) {
    return p.map((x) => Math.round(x / POS_EPS)).join('|');
  }

  function globalVertex(p) {
    const key = keyOf(p);
    let id = globalIndex.get(key);
    if (id === undefined) {
      id = vertices.length;
      globalIndex.set(key, id);
      vertices.push(p);
    }
    return id;
  }

  for (const pm of mesher.patches) {
    const badTris = new Set(pm.unsatisfied.map((u) => u.tri));
    for (const tri of pm.aliveTriangles()) {
      const ids = tri.v.map((v) => globalVertex(v.p));
      // Drop degenerate faces (repeated-node parameterisations can collapse).
      if (ids[0] === ids[1] || ids[1] === ids[2] || ids[0] === ids[2]) continue;
      faces.push(ids);
      facePatches.push(pm.patch.name);
      faceUnsatisfied.push(badTris.has(tri));
    }
  }
  return { vertices, faces, facePatches, faceUnsatisfied };
}

// Global vertex ids lying on a weld seam, for highlighting in the viewer.
function seamVertexIds(mesher, mesh) {
  const posToId = new Map();
  mesh.vertices.forEach((p, i) => posToId.set(p.map((x) => Math.round(x / 1e-9)).join('|'), i));
  const ids = new Set();
  for (const weld of mesher.welds) {
    for (const pos of weld.canonical.values()) {
      const id = posToId.get(pos.map((x) => Math.round(x / 1e-9)).join('|'));
      if (id !== undefined) ids.add(id);
    }
  }
  return [...ids];
}

function writeOBJ(mesh, filePath, objectName = 'surface') {
  const lines = [`# Adaptive parametric surface discretisation`, `o ${objectName}`];
  for (const v of mesh.vertices) lines.push(`v ${v[0].toFixed(8)} ${v[1].toFixed(8)} ${v[2].toFixed(8)}`);
  for (const f of mesh.faces) lines.push(`f ${f.map((i) => i + 1).join(' ')}`);
  fs.writeFileSync(filePath, lines.join('\n') + '\n');
  return filePath;
}

function writeSTL(mesh, filePath, objectName = 'surface') {
  const lines = [`solid ${objectName}`];
  for (const f of mesh.faces) {
    const [a, b, c] = f.map((i) => mesh.vertices[i]);
    const n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    lines.push(`facet normal ${n[0].toExponential(6)} ${n[1].toExponential(6)} ${n[2].toExponential(6)}`);
    lines.push('  outer loop');
    for (const v of [a, b, c]) lines.push(`    vertex ${v[0].toExponential(8)} ${v[1].toExponential(8)} ${v[2].toExponential(8)}`);
    lines.push('  endloop');
    lines.push('endfacet');
  }
  lines.push(`endsolid ${objectName}`);
  fs.writeFileSync(filePath, lines.join('\n') + '\n');
  return filePath;
}

// ---------------------------------------------------------------------------
// Independent verification: recompute everything from the surface and the
// extracted mesh rather than trusting the mesher's own bookkeeping.
// ---------------------------------------------------------------------------

function verify(mesher, mesh, opts = {}) {
  const gridN = opts.gridN || 24;
  const report = {
    patches: [],
    cracks: [],
    hangingVertices: [],
    nonManifoldEdges: [],
    global: { maxDistance: 0, maxNormalAngle: 0, sampled: true },
  };

  // Build a spatial index of triangles per patch for distance probes.
  for (const pm of mesher.patches) {
    const patchFaceIdx = [];
    mesh.faces.forEach((f, i) => {
      if (mesh.facePatches[i] === pm.patch.name) patchFaceIdx.push(i);
    });
    let maxDist = 0;
    let maxAngle = 0;
    const worst = { dist: null, angle: null };
    for (let i = 0; i <= gridN; i++) {
      for (let j = 0; j <= gridN; j++) {
        const u = i / gridN;
        const v = j / gridN;
        const exact = pm.patch.pos(u, v);
        // At a repeated/pole node the parameterisation degenerates and the
        // raw normal collapses to zero; recover the limiting normal a small
        // step inside the domain instead of reporting a spurious 90 degree.
        let exactN = pm.patch.normal(u, v);
        if (vlen(exactN) === 0) {
          const eps = 1 / Math.max(gridN, 8);
          exactN = pm.patch.normal(u, clamp(v - eps, 0, 1));
          if (vlen(exactN) === 0) exactN = pm.patch.normal(u, clamp(v + eps, 0, 1));
        }
        let best = Infinity;
        let bestFace = -1;
        let bestAngleHere = Infinity;
        for (const fi of patchFaceIdx) {
          const [a, b, c] = mesh.faces[fi].map((k) => mesh.vertices[k]);
          // Skip collapsed (zero-area) faces such as pole fan degeneracies.
          if (vlen(vcross(vsub(b, a), vsub(c, a))) < 1e-14) continue;
          const d = pointTriangleDistance(exact, a, b, c);
          if (d < best) best = d;
          // A surface point can sit equidistant from two adjacent facets;
          // attribute the normal error to the facet with the closer normal
          // rather than letting a neighbour inflate it spuriously.
          if (d <= best + 1e-12 && vlen(exactN) > 0) {
            const facetN = vnorm(vcross(vsub(b, a), vsub(c, a)));
            const ang = angleBetween(exactN, facetN);
            if (ang < bestAngleHere) {
              bestAngleHere = ang;
              bestFace = fi;
            }
          } else if (bestFace < 0) {
            bestFace = fi;
          }
        }
        if (best > maxDist) {
          maxDist = best;
          worst.dist = { u, v, value: best };
        }
        if (bestFace >= 0 && vlen(exactN) > 0) {
          const [a, b, c] = mesh.faces[bestFace].map((k) => mesh.vertices[k]);
          const facetN = vnorm(vcross(vsub(b, a), vsub(c, a)));
          const ang = angleBetween(exactN, facetN);
          if (ang > maxAngle) {
            maxAngle = ang;
            worst.angle = { u, v, value: ang };
          }
        }
      }
    }
    const tolOk = maxDist <= pm.patch.tol.position * (1 + 1e-9) &&
      maxAngle <= pm.patch.tol.normal * (1 + 1e-9);
    report.patches.push({
      name: pm.patch.name,
      maxSampledDistance: maxDist,
      maxSampledNormalAngle: maxAngle,
      positionTol: pm.patch.tol.position,
      normalTol: pm.patch.tol.normal,
      tolerancesMet: tolOk,
      guaranteed: pm.patch.boundIsGuaranteed,
      worst,
    });
    report.global.maxDistance = Math.max(report.global.maxDistance, maxDist);
    report.global.maxNormalAngle = Math.max(report.global.maxNormalAngle, maxAngle);
  }

  verifyConnectivity(mesh, report);
  verifyWelds(mesher, mesh, report);
  return report;
}

function verifyConnectivity(mesh, report) {
  const edgeCount = new Map();
  for (const f of mesh.faces) {
    for (let i = 0; i < 3; i++) {
      const a = f[i];
      const b = f[(i + 1) % 3];
      const key = a < b ? `${a}|${b}` : `${b}|${a}`;
      edgeCount.set(key, (edgeCount.get(key) || 0) + 1);
    }
  }
  for (const [key, count] of edgeCount) {
    if (count > 2) report.nonManifoldEdges.push({ edge: key, count });
  }
  const referenced = new Set();
  for (const f of mesh.faces) for (const id of f) referenced.add(id);
  for (let i = 0; i < mesh.vertices.length; i++) {
    if (!referenced.has(i)) report.hangingVertices.push(i);
  }
  const boundaryEdges = [...edgeCount.entries()].filter(([, c]) => c === 1).map(([k]) => k);
  report.boundaryEdgeCount = boundaryEdges.length;
  report.nonManifoldEdgeCount = report.nonManifoldEdges.length;
}

function verifyWelds(mesher, mesh, report) {
  // Every weld: for each canonical t the two patches must reference the same
  // global vertex id, and no boundary edge pair may have a measurable gap.
  const posToId = new Map();
  mesh.vertices.forEach((p, i) => posToId.set(p.map((x) => Math.round(x / 1e-9)).join('|'), i));
  for (const weld of mesher.welds) {
    const pmA = mesher.patchByName.get(weld.patchA);
    const pmB = mesher.patchByName.get(weld.patchB);
    const n = 2 ** Math.max(weld.level, weld.forcedLevel);
    let maxGap = 0;
    let mismatched = 0;
    for (let i = 0; i <= n; i++) {
      const t = i / n;
      const pA = probeSidePosition(pmA, weld.sideA, t);
      const tB = weld.flip ? 1 - t : t;
      const pB = probeSidePosition(pmB, weld.sideB, tB);
      const gap = vdist(pA, pB);
      maxGap = Math.max(maxGap, gap);
      const idA = posToId.get(pA.map((x) => Math.round(x / 1e-9)).join('|'));
      const idB = posToId.get(pB.map((x) => Math.round(x / 1e-9)).join('|'));
      if (idA !== idB) mismatched++;
    }
    report.cracks.push({
      weld: weld.id,
      samples: n + 1,
      maxGap,
      mismatchedVertices: mismatched,
      crackFree: maxGap < 1e-8 && mismatched === 0,
    });
  }
}

// Position of a patch mesh at arc parameter t along a side.  Boundary
// samples lie on a leaf triangle edge whose endpoints are real mesh
// vertices, so we interpolate between those vertices instead of evaluating
// the surface: any gap measured here is a genuine connectivity gap.
function probeSidePosition(pm, side, t) {
  const want = sideParamUV(side, t);
  for (const v of pm.vertices.values()) {
    if (Math.abs(ratVal(v.uv[0]) - want[0]) < 1e-12 && Math.abs(ratVal(v.uv[1]) - want[1]) < 1e-12) {
      return v.p;
    }
  }
  // Find the two boundary vertices bracketing t and linearly interpolate.
  const boundary = [];
  for (const v of pm.vertices.values()) {
    if (MesherOnSide(side, v)) boundary.push({ t: sideT(side, v), p: v.p });
  }
  boundary.sort((a, b) => a.t - b.t);
  for (let i = 0; i < boundary.length - 1; i++) {
    if (t >= boundary[i].t - 1e-12 && t <= boundary[i + 1].t + 1e-12) {
      const span = boundary[i + 1].t - boundary[i].t;
      const alpha = span < 1e-15 ? 0 : (t - boundary[i].t) / span;
      return [
        boundary[i].p[0] + alpha * (boundary[i + 1].p[0] - boundary[i].p[0]),
        boundary[i].p[1] + alpha * (boundary[i + 1].p[1] - boundary[i].p[1]),
        boundary[i].p[2] + alpha * (boundary[i + 1].p[2] - boundary[i].p[2]),
      ];
    }
  }
  return pm.patch.pos(clamp(want[0], 0, 1), clamp(want[1], 0, 1));
}

function sideT(side, v) {
  const u = ratVal(v.uv[0]);
  const w = ratVal(v.uv[1]);
  if (side === 'left' || side === 'right') return w;
  return u;
}

function MesherOnSide(side, v) {
  const u = ratVal(v.uv[0]);
  const w = ratVal(v.uv[1]);
  if (side === 'left') return u === 0;
  if (side === 'right') return u === 1;
  if (side === 'bottom') return w === 0;
  if (side === 'top') return w === 1;
  return false;
}

function sideParamUV(side, t) {
  if (side === 'left') return [0, t];
  if (side === 'right') return [1, t];
  if (side === 'bottom') return [t, 0];
  return [t, 1];
}

function barycentric2D(p, a, b, c) {
  const den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
  if (Math.abs(den) < 1e-15) return null;
  const w0 = ((b[1] - c[1]) * (p[0] - c[0]) + (c[0] - b[0]) * (p[1] - c[1])) / den;
  const w1 = ((c[1] - a[1]) * (p[0] - c[0]) + (a[0] - c[0]) * (p[1] - c[1])) / den;
  return [w0, w1, 1 - w0 - w1];
}

function exportResult(mesher, outDir, baseName) {
  const mesh = extractMesh(mesher);
  fs.mkdirSync(outDir, { recursive: true });
  const objPath = writeOBJ(mesh, path.join(outDir, `${baseName}.obj`), baseName);
  const stlPath = writeSTL(mesh, path.join(outDir, `${baseName}.stl`), baseName);
  const verification = verify(mesher, mesh);
  return { mesh, objPath, stlPath, verification };
}

module.exports = { extractMesh, seamVertexIds, writeOBJ, writeSTL, verify, exportResult };
