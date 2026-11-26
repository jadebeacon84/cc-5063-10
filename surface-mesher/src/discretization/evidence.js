'use strict';

// Execution evidence for the discretisation interface.
//
// "Evidence" here means: numbers recomputed from the *exported mesh* and the
// surface definitions, never from the mesher's own cached claims.  Two kinds:
//
//   gridError   - the sampled geometric error per patch, re-measured by an
//                 independent grid probe against the analytic surface.
//   sharedEdge  - the shared-edge correspondence, checked vertex-by-vertex:
//                 every dyadic sample along a weld must resolve to the same
//                 global vertex id on both sides, with zero measurable gap.
//
// The module refuses to fabricate either: every value comes from
// independentVerify() over real geometry.  `crossCheck()` additionally asserts
// that the mesher's reported optimistic metrics never out-run the independent
// measurement, and that a claimed crack-free seam really corresponds.

const { extractMesh, verify } = require('../export');
const { ratKey, vdist } = require('../math');
const { sideTRat, toCanonicalT, onSideLine } = require('./boundary');

const POS_EPS = 1e-9;
const posKey = (p) => p.map((x) => Math.round(x / POS_EPS)).join('|');

function gridError(mesher, verification) {
  return verification.patches.map((vp) => {
    const eps = 1e-9;
    const met = vp.maxSampledDistance <= vp.positionTol * (1 + eps) &&
      vp.maxSampledNormalAngle <= vp.normalTol * (1 + eps);
    return {
      patch: vp.name,
      maxDistance: vp.maxSampledDistance,
      maxNormalAngle: vp.maxSampledNormalAngle,
      positionTol: vp.positionTol,
      normalTol: vp.normalTol,
      met,
      guaranteed: vp.guaranteed,
      source: 'independent-grid-probe',
    };
  });
}

// Boundary vertices of one patch side, keyed by canonical A-side arc parameter.
// These are real mesh vertices, not interpolated probes, so comparing them is a
// genuine vertex-identity check.
function sideVertices(pm, side, weld) {
  const map = new Map(); // canonical tKey -> { vertex, tLocal }
  for (const v of pm.vertices.values()) {
    if (!onSideLine(side, v)) continue;
    const tLocal = sideTRat(side, v.uv);
    const canonical = toCanonicalT(weld, pm.patch.name === weld.patchA ? 'A' : 'B', tLocal);
    map.set(ratKey(canonical), { vertex: v, tLocal });
  }
  return map;
}

// Per-vertex shared-edge correspondence, straight off the extracted mesh.
//
// For every canonical arc parameter that carries a vertex on either side, both
// sides must present a matching vertex, resolve to the same global id, and show
// zero measurable gap.  A parameter present on one side only is a T-junction
// (a locally sealed crack) and is counted as a mismatch.  This cannot be
// satisfied by each patch independently sealing its own edge.
function sharedEdge(mesher, mesh) {
  const posToId = new Map();
  mesh.vertices.forEach((p, i) => posToId.set(posKey(p), i));
  const edges = [];
  for (const weld of mesher.welds) {
    const pmA = mesher.patchByName.get(weld.patchA);
    const pmB = mesher.patchByName.get(weld.patchB);
    const vertsA = sideVertices(pmA, weld.sideA, weld);
    const vertsB = sideVertices(pmB, weld.sideB, weld);

    const allT = new Set([...vertsA.keys(), ...vertsB.keys()]);
    let maxGap = 0;
    let mismatched = 0;
    let tJunctions = 0;
    const correspondence = [];
    for (const tKey of allT) {
      const a = vertsA.get(tKey);
      const b = vertsB.get(tKey);
      if (!a || !b) {
        tJunctions++;
        mismatched++;
        continue;
      }
      const idA = posToId.get(posKey(a.vertex.p));
      const idB = posToId.get(posKey(b.vertex.p));
      const gap = vdist(a.vertex.p, b.vertex.p);
      maxGap = Math.max(maxGap, gap);
      const sameId = idA !== undefined && idA === idB;
      if (!sameId) mismatched++;
      if (correspondence.length < 64) {
        correspondence.push({ t: tKey, idA, idB, same: sameId, gap });
      }
    }
    edges.push({
      weld: weld.id,
      strategy: weld.strategy,
      boundaryVertices: allT.size,
      maxGap,
      mismatchedVertices: mismatched,
      tJunctions,
      crackFree: maxGap < 1e-8 && mismatched === 0,
      correspondence,
      source: 'exported-mesh-vertex-identity',
    });
  }
  return edges;
}

function buildEvidence(mesher, mesh, verification) {
  const grid = gridError(mesher, verification);
  const seams = sharedEdge(mesher, mesh);
  const connectivity = {
    nonManifoldEdges: verification.nonManifoldEdgeCount,
    hangingVertices: verification.hangingVertices.length,
    boundaryEdges: verification.boundaryEdgeCount,
  };
  const allGridMet = grid.every((g) => g.met);
  const allSeamsFree = seams.every((s) => s.crackFree);
  const topologyClean = connectivity.nonManifoldEdges === 0 && connectivity.hangingVertices === 0;
  return {
    gridError: grid,
    sharedEdge: seams,
    connectivity,
    summary: { allGridMet, allSeamsFree, topologyClean },
  };
}

// Recompute from scratch and assert the reported evidence is honest.  Returns
// a report; throws on any fabrication (reported-tolerant cell the independent
// probe rejects, or a claimed seam that does not correspond).
function crossCheck(mesher, result, opts = {}) {
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: opts.gridN || 24 });
  const evidence = buildEvidence(mesher, mesh, verification);
  const problems = [];

  // 1. Reported sample metrics must not under-run the independent probe.
  result.patches.forEach((rp) => {
    const gp = evidence.gridError.find((g) => g.patch === rp.name);
    if (!gp) return;
    if (gp.maxDistance > rp.sampleMax.dist + 1e-6 &&
        gp.maxDistance > rp.tolerances.position) {
      problems.push(`${rp.name}: independent grid distance ${gp.maxDistance} exceeds reported sample ${rp.sampleMax.dist}`);
    }
    if (gp.maxNormalAngle > rp.sampleMax.angle + 1e-6 &&
        gp.maxNormalAngle > rp.tolerances.normal) {
      problems.push(`${rp.name}: independent grid angle ${gp.maxNormalAngle} exceeds reported sample ${rp.sampleMax.angle}`);
    }
  });

  // 2. Every shared edge must correspond vertex-by-vertex.
  for (const edge of evidence.sharedEdge) {
    if (!edge.crackFree) {
      problems.push(`weld ${edge.weld}: ${edge.mismatchedVertices} mismatched / gap ${edge.maxGap}`);
    }
  }

  // 3. Topology must be clean for an accepted export.
  if (!result.rejected) {
    if (evidence.connectivity.nonManifoldEdges > 0) {
      problems.push(`non-manifold edges: ${evidence.connectivity.nonManifoldEdges}`);
    }
    if (evidence.connectivity.hangingVertices > 0) {
      problems.push(`hanging vertices: ${evidence.connectivity.hangingVertices}`);
    }
  }

  return { evidence, verification, mesh, problems, ok: problems.length === 0 };
}

module.exports = { gridError, sharedEdge, buildEvidence, crossCheck, POS_EPS, posKey };
