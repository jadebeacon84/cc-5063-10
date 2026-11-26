'use strict';

// Local cell-error estimation.
//
// This layer answers one question only: "how far is this candidate cell from
// the underlying surface, and does that exceed the tolerance?"  It is
// deliberately blind to shared edges, cracks, budgets and neighbour policy.
// Crack constraints are a coordination concern (boundary.js); keeping them
// out of here is what lets the metric be swapped without perturbing welds.
//
// Two kinds of numbers are produced and must never be conflated:
//
//   guaranteed bounds  - surfaces with an analytic Hessian bound derive a
//                        chord bound (1/2) M h^2 and a derivative-Lipschitz
//                        normal-angle bound that hold for EVERY interior
//                        point, not only sampled ones.
//   sampled estimates  - maxima over a dense, size-adapted supersampling of
//                        the triangle plus an estimated Hessian.  They show
//                        what the discretisation looks like but cannot by
//                        themselves prove compliance at unsampled points.
//
// The active metric is a plain object with a stable interface:
//
//   {
//     name, kind,
//     evaluate({ surface, tri, depth, maxDepth, tol }) -> verdict
//   }
//
// verdict = { needsRefine, capped, reasons[], metrics{...} }.  The refinement
// driver consumes only this shape, so a different metric (sampled-only,
// curvature-weighted, ...) can be dropped in mid-flight.  An estimation can
// optionally be routed through an audit backend (see createAuditBackend).

const {
  vsub, vlen, angleBetween, ratVal,
} = require('../math');
const { hullProbePoints, maxParamEdge, triangleParamPoints, sampledErrors } = require('./sampling');

function guaranteedPositionBound(surface, tri, M) {
  const h = maxParamEdge(tri);
  return 0.5 * M * h * h;
}

// Normal-angle bound from a derivative-Lipschitz argument.  See the original
// derivation: |J(q)-J(a)| <= M h, so sin(angle) <= M h (|e1|+|e2|)/|e1 x e2|;
// with an analytic curvature kappa a tighter cone kappa * arc-length applies.
function guaranteedNormalBound(surface, tri, M) {
  const p3 = tri.v.map((v) => v.p);
  let Lmax = 0;
  let Lmin = Infinity;
  for (let i = 0; i < 3; i++) {
    const L = vlen(vsub(p3[i], p3[(i + 1) % 3]));
    Lmax = Math.max(Lmax, L);
    Lmin = Math.min(Lmin, L);
  }
  if (typeof surface.curvatureBound === 'function') {
    const pts = hullProbePoints(tri);
    const kappa = surface.curvatureBound(pts);
    if (Number.isFinite(kappa) && kappa >= 0 && Lmax > 1e-12) {
      const h = maxParamEdge(tri);
      let jMax = 0;
      for (const [u, v] of pts) {
        jMax = Math.max(jMax, vlen(surface.derivU(u, v)), vlen(surface.derivV(u, v)));
      }
      const sMax = Math.max(Lmax, jMax * h);
      const curvatureCone = 1.5 * kappa * sMax;
      const corners = triangleParamPoints(tri);
      let vertexCone = 0;
      const nrm = corners.map(([u, v]) => surface.normal(u, v));
      for (let a = 0; a < 3; a++) {
        for (let b = a + 1; b < 3; b++) {
          if (vlen(nrm[a]) > 0 && vlen(nrm[b]) > 0) {
            vertexCone = Math.max(vertexCone, angleBetween(nrm[a], nrm[b]));
          }
        }
      }
      return Math.min(Math.PI / 2, Math.max(curvatureCone, vertexCone));
    }
  }
  const h = maxParamEdge(tri);
  const delta = 0.5 * M * h * h;
  if (delta <= 0) return 0;
  if (!(Lmin > 1e-12)) return Math.PI / 2;
  const ratio = (2 * delta) / Lmin;
  if (ratio >= 1) return Math.PI / 2;
  return Math.asin(ratio);
}

// ---------------------------------------------------------------------------
// Metric: analytic + sampled (the historical default).
// ---------------------------------------------------------------------------

function evaluateTriangle(surface, tri, depth, maxDepth, effTol) {
  const tol = effTol || surface.tol;
  const h = maxParamEdge(tri);
  const pts = hullProbePoints(tri);
  const M = surface.hessianBound(pts);
  const posBound = guaranteedPositionBound(surface, tri, M);
  const nrmBound = guaranteedNormalBound(surface, tri, M);
  const sampled = sampledErrors(surface, tri);
  const isGuaranteed = surface.boundIsGuaranteed;

  const reasons = [];
  let needsRefine = false;

  if (tol.position > 0) {
    if (isGuaranteed) {
      if (posBound > tol.position) {
        needsRefine = true;
        reasons.push({
          code: 'pos-bound',
          detail: `guaranteed chord bound ${posBound.toExponential(2)} > tol ${tol.position.toExponential(2)}`,
        });
      }
    } else {
      if (sampled.maxDist > tol.position || posBound > tol.position) {
        needsRefine = true;
        reasons.push({
          code: 'pos-estimate',
          detail: `sampled ${sampled.maxDist.toExponential(2)} / est-bound ${posBound.toExponential(2)} vs tol ${tol.position.toExponential(2)} (no guarantee)`,
        });
      }
    }
  }

  if (tol.normal > 0) {
    if (isGuaranteed) {
      if (nrmBound > tol.normal) {
        needsRefine = true;
        reasons.push({
          code: 'normal-bound',
          detail: `guaranteed normal angle ${(nrmBound * 180 / Math.PI).toFixed(2)}deg > tol ${(tol.normal * 180 / Math.PI).toFixed(2)}deg`,
        });
      }
    } else if (sampled.maxAngle > tol.normal || nrmBound > tol.normal) {
      needsRefine = true;
      reasons.push({
        code: 'normal-estimate',
        detail: `sampled ${(sampled.maxAngle * 180 / Math.PI).toFixed(2)}deg / est ${(nrmBound * 180 / Math.PI).toFixed(2)}deg vs tol ${(tol.normal * 180 / Math.PI).toFixed(2)}deg`,
      });
    }
  }

  applySingularRule(surface, tri, depth, maxDepth, reasons, () => { needsRefine = true; });

  if (depth >= maxDepth && needsRefine) {
    reasons.push({ code: 'depth-cap', detail: `refinement stopped at depth cap ${maxDepth}` });
  }

  return {
    needsRefine: needsRefine && depth < maxDepth,
    capped: needsRefine && depth >= maxDepth,
    reasons,
    metrics: {
      h,
      posBound,
      nrmBound,
      sampledDist: sampled.maxDist,
      sampledAngle: sampled.maxAngle,
      probes: sampled.probes,
      posGuaranteed: isGuaranteed,
      nrmGuaranteed: isGuaranteed,
      depth,
    },
  };
}

// ---------------------------------------------------------------------------
// Metric: sampled-only.  A genuinely different error measure: it ignores the
// analytic guarantee and refines from dense sampling alone, reporting its
// numbers as estimates.  Used to prove that a metric swap does not disturb
// shared-edge identity.
// ---------------------------------------------------------------------------

function evaluateSampledOnly(surface, tri, depth, maxDepth, effTol) {
  const tol = effTol || surface.tol;
  const h = maxParamEdge(tri);
  const sampled = sampledErrors(surface, tri);
  const reasons = [];
  let needsRefine = false;
  const isGuaranteed = false;

  if (tol.position > 0 && sampled.maxDist > tol.position) {
    needsRefine = true;
    reasons.push({
      code: 'pos-sampled',
      detail: `sampled ${sampled.maxDist.toExponential(2)} > tol ${tol.position.toExponential(2)}`,
    });
  }
  if (tol.normal > 0 && sampled.maxAngle > tol.normal) {
    needsRefine = true;
    reasons.push({
      code: 'normal-sampled',
      detail: `sampled ${(sampled.maxAngle * 180 / Math.PI).toFixed(2)}deg > tol ${(tol.normal * 180 / Math.PI).toFixed(2)}deg`,
    });
  }

  applySingularRule(surface, tri, depth, maxDepth, reasons, () => { needsRefine = true; });

  if (depth >= maxDepth && needsRefine) {
    reasons.push({ code: 'depth-cap', detail: `refinement stopped at depth cap ${maxDepth}` });
  }

  return {
    needsRefine: needsRefine && depth < maxDepth,
    capped: needsRefine && depth >= maxDepth,
    reasons,
    metrics: {
      h,
      posBound: sampled.maxDist,
      nrmBound: sampled.maxAngle,
      sampledDist: sampled.maxDist,
      sampledAngle: sampled.maxAngle,
      probes: sampled.probes,
      posGuaranteed: isGuaranteed,
      nrmGuaranteed: isGuaranteed,
      depth,
    },
  };
}

// Repeated / singular parameter nodes: a fan-seeded pole is already one shared
// vertex (collapsedTo / canonical), so only force refinement when a triangle
// still carries an unresolved collapse.
function applySingularRule(surface, tri, depth, maxDepth, reasons, markRefine) {
  for (const [su, sv] of surface.singularNodes) {
    for (const vx of tri.v) {
      if (vx.collapsedTo !== undefined || vx.canonical) continue;
      const u = ratVal(vx.uv[0]);
      const vv = ratVal(vx.uv[1]);
      if (Math.hypot(u - su, vv - sv) < 1e-12 && depth < maxDepth) {
        markRefine();
        reasons.push({
          code: 'singular-node',
          detail: `triangle touches repeated node (${su},${sv}); forced depth refinement`,
        });
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Estimator objects.
// ---------------------------------------------------------------------------

function adaptEvaluator(name, kind, fn) {
  return {
    name,
    kind,
    evaluate(ctx) {
      return fn(ctx.surface, ctx.tri, ctx.depth, ctx.maxDepth, ctx.tol);
    },
  };
}

const defaultEstimator = adaptEvaluator(
  'analytic-sampled', 'guaranteed+sampled', evaluateTriangle,
);

const estimators = {
  'analytic-sampled': defaultEstimator,
  'sampled-only': adaptEvaluator('sampled-only', 'sampled', evaluateSampledOnly),
};

function resolveEstimator(spec) {
  if (!spec) return defaultEstimator;
  if (typeof spec === 'string') {
    const est = estimators[spec];
    if (!est) throw new Error(`unknown error metric ${spec}`);
    return est;
  }
  if (typeof spec.evaluate === 'function' && typeof spec.name === 'string') return spec;
  throw new Error('invalid error metric: expected name or {name, evaluate}');
}

// ---------------------------------------------------------------------------
// Replaceable audit backend.
//
// The error module never writes audit data directly; it hands events to this
// backend.  Swapping the backend (e.g. in-memory -> structured log) must not
// change estimation results.
// ---------------------------------------------------------------------------

function createAuditBackend(opts = {}) {
  const cap = opts.cap || 5000;
  const records = [];
  return {
    name: opts.name || 'memory-audit',
    record(event) {
      if (records.length >= cap) records.shift();
      records.push({ ...event, at: Date.now() });
      return true;
    },
    records,
    clear() { records.length = 0; },
    snapshot() { return records.slice(); },
  };
}

module.exports = {
  evaluateTriangle,
  evaluateSampledOnly,
  guaranteedPositionBound,
  guaranteedNormalBound,
  sampledErrors,
  maxParamEdge,
  defaultEstimator,
  estimators,
  resolveEstimator,
  createAuditBackend,
};
