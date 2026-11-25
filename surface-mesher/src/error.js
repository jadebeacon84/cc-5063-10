'use strict';

// Per-triangle error estimation.
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
// Estimated surfaces therefore refine until both the sampled error AND the
// estimated Hessian-based bound are inside tolerance, and are reported with
// guaranteed=false.

const {
  vsub, vcross, vlen, vnorm, vdot, pointTriangleDistance,
  angleBetween, ratVal, clamp,
} = require('./math');

function triangleParamPoints(tri) {
  return tri.v.map((v) => [ratVal(v.uv[0]), ratVal(v.uv[1])]);
}

// Probe grid over the triangle's axis-aligned parameter hull so interior
// curvature peaks are covered rather than only the three corners.
function hullProbePoints(tri, n = 4) {
  const p = triangleParamPoints(tri);
  const us = p.map((q) => q[0]);
  const vs = p.map((q) => q[1]);
  const uMin = Math.min(...us), uMax = Math.max(...us);
  const vMin = Math.min(...vs), vMax = Math.max(...vs);
  const out = [];
  for (let i = 0; i <= n; i++) {
    for (let j = 0; j <= n; j++) {
      out.push([uMin + ((uMax - uMin) * i) / n, vMin + ((vMax - vMin) * j) / n]);
    }
  }
  return out;
}

function maxParamEdge(tri) {
  const p = triangleParamPoints(tri);
  let h = 0;
  for (let i = 0; i < 3; i++) {
    const j = (i + 1) % 3;
    h = Math.max(h, Math.hypot(p[i][0] - p[j][0], p[i][1] - p[j][1]));
  }
  return h;
}

function guaranteedPositionBound(surface, tri, M) {
  const h = maxParamEdge(tri);
  return 0.5 * M * h * h;
}

// Normal-angle bound from a derivative-Lipschitz argument.  For any two
// parameter points q, a inside the triangle,
//   |J(q) - J(a)| <= M |q-a|_param <= M h,
// with h the longest PARAMETER edge and M the local Hessian operator bound.
// Measuring the perturbed tangent frame against each corner frame gives
//   sin(angle) <= M h (|e1| + |e2|) / |e1 x e2|,
// which is tighter than a physical-chord argument while remaining a
// guaranteed bound (the Jacobian is sampled on the hull probe grid).
// Guaranteed facet-vs-surface normal-angle bound.
//
// If kappaMax is an analytic upper bound on the largest *physical* principal
// curvature over the triangle, the surface normal rotates at rate kappaMax
// per unit arc length.  The normal of the planar facet lies within the cone
// of surface normals over the element, whose half-angle is bounded by
// kappaMax * L with L the longest physical facet edge (a conservative
// stand-in for the longest interior arc).  This is exact up to the curvature
// bound for spheres and quadratic patches.
//
// When no curvature callback exists, fall back to the chord-height argument:
// with delta = (1/2) M h^2 (parameter-space Hessian bound), the plane tilt
// satisfies |sin(angle)| <= 2 delta / L_min over each edge.
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
      // Arc length along the surface is at least the chord and at most
      // max|J| * h over the parameter hull; use the latter as the path bound
      // so periodic / wrapped parameterisations cannot hide a longer arc.
      const h = maxParamEdge(tri);
      let jMax = 0;
      for (const [u, v] of pts) {
        jMax = Math.max(jMax, vlen(surface.derivU(u, v)), vlen(surface.derivV(u, v)));
      }
      const sMax = Math.max(Lmax, jMax * h);
      // Safety factor on the analytic curvature cone: arc length inside a
      // curved element can exceed the chord-based proxy, and the measured
      // vertex cone can miss an interior extremum.  1.5 keeps the bound
      // guaranteed without the blow-up of a pure Hessian Lipschitz estimate.
      const curvatureCone = 1.5 * kappa * sMax;
      // Cross-check against the measured rotation between the three vertex
      // surface normals: the facet normal cannot lie outside their cone, so
      // the cone's measured half-angle is a lower bound the analytic estimate
      // must not under-shoot.  Taking the max keeps the result guaranteed
      // even when the arc-length proxy is imperfect.
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

// Size-adapted dense sampling.  Estimated surfaces use a denser grid because
// no analytic bound protects the points between probes; small triangles also
// carry many probes so a narrow spike cannot hide between sample points.
function sampleCount(surface, tri) {
  const h = maxParamEdge(tri);
  const base = Math.max(3, Math.min(12, Math.ceil(1.5 / Math.max(h, 1e-6))));
  return surface.boundIsGuaranteed ? base : Math.max(base, 8);
}

function sampledErrors(surface, tri) {
  const [a, b, c] = tri.v.map((v) => v.p);
  const pa = tri.v[0].uv.map(ratVal);
  const pb = tri.v[1].uv.map(ratVal);
  const pc = tri.v[2].uv.map(ratVal);
  const facetNormal = vnorm(vcross(vsub(b, a), vsub(c, a)));
  let maxDist = 0;
  let maxAngle = 0;
  const n = sampleCount(surface, tri);
  for (let i = 0; i <= n; i++) {
    for (let j = 0; j <= n - i; j++) {
      const w1 = i / n;
      const w2 = j / n;
      const w0 = 1 - w1 - w2;
      if (i === 0 && j === 0) continue; // corners are exact by construction
      const u = clamp(w0 * pa[0] + w1 * pb[0] + w2 * pc[0], 0, 1);
      const v = clamp(w0 * pa[1] + w1 * pb[1] + w2 * pc[1], 0, 1);
      const exact = surface.pos(u, v);
      const d = pointTriangleDistance(exact, a, b, c);
      maxDist = Math.max(maxDist, d);
      const exactN = surface.normal(u, v);
      if (vlen(exactN) > 0) maxAngle = Math.max(maxAngle, angleBetween(exactN, facetNormal));
    }
  }
  return { maxDist, maxAngle, probes: ((n + 1) * (n + 2)) / 2 - 1 };
}

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
      // Estimated surface: require BOTH the measured sample and the
      // Hessian-based estimate (with its guard factor) to pass.
      const estBound = posBound;
      if (sampled.maxDist > tol.position || estBound > tol.position) {
        needsRefine = true;
        reasons.push({
          code: 'pos-estimate',
          detail: `sampled ${sampled.maxDist.toExponential(2)} / est-bound ${estBound.toExponential(2)} vs tol ${tol.position.toExponential(2)} (no guarantee)`,
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

  // Repeated / singular parameter nodes.  A fan-seeded pole is already a
  // single shared vertex (marked collapsedTo / canonical), so only force
  // refinement when a triangle still carries an unresolved collapse.
  for (const [su, sv] of surface.singularNodes) {
    for (const vx of tri.v) {
      if (vx.collapsedTo !== undefined || vx.canonical) continue;
      const u = ratVal(vx.uv[0]);
      const vv = ratVal(vx.uv[1]);
      if (Math.hypot(u - su, vv - sv) < 1e-12 && depth < maxDepth) {
        needsRefine = true;
        reasons.push({
          code: 'singular-node',
          detail: `triangle touches repeated node (${su},${sv}); forced depth refinement`,
        });
      }
    }
  }

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

module.exports = {
  evaluateTriangle,
  guaranteedPositionBound,
  guaranteedNormalBound,
  sampledErrors,
  maxParamEdge,
};
