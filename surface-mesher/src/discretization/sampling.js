'use strict';

// Surface sampling layer.
//
// Everything that measures the discretisation against the underlying surface
// lives here: parameter-space probe layouts, element size, and the dense
// size-adapted supersampling used to *measure* (never to prove) error.
//
// This module is deliberately geometry-only.  It knows nothing about
// refinement policy, shared edges, or budgets, so it can be reused by any
// error metric that wants to sample a cell.

const {
  vsub, vcross, vlen, vnorm, angleBetween,
  ratVal, clamp, pointTriangleDistance,
} = require('../math');

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

module.exports = {
  triangleParamPoints,
  hullProbePoints,
  maxParamEdge,
  sampleCount,
  sampledErrors,
};
