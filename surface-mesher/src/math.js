'use strict';

// Minimal 3D vector + rational helpers.  Vectors are plain [x, y, z] arrays.

const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vcross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const vlen = (a) => Math.hypot(a[0], a[1], a[2]);
const vdist = (a, b) => vlen(vsub(a, b));
const vnorm = (a) => {
  const l = vlen(a);
  return l > 1e-300 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0];
};
const vlerp = (a, b, t) => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
];
const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

function angleBetween(a, b) {
  const na = vnorm(a);
  const nb = vnorm(b);
  if (vlen(na) === 0 || vlen(nb) === 0) return Math.PI / 2;
  return Math.acos(clamp(vdot(na, nb), -1, 1));
}

// Exact dyadic rationals so that vertex identity never depends on float
// rounding: every point reached by repeated midpoint bisection is dyadic.
// Stored as {n, d} with d a power of two and gcd reduced.
function rat(n, d) {
  n = Math.round(n);
  d = Math.round(d);
  while (d > 1 && n % 2 === 0) {
    n /= 2;
    d /= 2;
  }
  return { n, d };
}
const ratAdd = (r, s) => rat(r.n * s.d + s.n * r.d, r.d * s.d);
const ratMid = (r, s) => rat(r.n * s.d + s.n * r.d, 2 * r.d * s.d);
const ratVal = (r) => r.n / r.d;
const ratKey = (r) => `${r.n}/${r.d}`;
const ratLt = (r, s) => r.n * s.d < s.n * r.d;

// Point-to-triangle distance (Ericson's closest-point routine).
function pointTriangleDistance(p, a, b, c) {
  const ab = vsub(b, a);
  const ac = vsub(c, a);
  const ap = vsub(p, a);
  const d1 = vdot(ab, ap);
  const d2 = vdot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return vdist(p, a);
  const bp = vsub(p, b);
  const d3 = vdot(ab, bp);
  const d4 = vdot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return vdist(p, b);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const t = d1 / (d1 - d3);
    return vdist(p, vlerp(a, b, t));
  }
  const cp = vsub(p, c);
  const d5 = vdot(ab, cp);
  const d6 = vdot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return vdist(p, c);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const t = d2 / (d2 - d6);
    return vdist(p, vlerp(a, c, t));
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    const t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return vdist(p, vlerp(b, c, t));
  }
  const denom = 1 / (va + vb + vc);
  const v = vb * denom;
  const w = vc * denom;
  const proj = vadd(a, vadd(vscale(ab, v), vscale(ac, w)));
  return vdist(p, proj);
}

module.exports = {
  vsub, vadd, vscale, vdot, vcross, vlen, vdist, vnorm, vlerp, clamp,
  angleBetween, rat, ratAdd, ratMid, ratVal, ratKey, ratLt,
  pointTriangleDistance,
};
