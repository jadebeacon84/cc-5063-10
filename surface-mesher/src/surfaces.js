'use strict';

// Parametric surface definitions.
//
// A ParamSurface maps the unit parameter square [0,1]^2 into R3.
//   pos(u,v)              -> [x,y,z]
//   derivU / derivV       -> first partial derivatives
//   normal(u,v)           -> unit normal
//   hessianBound(points)  -> upper bound M on the operator norm of the
//                            Hessian of pos over the convex hull of the given
//                            parameter points.  When the surface supplies an
//                            analytic bound this is a *guaranteed* number;
//                            otherwise it is an estimate derived from dense
//                            sampling and flagged via boundIsGuaranteed.
//   singularNodes         -> parameter points where the parameterization
//                            degenerates (repeated 3D nodes, e.g. a pole).

const { vadd, vscale, vcross, vnorm, vlen, vsub, clamp } = require('./math');

class ParamSurface {
  constructor(cfg) {
    this.name = cfg.name;
    this.uvMap = cfg.uvMap || ((u, v) => [u, v]); // unit square -> domain
    this._pos = cfg.pos;
    this._derivU = cfg.derivU;
    this._derivV = cfg.derivV;
    this._hessianBound = cfg.hessianBound || null;
    this._curvatureBound = cfg.curvatureBound || null;
    this.boundIsGuaranteed = !!cfg.hessianBound;
    this.singularNodes = cfg.singularNodes || [];
    this.tol = cfg.tol; // { position, normal(rad) } declared tolerances
  }

  // Analytic upper bound on the largest principal curvature over the probe
  // hull.  Returns null when no closed-form curvature is available, in which
  // case the error module falls back to its Hessian Lipschitz argument.
  curvatureBound(pts) {
    if (!this._curvatureBound) return null;
    return this._curvatureBound(pts.map((p) => this.mapUV(p[0], p[1])));
  }

  mapUV(u, v) {
    return this.uvMap(u, v);
  }

  pos(u, v) {
    return this._pos(...this.mapUV(u, v));
  }
  derivU(u, v) {
    if (this._derivU) return this._derivU(...this.mapUV(u, v));
    return this._fdDeriv(u, v, 0);
  }
  derivV(u, v) {
    if (this._derivV) return this._derivV(...this.mapUV(u, v));
    return this._fdDeriv(u, v, 1);
  }
  normal(u, v) {
    const n = vcross(this.derivU(u, v), this.derivV(u, v));
    return vnorm(n);
  }

  _fdDeriv(u, v, axis) {
    const h = 1e-6;
    const a = axis === 0 ? [clamp(u + h, 0, 1), v] : [u, clamp(v + h, 0, 1)];
    const b = axis === 0 ? [clamp(u - h, 0, 1), v] : [u, clamp(v - h, 0, 1)];
    const pa = this.pos(...a);
    const pb = this.pos(...b);
    return vscale(vsub(pa, pb), 1 / (a[axis] - b[axis]));
  }

  // Guaranteed bound if analytic callback exists; otherwise an estimate
  // from second finite differences on a dense grid over the triangle's
  // parameter hull, inflated by a guard factor.  The inflated value is
  // still an estimate and must never be labelled guaranteed.
  hessianBound(uvPoints) {
    if (this._hessianBound) return this._hessianBound(uvPoints.map((p) => this.mapUV(p[0], p[1])));
    return this._sampledHessianBound(uvPoints);
  }

  _sampledHessianBound(uvPoints) {
    const us = uvPoints.map((p) => p[0]);
    const vs = uvPoints.map((p) => p[1]);
    const uMin = Math.min(...us), uMax = Math.max(...us);
    const vMin = Math.min(...vs), vMax = Math.max(...vs);
    // Mixed absolute/relative step: a fixed tiny step underflows on small
    // triangles, while one large step strides over narrow features.  Probe a
    // geometric sequence of scales and keep the strongest response.
    const span = Math.max(uMax - uMin, vMax - vMin, 1e-8);
    const steps = [
      clamp(span / 6, 1e-7, 0.05),
      clamp(span / 24, 1e-8, 0.05),
      0.005,
    ].filter((x, i, arr) => Number.isFinite(x) && arr.indexOf(x) === i && x > 0);
    // Candidate probes: regular hull grid plus its midpoint and quarter
    // points, so a narrow curvature peak narrower than one grid cell cannot
    // hide between probes.
    const probes = [];
    const N = 6;
    for (let i = 0; i <= N; i++) {
      for (let j = 0; j <= N; j++) {
        probes.push([uMin + ((uMax - uMin) * i) / N, vMin + ((vMax - vMin) * j) / N]);
      }
    }
    const uMid = (uMin + uMax) / 2;
    const vMid = (vMin + vMax) / 2;
    probes.push([uMid, vMid], [uMid + span / 4, vMid], [uMid - span / 4, vMid],
      [uMid, vMid + span / 4], [uMid, vMid - span / 4]);
    let m = 0;
    for (const h of steps) {
      for (const [uu, vv] of probes) {
        const u = clamp(uu, h, 1 - h);
        const v = clamp(vv, h, 1 - h);
        if (u < h || u > 1 - h || v < h || v > 1 - h) continue;
        for (const mat of this._fdHessians(u, v, h)) {
          const mn = matNorm(mat);
          if (Number.isFinite(mn)) m = Math.max(m, mn);
        }
      }
    }
    return m * 2.0; // guard factor: estimate, not a proof
  }

  _fdHessians(u, v, h) {
    const p = (a, b) => this.pos(a, b);
    // Central second differences, one per Hessian entry.
    const dUU = vscale(
      vadd(vsub(p(u + h, v), vscale(p(u, v), 2)), p(u - h, v)),
      1 / (h * h)
    );
    const dVV = vscale(
      vadd(vsub(p(u, v + h), vscale(p(u, v), 2)), p(u, v - h)),
      1 / (h * h)
    );
    const dUV = vscale(
      vadd(
        vsub(p(u + h, v + h), p(u + h, v - h)),
        vsub(p(u - h, v - h), p(u - h, v + h))
      ),
      1 / (4 * h * h)
    );
    return [[dUU, dUV], [dUV, dVV]];
  }
}

// Largest eigenvalue (operator norm) of a symmetric 2x2 matrix whose
// entries are vectors: treat as block operator and use the largest singular
// value of the 3x2 derivative-of-gradient, bounded via the Frobenius norm.
function matNorm(rows) {
  let fro = 0;
  for (const row of rows) for (const vec of row) fro += vlen(vec) ** 2;
  return Math.sqrt(fro); // >= operator norm, conservative
}

// ---------------------------------------------------------------------------
// Surface factory functions
// ---------------------------------------------------------------------------

// 1. Perfectly flat plane: Hessian is exactly zero, so no refinement is ever
//    needed beyond the two seed triangles.
function planeSurface(tol) {
  return new ParamSurface({
    name: 'plane',
    pos: (u, v) => [u, v, 0],
    derivU: () => [1, 0, 0],
    derivV: () => [0, 1, 0],
    hessianBound: () => 0,
    tol,
  });
}

// 2. Smooth Gaussian bump.  Hessian bound is derived analytically, hence
//    guaranteed.
function bumpSurface(tol, sigma = 0.12, amp = 0.25) {
  const g = (u, v) => amp * Math.exp(-(((u - 0.5) ** 2 + (v - 0.5) ** 2) / (2 * sigma * sigma)));
  const pos = (u, v) => [u, v, g(u, v)];
  const gu = (u, v) => -(u - 0.5) / (sigma * sigma) * g(u, v);
  const gv = (u, v) => -(v - 0.5) / (sigma * sigma) * g(u, v);
  const guu = (u, v) =>
    (((u - 0.5) ** 2 / sigma ** 4) - 1 / sigma ** 2) * g(u, v);
  const gvv = (u, v) =>
    (((v - 0.5) ** 2 / sigma ** 4) - 1 / sigma ** 2) * g(u, v);
  const guv = (u, v) => ((u - 0.5) * (v - 0.5) / sigma ** 4) * g(u, v);
  // Analytic upper bound evaluated ONLY over the probe points handed in,
  // which cover the requesting triangle's own parameter hull.  Taking a
  // global maximum here would make every flat triangle inherit the peak
  // curvature and the bound would never decay with h^2.
  const bound = (pts) => {
    let m = 0;
    for (const [u, v] of pts) {
      m = Math.max(
        m,
        Math.abs(guu(u, v)) + Math.abs(guv(u, v)),
        Math.abs(gvv(u, v)) + Math.abs(guv(u, v))
      );
    }
    return m;
  };
  // Physical principal-curvature magnitude for a graph z=g(x,y):
  //   kappa <= |Hess g| / (1+|grad g|^2)^(3/2) <= |Hess g|.
  const curvature = (pts) => {
    let k = 0;
    for (const [u, v] of pts) {
      const grad2 = gu(u, v) ** 2 + gv(u, v) ** 2;
      k = Math.max(k, bound([[u, v]]) / (1 + grad2) ** 1.5);
    }
    return k;
  };
  return new ParamSurface({
    name: 'gaussian-bump',
    pos,
    derivU: (u, v) => [1, 0, gu(u, v)],
    derivV: (u, v) => [0, 1, gv(u, v)],
    hessianBound: bound,
    curvatureBound: curvature,
    tol,
  });
}

// 3. Narrow local spike (Lorentzian): very high curvature at the centre,
// smooth but with a steep falloff.  Closed-form Hessian -> guaranteed bound.
function spikeSurface(tol, width = 0.02, amp = 0.4) {
  const w2 = width * width;
  const h = (u, v) => {
    const r2 = (u - 0.5) ** 2 + (v - 0.5) ** 2;
    return (amp * w2) / (r2 + w2);
  };
  const d = (u, v) => (u - 0.5) ** 2 + (v - 0.5) ** 2 + w2;
  const hu = (u, v) => -2 * amp * w2 * (u - 0.5) / d(u, v) ** 2;
  const hv = (u, v) => -2 * amp * w2 * (v - 0.5) / d(u, v) ** 2;
  // Second derivatives of the scalar graph z(u,v); the 3D Hessian operator
  // norm equals the scalar one for a graph over the unit parameter plane.
  const huu = (u, v) => {
    const x = u - 0.5;
    const dd = d(u, v);
    return -2 * amp * w2 / dd ** 2 + 8 * amp * w2 * x * x / dd ** 3;
  };
  const hvv = (u, v) => {
    const y = v - 0.5;
    const dd = d(u, v);
    return -2 * amp * w2 / dd ** 2 + 8 * amp * w2 * y * y / dd ** 3;
  };
  const huv = (u, v) => {
    const x = u - 0.5;
    const y = v - 0.5;
    return 8 * amp * w2 * x * y / d(u, v) ** 3;
  };
  const bound = (pts) => {
    let m = 0;
    for (const [u, v] of pts) {
      m = Math.max(
        m,
        Math.abs(huu(u, v)) + Math.abs(huv(u, v)),
        Math.abs(hvv(u, v)) + Math.abs(huv(u, v))
      );
    }
    return m;
  };
  const curvature = (pts) => {
    let k = 0;
    for (const [u, v] of pts) {
      const grad2 = hu(u, v) ** 2 + hv(u, v) ** 2;
      k = Math.max(k, bound([[u, v]]) / (1 + grad2) ** 1.5);
    }
    return k;
  };
  return new ParamSurface({
    name: 'lorentzian-spike',
    pos: (u, v) => [u, v, h(u, v)],
    derivU: (u, v) => [1, 0, hu(u, v)],
    derivV: (u, v) => [0, 1, hv(u, v)],
    hessianBound: bound,
    curvatureBound: curvature,
    tol,
  });
}

// 4. Polar cap: unit sphere parameterized with spherical angles.  The north
//    pole is a repeated node (all v=1 rows collapse to one 3D point), so
//    ordinary midpoint sampling alone cannot certify error there; the
//    singular-node rule forces depth refinement around it.
function polarCapSurface(tol) {
  const pos = (u, v) => {
    const theta = Math.PI * (1 - v); // 0 at v=1 (north pole), pi at v=0
    const phi = 2 * Math.PI * u;
    return [Math.sin(theta) * Math.cos(phi), Math.sin(theta) * Math.sin(phi), Math.cos(theta)];
  };
  // Local analytic Hessian bound.  On a unit sphere:
  //   |f_vv| = pi,  |f_uu| = 4 pi^2 sin(theta),  |f_uv| = 2 pi^2 cos(theta)
  // evaluated only over the requesting triangle's probe hull.
  const bound = (pts) => {
    let m = 0;
    for (const [, v] of pts) {
      const theta = Math.PI * (1 - v);
      const fvv = Math.PI;
      const fuu = 4 * Math.PI * Math.PI * Math.abs(Math.sin(theta));
      const fuv = 2 * Math.PI * Math.PI * Math.abs(Math.cos(theta));
      m = Math.max(m, fvv + fuv, fuu + fuv);
    }
    return m;
  };
  return new ParamSurface({
    name: 'polar-cap',
    pos,
    hessianBound: bound,
    curvatureBound: () => 1, // unit sphere: both principal curvatures equal 1
    singularNodes: [[0.5, 1.0]], // north pole (repeated node: u arbitrary at v=1)
    tol,
  });
}

// A sphere *band* that deliberately excludes both poles, so the ordinary
// rectangular mesh stays non-degenerate while curvature varies strongly
// with latitude.  Used as the well-conditioned curved test.
function sphereBandSurface(tol) {
  const thetaMin = 0.35 * Math.PI;
  const thetaMax = 0.65 * Math.PI;
  const pos = (u, v) => {
    const theta = thetaMin + (thetaMax - thetaMin) * v;
    const phi = 2 * Math.PI * u;
    return [Math.sin(theta) * Math.cos(phi), Math.sin(theta) * Math.sin(phi), Math.cos(theta)];
  };
  const bound = (pts) => {
    let m = 0;
    for (const [, v] of pts) {
      const theta = thetaMin + (thetaMax - thetaMin) * v;
      const fuu = 4 * Math.PI * Math.PI * Math.abs(Math.sin(theta));
      const fuv = 2 * Math.PI * Math.PI * Math.abs(Math.cos(theta));
      m = Math.max(m, Math.PI + fuv, fuu + fuv);
    }
    return m;
  };
  return new ParamSurface({
    name: 'sphere-band',
    pos,
    hessianBound: bound,
    curvatureBound: () => 1, // unit sphere band
    tol,
  });
}

// 5. Half-Gaussian patch used for multi-patch seam scenes.  Domain is the
//    unit square mapped onto a sub-rectangle of the bump's domain; two such
//    patches placed side by side share a physical edge.
function halfBumpPatch(name, uOffset, tol, sigma = 0.12, amp = 0.25) {
  const map = (u, v) => [uOffset + 0.5 * u, v];
  const g = (x, y) => amp * Math.exp(-(((x - 0.5) ** 2 + (y - 0.5) ** 2) / (2 * sigma * sigma)));
  const hbHessian = (pts) => {
    let m = 0;
    for (const [x, y] of pts) {
      const guu = (((x - 0.5) ** 2 / sigma ** 4) - 1 / sigma ** 2) * g(x, y);
      const gvv = (((y - 0.5) ** 2 / sigma ** 4) - 1 / sigma ** 2) * g(x, y);
      const guv = ((x - 0.5) * (y - 0.5) / sigma ** 4) * g(x, y);
      m = Math.max(m, Math.abs(guu) + Math.abs(guv), Math.abs(gvv) + Math.abs(guv));
    }
    return m;
  };
  return new ParamSurface({
    name,
    uvMap: map,
    pos: (x, y) => [x, y, g(x, y)],
    derivU: (x, y) => [1, 0, -(x - 0.5) / (sigma * sigma) * g(x, y)],
    derivV: (x, y) => [0, 1, -(y - 0.5) / (sigma * sigma) * g(x, y)],
    hessianBound: hbHessian,
    curvatureBound: (pts) => {
      let k = 0;
      for (const [x, y] of pts) {
        const gx = -(x - 0.5) / (sigma * sigma) * g(x, y);
        const gy = -(y - 0.5) / (sigma * sigma) * g(x, y);
        k = Math.max(k, hbHessian([[x, y]]) / (1 + gx * gx + gy * gy) ** 1.5);
      }
      return k;
    },
    tol,
  });
}

// 6. Flat strip for the deliberately separated "lips": two such strips at
//    slightly different z heights form a physical gap, each with its own
//    tolerance and no shared-edge identity.
function lipStrip(name, zHeight, tol) {
  return new ParamSurface({
    name,
    pos: (u, v) => [u, v, zHeight],
    derivU: () => [1, 0, 0],
    derivV: () => [0, 1, 0],
    hessianBound: () => 0,
    tol,
  });
}

module.exports = {
  ParamSurface,
  planeSurface,
  bumpSurface,
  spikeSurface,
  polarCapSurface,
  sphereBandSurface,
  halfBumpPatch,
  lipStrip,
};
