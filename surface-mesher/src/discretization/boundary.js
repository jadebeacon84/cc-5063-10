'use strict';

// Cross-patch shared-edge coordination.
//
// A weld ties two patch sides that must agree on the boundary sampling so the
// exported mesh has no crack.  This module owns *how* that agreement is
// reached.  Two interchangeable strategies are provided so their behaviour can
// be compared at the same seams:
//
//   unified-cached   - one canonical 3D position per dyadic arc parameter is
//                      cached on the weld and reused by both sides.  The split
//                      decision is unified: a single cached sample identifies
//                      the vertex, so both sides are bit-identical by
//                      construction.
//
//   negotiated-depth - each side samples its own surface for boundary
//                      midpoints; the two sides negotiate the shared edge's
//                      refinement depth (the maximum either side demands) and
//                      both subdivide to it.  Vertices coincide up to
//                      floating error and are reconciled at export, so the
//                      comparison exposes any real gap the cache would mask.
//
// Both strategies receive the refinement driver (`ctx`) at call time instead
// of requiring it, keeping this module free of a require-cycle.  The driver
// supplies the low-level mechanics: budgetRemaining(), forceBoundaryLevel(),
// boundaryMinLevel(), patchByName, events.

const { rat, ratVal, ratKey, ratMid } = require('../math');
const { uvKey } = require('./keys');

const SIDES = ['left', 'right', 'bottom', 'top'];

// ---------------------------------------------------------------------------
// Parameter geometry helpers (shared by the driver and the strategies).
// ---------------------------------------------------------------------------

function sideUV(side) {
  switch (side) {
    case 'left': return [[rat(0, 1), rat(0, 1)], [rat(0, 1), rat(1, 1)]];
    case 'right': return [[rat(1, 1), rat(0, 1)], [rat(1, 1), rat(1, 1)]];
    case 'bottom': return [[rat(0, 1), rat(0, 1)], [rat(1, 1), rat(0, 1)]];
    case 'top': return [[rat(0, 1), rat(1, 1)], [rat(1, 1), rat(1, 1)]];
    default: throw new Error(`unknown side ${side}`);
  }
}

function onSideLine(side, v) {
  const u = ratVal(v.uv[0]);
  const w = ratVal(v.uv[1]);
  if (side === 'left') return u === 0;
  if (side === 'right') return u === 1;
  if (side === 'bottom') return w === 0;
  if (side === 'top') return w === 1;
  return false;
}

// Arc parameter t (dyadic) along a side for a vertex on that side.
function sideTRat(side, uv) {
  switch (side) {
    case 'left':
    case 'right':
      return uv[1];
    case 'bottom':
    case 'top':
      return uv[0];
    default: throw new Error(side);
  }
}

function uvAtT(side, t) {
  switch (side) {
    case 'left': return [rat(0, 1), t];
    case 'right': return [rat(1, 1), t];
    case 'bottom': return [t, rat(0, 1)];
    case 'top': return [t, rat(1, 1)];
    default: throw new Error(side);
  }
}

// ---------------------------------------------------------------------------
// Weld record.
// ---------------------------------------------------------------------------

class Weld {
  constructor(cfg) {
    this.id = cfg.id;
    this.patchA = cfg.patchA;
    this.sideA = cfg.sideA;
    this.patchB = cfg.patchB;
    this.sideB = cfg.sideB;
    this.flip = !!cfg.flip;
    this.level = 0;
    // t-key -> canonical 3D position, single sampling identity (unified only).
    this.canonical = new Map();
    // t-key -> { A?: pos, B?: pos }, per-side samples (negotiated only).
    this.negotiated = new Map();
    this.tightTol = cfg.tightTol;
    this.forcedLevel = 0; // level demanded by cross-side propagation
    this.strategy = cfg.strategy || 'unified-cached';
  }
}

// Map a local-side arc parameter to the canonical A-side arc parameter.
function toCanonicalT(weld, local, tLocal) {
  if (local === 'A') return tLocal;
  return weld.flip ? rat(tLocal.d - tLocal.n, tLocal.d) : tLocal;
}

// Map a canonical A-side arc parameter to a local-side arc parameter.
function fromCanonicalT(weld, local, tA) {
  if (local === 'A') return tA;
  return weld.flip ? rat(tA.d - tA.n, tA.d) : tA;
}

// ---------------------------------------------------------------------------
// Strategy: unified cached boundary split decision (historical default).
// ---------------------------------------------------------------------------

class UnifiedCachedStrategy {
  constructor() {
    this.name = 'unified-cached';
  }

  registerEndpoints(ctx, weld) {
    const pmA = ctx.patchByName.get(weld.patchA);
    const pmB = ctx.patchByName.get(weld.patchB);
    for (const t of [rat(0, 1), rat(1, 1)]) {
      const uvA = uvAtT(weld.sideA, t);
      const vA = pmA.getOrCreateVertex(uvA);
      weld.canonical.set(ratKey(t), vA.p);
      const tB = fromCanonicalT(weld, 'B', t);
      pmB.getOrCreateVertex(uvAtT(weld.sideB, tB), vA.p);
    }
    weld.level = 0;
  }

  // Canonical position for a boundary midpoint.  Returns a 3D position that
  // both sides must adopt verbatim, or undefined to fall back to own-surface
  // evaluation (never happens for a registered weld).
  midpointPosition(ctx, pm, weldEntry, uv) {
    const { weld, local, side } = weldEntry;
    const tLocal = sideTRat(side, uv);
    const tA = toCanonicalT(weld, local, tLocal);
    const keyA = ratKey(tA);
    let canonicalPos = weld.canonical.get(keyA);
    if (!canonicalPos) {
      const uvA = uvAtT(weld.sideA, tA);
      const pmA = ctx.patchByName.get(weld.patchA);
      canonicalPos = pmA.getOrCreateVertex(uvA).p;
      weld.canonical.set(keyA, canonicalPos);
    }
    return canonicalPos;
  }

  onRefine(ctx, pm, tri, verdict) {
    const side = triangleWeldSide(ctx, pm, tri);
    if (!side) return;
    const level = tri.depth + 1;
    const reason = verdict.reasons[0] ? verdict.reasons[0].detail : 'tolerance';
    this._ensureLevel(ctx, pm, side, level, reason);
  }

  _ensureLevel(ctx, pm, side, requiredLevel, reasonDetail) {
    const entry = ctx.weldByPatchSide.get(`${pm.patch.name}:${side}`);
    if (!entry) return;
    const { weld, local } = entry;
    if (requiredLevel <= weld.forcedLevel) return;
    const old = weld.forcedLevel;
    weld.forcedLevel = requiredLevel;
    weld.level = Math.max(weld.level, requiredLevel);
    ctx.events.push({
      type: 'weld-propagate', strategy: this.name, weld: weld.id,
      fromLevel: old, toLevel: requiredLevel, reason: reasonDetail,
    });
    const otherPM = local === 'A'
      ? ctx.patchByName.get(weld.patchB)
      : ctx.patchByName.get(weld.patchA);
    const otherSide = local === 'A' ? weld.sideB : weld.sideA;
    ctx.forceBoundaryLevel(otherPM, otherSide, requiredLevel, weld);
  }

  describe(weld) {
    return {
      strategy: this.name,
      level: Math.max(weld.level, weld.forcedLevel),
      canonicalVertices: weld.canonical.size,
    };
  }
}

// ---------------------------------------------------------------------------
// Strategy: neighbour-negotiated shared-edge refinement depth.
// ---------------------------------------------------------------------------

class NegotiatedDepthStrategy {
  constructor() {
    this.name = 'negotiated-depth';
  }

  registerEndpoints(ctx, weld) {
    // Only the shared end points are forced to a common identity; interior
    // samples are each side's own surface evaluation.
    const pmA = ctx.patchByName.get(weld.patchA);
    const pmB = ctx.patchByName.get(weld.patchB);
    for (const t of [rat(0, 1), rat(1, 1)]) {
      const uvA = uvAtT(weld.sideA, t);
      const vA = pmA.getOrCreateVertex(uvA);
      const tB = fromCanonicalT(weld, 'B', t);
      pmB.getOrCreateVertex(uvAtT(weld.sideB, tB), vA.p);
    }
    weld.level = 0;
  }

  // Each side evaluates its own surface at the midpoint parameter; the shared
  // end identities remain from registerEndpoints.
  midpointPosition() {
    return undefined;
  }

  onRefine(ctx, pm, tri, verdict) {
    const side = triangleWeldSide(ctx, pm, tri);
    if (!side) return;
    const entry = ctx.weldByPatchSide.get(`${pm.patch.name}:${side}`);
    if (!entry) return;
    const { weld, local } = entry;
    const demanded = tri.depth + 1;
    if (demanded <= weld.forcedLevel) return;
    const old = weld.forcedLevel;
    weld.forcedLevel = demanded;
    weld.level = Math.max(weld.level, demanded);
    // Negotiation: both sides agree the shared edge reaches `demanded`, and
    // each independently subdivides to it with its own samples.
    const reason = verdict.reasons[0] ? verdict.reasons[0].detail : 'tolerance';
    ctx.events.push({
      type: 'weld-negotiate', strategy: this.name, weld: weld.id,
      fromLevel: old, toLevel: demanded, reason,
    });
    const pmA = ctx.patchByName.get(weld.patchA);
    const pmB = ctx.patchByName.get(weld.patchB);
    ctx.forceBoundaryLevel(pmA, weld.sideA, demanded, weld);
    ctx.forceBoundaryLevel(pmB, weld.sideB, demanded, weld);
  }

  describe(weld) {
    return {
      strategy: this.name,
      level: Math.max(weld.level, weld.forcedLevel),
      canonicalVertices: weld.canonical.size,
    };
  }
}

// ---------------------------------------------------------------------------
// Driver-facing helpers.
// ---------------------------------------------------------------------------

function triangleWeldSide(ctx, pm, tri) {
  for (const side of SIDES) {
    if (ctx.weldByPatchSide.has(`${pm.patch.name}:${side}`) &&
        tri.v.some((v) => onSideLine(side, v))) {
      return side;
    }
  }
  return null;
}

function weldForEdge(ctx, pm, a, b) {
  for (const side of SIDES) {
    const entry = ctx.weldByPatchSide.get(`${pm.patch.name}:${side}`);
    if (entry && onSideLine(side, a) && onSideLine(side, b)) {
      return { ...entry, side };
    }
  }
  return null;
}

const strategies = {
  'unified-cached': UnifiedCachedStrategy,
  'negotiated-depth': NegotiatedDepthStrategy,
};

function resolveStrategy(spec) {
  if (!spec) return new UnifiedCachedStrategy();
  if (typeof spec === 'string') {
    const Cls = strategies[spec];
    if (!Cls) throw new Error(`unknown boundary strategy ${spec}`);
    return new Cls();
  }
  if (typeof spec.onRefine === 'function') return spec; // already an instance
  throw new Error('invalid boundary strategy');
}

module.exports = {
  Weld,
  UnifiedCachedStrategy,
  NegotiatedDepthStrategy,
  strategies,
  resolveStrategy,
  sideUV,
  onSideLine,
  sideTRat,
  uvAtT,
  toCanonicalT,
  fromCanonicalT,
  triangleWeldSide,
  weldForEdge,
  SIDES,
};
