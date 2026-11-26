'use strict';

// Adaptive refinement driver.
//
// Meshing model (unchanged externally):
// each patch starts from a two-triangle seed on the unit parameter square and
// refines by longest-edge (Rivara) bisection.  Splits propagate to the
// neighbour opposite the split edge so the mesh inside each patch stays
// conforming.  Cross-patch welded edges carry a shared sampling identity
// (boundary.js), so independently refined neighbours stay crack-free.
//
// This driver is the *policy* layer.  It does not compute error and it does
// not decide how seams agree; it consumes:
//   - explicit per-cell estimates from a pluggable error metric
//     (estimation.js; verdicts are the only refinement input), and
//   - shared-edge decisions from a pluggable boundary strategy
//     (boundary.js).
// Both are swappable at run time.  The local error estimate and the crack
// constraint are therefore independent: changing the metric changes where we
// refine, never the boundary identity.

const { rat, ratVal, ratMid } = require('../math');
const keys = require('./keys');
const { uvKey, edgeKey } = keys;
const {
  Weld, resolveStrategy, sideUV, onSideLine, sideTRat, uvAtT,
  triangleWeldSide, weldForEdge,
} = require('./boundary');
const { resolveEstimator } = require('./estimation');

const MAX_DEPTH = 30;
const MAX_RELAX = 16; // remote patches may degrade at most 16x declared tol

class PatchMesh {
  constructor(patch) {
    this.patch = patch;
    this.locked = false;
    this.vertices = new Map();
    this.triangles = [];
    this.edgeAdj = new Map(); // edge key -> Set of triangle ids
    this.nextTriId = 0;
    this.unsatisfied = [];
    this.reasonCounts = {};
    this.coarsenRelax = 1; // effective tolerance multiplier (budget relief)
  }

  getOrCreateVertex(uv, forcedPos) {
    const key = uvKey(uv);
    let v = this.vertices.get(key);
    if (!v) {
      const p = forcedPos || this.patch.pos(ratVal(uv[0]), ratVal(uv[1]));
      v = { id: this.vertices.size, uv, p };
      this.vertices.set(key, v);
    } else if (forcedPos) {
      v.p = forcedPos;
    }
    return v;
  }

  addTriangle(verts, depth) {
    const tri = { id: this.nextTriId++, v: verts, depth, alive: true, blocked: false };
    this.triangles.push(tri);
    for (let i = 0; i < 3; i++) {
      const key = edgeKey(verts[i], verts[(i + 1) % 3]);
      let set = this.edgeAdj.get(key);
      if (!set) {
        set = new Set();
        this.edgeAdj.set(key, set);
      }
      set.add(tri.id);
    }
    return tri;
  }

  removeTriangle(tri) {
    tri.alive = false;
    for (let i = 0; i < 3; i++) {
      const key = edgeKey(tri.v[i], tri.v[(i + 1) % 3]);
      const set = this.edgeAdj.get(key);
      if (set) {
        set.delete(tri.id);
        if (set.size === 0) this.edgeAdj.delete(key);
      }
    }
  }

  neighbourAcross(a, b, excludeId) {
    const set = this.edgeAdj.get(edgeKey(a, b));
    if (!set) return null;
    for (const id of set) {
      if (id !== excludeId) {
        const t = this.triangles[id];
        if (t.alive) return t;
      }
    }
    return null;
  }

  aliveTriangles() {
    return this.triangles.filter((t) => t.alive);
  }
}

// ---------------------------------------------------------------------------
// Driver.
// ---------------------------------------------------------------------------

class Mesher {
  constructor(scene, opts = {}) {
    this.scene = scene;
    this.patches = scene.patches.map((p) => {
      const pm = new PatchMesh(p);
      pm.locked = !!(scene.locks && scene.locks.includes(p.name));
      return pm;
    });
    this.patchByName = new Map(this.patches.map((pm) => [pm.patch.name, pm]));
    this.boundary = resolveStrategy(opts.boundaryStrategy || scene.boundaryStrategy);
    this.welds = (scene.welds || []).map((w) => this._buildWeld(w));
    this.weldByPatchSide = new Map();
    for (const w of this.welds) {
      this.weldByPatchSide.set(`${w.patchA}:${w.sideA}`, { weld: w, local: 'A' });
      this.weldByPatchSide.set(`${w.patchB}:${w.sideB}`, { weld: w, local: 'B' });
    }
    this.vertexCap = scene.vertexCap || Infinity;
    this.mode = scene.mode || 'coarsen-remote';
    this.events = [];
    this.exportRejected = false;
    this.rejectReason = null;
    this.audit = opts.audit || null;
    this.metricSwaps = [];
    this._runPass = 0;
    this.setEstimator(opts.estimator || scene.errorMetric, { silent: true });
  }

  get estimator() { return this._estimator; }

  // Swap the error metric at run time.  Only refinement decisions change;
  // boundary identity registered so far is left untouched, so seams already
  // agreed stay valid and verifiable vertex-by-vertex.
  setEstimator(spec, opts = {}) {
    const next = resolveEstimator(spec);
    const prev = this._estimator;
    this._estimator = next;
    if (!opts.silent) {
      this.metricSwaps.push({
        from: prev ? prev.name : null,
        to: next.name,
        pass: this._runPass,
        at: Date.now(),
      });
      this.events.push({
        type: 'metric-swap', from: prev ? prev.name : null,
        to: next.name, pass: this._runPass,
      });
      if (this.audit) {
        this.audit.record({ type: 'metric-swap', from: prev ? prev.name : null, to: next.name });
      }
    }
    return next;
  }

  _buildWeld(cfg) {
    const pa = this.patchByName.get(cfg.patchA).patch;
    const pb = this.patchByName.get(cfg.patchB).patch;
    const tightTol = Math.min(pa.tol.position, pb.tol.position);
    return new Weld({ ...cfg, tightTol, strategy: this.boundary.name });
  }

  static sideUV(side) { return sideUV(side); }
  static onSideLine(side, v) { return onSideLine(side, v); }
  static sideTRat(side, uv) { return sideTRat(side, uv); }
  static uvAtT(side, t) { return uvAtT(side, t); }

  // -------------------------------------------------------------------------
  // Seeding.
  // -------------------------------------------------------------------------

  _seed() {
    for (const pm of this.patches) {
      if (pm.patch.singularNodes.length > 0 && this.scene.fanSeed) {
        this._seedFan(pm);
      } else {
        this._seedQuad(pm);
      }
    }
    for (const weld of this.welds) this.boundary.registerEndpoints(this, weld);
  }

  _seedQuad(pm) {
    const c = pm.getOrCreateVertex([rat(0, 1), rat(0, 1)]);
    const b = pm.getOrCreateVertex([rat(1, 1), rat(0, 1)]);
    const a = pm.getOrCreateVertex([rat(1, 1), rat(1, 1)]);
    const d = pm.getOrCreateVertex([rat(0, 1), rat(1, 1)]);
    pm.addTriangle([c, b, a], 0);
    pm.addTriangle([c, a, d], 0);
  }

  _seedFan(pm) {
    const N = 8; // dyadic ring subdivisions
    const ring = [];
    for (let i = 0; i <= N; i++) {
      ring.push(pm.getOrCreateVertex([rat(i, N), rat(0, 1)]));
    }
    ring[N].p = ring[0].p;
    ring[N].seamTwin = ring[0].id;
    const pole = pm.getOrCreateVertex([rat(1, 2), rat(1, 1)]);
    for (let i = 1; i < N; i++) {
      const collapse = pm.getOrCreateVertex([rat(i, N), rat(1, 1)], pole.p);
      collapse.collapsedTo = pole.id;
    }
    for (let i = 0; i < N; i++) {
      const top = i === 0
        ? pm.getOrCreateVertex([rat(0, 1), rat(1, 1)], pole.p)
        : pm.getOrCreateVertex([rat(i, N), rat(1, 1)]);
      const topNext = pm.getOrCreateVertex([rat(i + 1, N), rat(1, 1)]);
      pm.addTriangle([ring[i], ring[i + 1], topNext], 0);
      pm.addTriangle([ring[i], topNext, top], 0);
    }
  }

  totalVertices() {
    let n = 0;
    for (const pm of this.patches) n += pm.vertices.size;
    return n;
  }

  budgetRemaining() {
    return this.vertexCap - this.totalVertices();
  }

  // -------------------------------------------------------------------------
  // Splitting (conforming inside a patch).
  // -------------------------------------------------------------------------

  _longestEdge(tri) {
    let best = null;
    let bestLen = -1;
    for (let i = 0; i < 3; i++) {
      const a = tri.v[i];
      const b = tri.v[(i + 1) % 3];
      const d = Math.hypot(ratVal(a.uv[0]) - ratVal(b.uv[0]), ratVal(a.uv[1]) - ratVal(b.uv[1]));
      if (d > bestLen) {
        bestLen = d;
        best = [a, b];
      }
    }
    return best;
  }

  _midpointVertex(pm, a, b) {
    const uv = [ratMid(a.uv[0], b.uv[0]), ratMid(a.uv[1], b.uv[1])];
    const existing = pm.vertices.get(uvKey(uv));
    if (existing) return existing;
    const weldEntry = weldForEdge(this, pm, a, b);
    if (weldEntry) {
      const pos = this.boundary.midpointPosition(this, pm, weldEntry, uv);
      if (pos) return pm.getOrCreateVertex(uv, pos);
    }
    return pm.getOrCreateVertex(uv);
  }

  _splitTriangle(pm, tri, stack, guard) {
    guard = guard || 0;
    if (!tri.alive || tri.blocked || guard > MAX_DEPTH + 4) return;
    const [a, b] = this._longestEdge(tri);
    const neighbour = pm.neighbourAcross(a, b, tri.id);
    if (neighbour && neighbour.depth < tri.depth) {
      this._splitTriangle(pm, neighbour, stack, guard + 1);
    }
    if (!tri.alive) return;
    const m = this._midpointVertex(pm, a, b);
    const third = tri.v.find((v) => v !== a && v !== b);
    pm.removeTriangle(tri);
    const t1 = pm.addTriangle([a, m, third], tri.depth + 1);
    const t2 = pm.addTriangle([m, b, third], tri.depth + 1);
    stack.push(t1, t2);
  }

  // -------------------------------------------------------------------------
  // Weld level enforcement (delegated to the boundary strategy).
  // -------------------------------------------------------------------------

  _boundaryMinLevel(pm, side) {
    let min = Infinity;
    for (const tri of pm.aliveTriangles()) {
      if (tri.v.some((v) => onSideLine(side, v))) min = Math.min(min, tri.depth);
    }
    return min === Infinity ? 0 : min;
  }

  _forceBoundaryLevel(pm, side, level, weld) {
    let guard = 0;
    while (this._boundaryMinLevel(pm, side) < level && guard < MAX_DEPTH + 2) {
      let pick = null;
      for (const tri of pm.aliveTriangles()) {
        if (!tri.v.some((v) => onSideLine(side, v))) continue;
        if (!pick || tri.depth < pick.depth) pick = tri;
      }
      if (!pick) break;
      if (this.budgetRemaining() <= 0) {
        pick.blocked = true;
        pm.unsatisfied.push({
          tri: pick,
          reason: { code: 'budget-weld', detail: `weld ${weld.id} demands level ${level}` },
        });
        this.events.push({ type: 'budget-blocked-weld', weld: weld.id, patch: pm.patch.name });
        break;
      }
      this._splitTriangle(pm, pick, []);
      guard++;
    }
  }

  // Public driver mechanics exposed to boundary strategies (which must not
  // depend on private internals).  These are the only driver methods the
  // strategies call.
  forceBoundaryLevel(pm, side, level, weld) {
    return this._forceBoundaryLevel(pm, side, level, weld);
  }

  // -------------------------------------------------------------------------
  // Error evaluation: consumes the pluggable metric, records explicit evidence.
  // -------------------------------------------------------------------------

  _effectiveTol(pm) {
    return {
      position: pm.patch.tol.position * pm.coarsenRelax,
      normal: pm.patch.tol.normal * pm.coarsenRelax,
    };
  }

  _evaluate(pm, tri) {
    const verdict = this._estimator.evaluate({
      surface: pm.patch,
      tri,
      depth: tri.depth,
      maxDepth: MAX_DEPTH,
      tol: this._effectiveTol(pm),
    });
    tri.verdict = verdict;
    tri.metricName = this._estimator.name;
    return verdict;
  }

  // -------------------------------------------------------------------------
  // Adaptive driver.
  // -------------------------------------------------------------------------

  run() {
    this._seed();
    for (let pass = 0; pass < MAX_DEPTH + 4; pass++) {
      this._runPass = pass;
      this._repairConformity();
      // Batch: evaluate every leaf once, then split every failing leaf.
      const failing = [];
      for (const pm of this.patches) {
        for (const tri of pm.aliveTriangles()) {
          if (tri.blocked) continue;
          const verdict = this._evaluate(pm, tri);
          if (verdict.needsRefine) failing.push({ pm, tri, verdict });
        }
      }
      if (failing.length === 0) break;
      failing.sort((a, b) => b.verdict.metrics.h - a.verdict.metrics.h);
      let budgetStalled = false;
      for (const item of failing) {
        if (!item.tri.alive || item.tri.blocked) continue;
        if (this.budgetRemaining() <= 0) {
          const progressed = this._handleBudget(item);
          if (this.exportRejected) break;
          if (!progressed) {
            item.tri.blocked = true;
            item.pm.unsatisfied.push({
              tri: item.tri,
              reason: item.verdict.reasons[0] || { code: 'budget', detail: 'vertex cap' },
            });
            budgetStalled = true;
          }
        }
        if (this.exportRejected) break;
        this._execute(item);
      }
      if (this.exportRejected) break;
      if (budgetStalled && this.budgetRemaining() <= 0) break;
    }
    this._repairConformity();
    this._collectUnsatisfied();
    return this._buildResult();
  }

  _repairConformity() {
    let repairs = 0;
    for (const pm of this.patches) {
      let dirty = true;
      let guard = 0;
      while (dirty && guard++ < MAX_DEPTH * 4) {
        dirty = false;
        for (const [key, set] of pm.edgeAdj) {
          if (set.size !== 2) continue;
          const ids = [...set];
          const t0 = pm.triangles[ids[0]];
          const t1 = pm.triangles[ids[1]];
          if (Math.abs(t0.depth - t1.depth) > 1) {
            const shallow = t0.depth < t1.depth ? t0 : t1;
            if (this.budgetRemaining() <= 0) {
              shallow.blocked = true;
              return repairs;
            }
            this._splitTriangle(pm, shallow, []);
            repairs++;
            dirty = true;
            break;
          }
        }
      }
    }
    return repairs;
  }

  _execute(action) {
    const { pm, tri, verdict } = action;
    for (const r of verdict.reasons) {
      pm.reasonCounts[r.code] = (pm.reasonCounts[r.code] || 0) + 1;
    }
    // The explicit estimate the policy consumed, recorded for audit/evidence.
    if (this.audit) {
      this.audit.record({
        type: 'estimate', metric: this._estimator.name, patch: pm.patch.name,
        depth: tri.depth, needsRefine: verdict.needsRefine,
        h: verdict.metrics.h, sampledDist: verdict.metrics.sampledDist,
        reasons: verdict.reasons.map((r) => r.code),
      });
    }
    this.boundary.onRefine(this, pm, tri, verdict);
    this._splitTriangle(pm, tri, []);
  }

  _handleBudget(action) {
    const lockedRequester = action.pm.locked;
    const releasable = this.patches.filter((pm) => !pm.locked && pm.aliveTriangles().some((t) => t.depth > 0));
    if (this.mode === 'reject') {
      this.exportRejected = true;
      this.rejectReason = `vertex cap ${this.vertexCap} reached in reject mode; locked patches preserved, export refused`;
      this.events.push({ type: 'export-rejected', cap: this.vertexCap });
      return false;
    }
    if (releasable.length === 0 || (lockedRequester && releasable.every((pm) => pm === action.pm))) {
      this.exportRejected = true;
      this.rejectReason = `vertex cap ${this.vertexCap} reached and no unlocked remote patch can release vertices`;
      this.events.push({ type: 'export-rejected', cap: this.vertexCap });
      return false;
    }
    const victim = releasable
      .filter((pm) => pm !== action.pm)
      .sort((a, b) => b.aliveTriangles().length - a.aliveTriangles().length)[0];
    if (!victim) {
      this.exportRejected = true;
      this.rejectReason = 'vertex cap reached; only the requesting patch has refinements to spare';
      return false;
    }
    if (victim.coarsenRelax >= MAX_RELAX) {
      this.exportRejected = true;
      this.rejectReason = `vertex cap ${this.vertexCap} reached; remote patch ${victim.patch.name} already at its ${MAX_RELAX}x precision floor, export refused`;
      this.events.push({ type: 'export-rejected', cap: this.vertexCap, floorPatch: victim.patch.name });
      return false;
    }
    this._rebuildPatchRelaxed(victim);
    this.events.push({
      type: 'coarsen-remote',
      patch: victim.patch.name,
      relax: victim.coarsenRelax,
      verticesAfter: victim.vertices.size,
    });
    return true;
  }

  _rebuildPatchRelaxed(pm) {
    pm.coarsenRelax *= 2;
    const oldCount = pm.vertices.size;
    pm.vertices = new Map();
    pm.triangles = [];
    pm.edgeAdj = new Map();
    pm.nextTriId = 0;
    pm.unsatisfied = [];
    const c = pm.getOrCreateVertex([rat(0, 1), rat(0, 1)]);
    const b = pm.getOrCreateVertex([rat(1, 1), rat(0, 1)]);
    const a = pm.getOrCreateVertex([rat(1, 1), rat(1, 1)]);
    const d = pm.getOrCreateVertex([rat(0, 1), rat(1, 1)]);
    pm.addTriangle([c, b, a], 0);
    pm.addTriangle([c, a, d], 0);
    // Re-attach the shared sampling identity this patch participates in.
    for (const weld of this.welds) {
      const local = weld.patchA === pm.patch.name ? 'A' : weld.patchB === pm.patch.name ? 'B' : null;
      if (!local) continue;
      const side = local === 'A' ? weld.sideA : weld.sideB;
      for (const [tKey, pos] of weld.canonical.entries()) {
        const [n, dden] = tKey.split('/').map(Number);
        const t = rat(n, dden);
        const tLocal = local === 'B' && weld.flip ? rat(t.d - t.n, t.d) : t;
        const uv = uvAtT(side, tLocal);
        pm.getOrCreateVertex(uv, pos);
      }
      this._forceBoundaryLevel(pm, side, weld.forcedLevel, weld);
    }
    this.events.push({ type: 'patch-rebuilt', patch: pm.patch.name, before: oldCount, after: pm.vertices.size });
  }

  _collectUnsatisfied() {
    for (const pm of this.patches) {
      for (const tri of pm.aliveTriangles()) {
        const verdict = tri.verdict || this._evaluate(pm, tri);
        const failing = verdict.needsRefine || verdict.capped || tri.blocked ||
          pm.coarsenRelax > 1 && this._aboveDeclaredTol(pm, verdict);
        if (failing && !pm.unsatisfied.some((u) => u.tri === tri)) {
          pm.unsatisfied.push({
            tri,
            reason: tri.blocked
              ? { code: 'budget', detail: 'vertex cap prevented further refinement' }
              : verdict.reasons[0] || { code: 'relaxed-tol', detail: `tolerance relaxed x${pm.coarsenRelax} for budget` },
          });
        }
      }
    }
  }

  _aboveDeclaredTol(pm, verdict) {
    const t = pm.patch.tol;
    return verdict.metrics.sampledDist > t.position || verdict.metrics.sampledAngle > t.normal;
  }

  _buildResult() {
    const patches = this.patches.map((pm) => {
      const tris = pm.aliveTriangles();
      const sampleMax = { dist: 0, angle: 0 };
      const boundMax = { dist: 0, angle: 0 };
      for (const t of tris) {
        const m = t.verdict.metrics;
        sampleMax.dist = Math.max(sampleMax.dist, m.sampledDist);
        sampleMax.angle = Math.max(sampleMax.angle, m.sampledAngle);
        boundMax.dist = Math.max(boundMax.dist, m.posBound);
        boundMax.angle = Math.max(boundMax.angle, m.nrmBound);
      }
      return {
        name: pm.patch.name,
        locked: pm.locked,
        vertices: pm.vertices.size,
        triangles: tris.length,
        // Guaranteed only when the surface has an analytic bound AND the
        // active metric actually used it.  A sampled-only metric never proves
        // anything, even on a surface that could have been bounded.
        guaranteed: pm.patch.boundIsGuaranteed && this._estimator.kind !== 'sampled',
        errorMetric: this._estimator.name,
        relax: pm.coarsenRelax,
        tolerances: pm.patch.tol,
        sampleMax,
        boundMax,
        reasonCounts: pm.reasonCounts,
        unsatisfied: pm.unsatisfied.map((u) => ({
          triangleId: u.tri.id,
          depth: u.tri.depth,
          reason: u.reason,
        })),
      };
    });
    return {
      rejected: this.exportRejected,
      rejectReason: this.rejectReason,
      vertexCap: this.vertexCap,
      mode: this.mode,
      boundaryStrategy: this.boundary.name,
      errorMetric: this._estimator.name,
      metricSwaps: this.metricSwaps,
      totalVertices: this.totalVertices(),
      totalTriangles: this.patches.reduce((s, pm) => s + pm.aliveTriangles().length, 0),
      welds: this.welds.map((w) => ({
        id: w.id,
        ...this.boundary.describe(w),
        patchA: w.patchA, sideA: w.sideA,
        patchB: w.patchB, sideB: w.sideB,
        tightTol: w.tightTol,
      })),
      events: this.events,
      patches,
      _mesher: this,
    };
  }
}

module.exports = {
  Mesher, PatchMesh, Weld,
  edgeKey, uvKey, keys,
  MAX_DEPTH, MAX_RELAX,
};
