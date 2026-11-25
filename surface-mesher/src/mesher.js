'use strict';

// Adaptive parametric-surface discretiser.
//
// Meshing model
// -------------
// Each ParamSurface patch starts from a two-triangle seed on the unit
// parameter square.  Triangles are refined by longest-edge (Rivara)
// bisection.  A split always propagates to the neighbour opposite the split
// edge, so the mesh inside each patch is conforming: no T-junctions and no
// hanging vertices.
//
// Cross-patch welded edges carry a single sampling identity.  The weld
// registry maps each dyadic arc parameter t (measured on side A) to one
// canonical 3D position.  Side B looks the position up through the same t
// (flipped when orientations differ), so independently refined neighbours
// still produce coincident boundary vertices and no cracks.  When one side
// needs a finer edge level the requirement propagates and forces the other
// side to subdivide to match.  Deliberately separated "lips" are not
// registered as welds and keep independent vertices and tolerances.
//
// A hard global vertex cap is enforced at every split.  When the cap is
// reached the mesher either (mode 'coarsen-remote') rebuilds unlocked,
// non-locked patches at looser effective tolerances to free vertices, or
// (mode 'reject') preserves locked patches and refuses the export.

const { rat, ratVal, ratMid, ratKey, vdist } = require('./math');
const { evaluateTriangle } = require('./error');

const MAX_DEPTH = 30;
const MAX_RELAX = 16; // remote patches may degrade at most 16x declared tol

function uvKey(uv) {
  return `${ratKey(uv[0])}|${ratKey(uv[1])}`;
}

function edgeKey(v1, v2) {
  const a = uvKey(v1.uv);
  const b = uvKey(v2.uv);
  return a < b ? `${a}#${b}` : `${b}#${a}`;
}

class Weld {
  constructor(cfg) {
    this.id = cfg.id;
    this.patchA = cfg.patchA;
    this.sideA = cfg.sideA;
    this.patchB = cfg.patchB;
    this.sideB = cfg.sideB;
    this.flip = !!cfg.flip;
    this.level = 0;
    // t-key -> canonical 3D position, single sampling identity.
    this.canonical = new Map();
    this.tightTol = cfg.tightTol;
    this.forcedLevel = 0; // level demanded by cross-side propagation
  }
}

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

class Mesher {
  constructor(scene) {
    this.scene = scene;
    this.patches = scene.patches.map((p) => {
      const pm = new PatchMesh(p);
      pm.locked = !!(scene.locks && scene.locks.includes(p.name));
      return pm;
    });
    this.patchByName = new Map(this.patches.map((pm) => [pm.patch.name, pm]));
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
  }

  _buildWeld(cfg) {
    const pa = this.patchByName.get(cfg.patchA).patch;
    const pb = this.patchByName.get(cfg.patchB).patch;
    const tightTol = Math.min(pa.tol.position, pb.tol.position);
    return new Weld({ ...cfg, tightTol });
  }

  static sideUV(side) {
    switch (side) {
      case 'left': return [[rat(0, 1), rat(0, 1)], [rat(0, 1), rat(1, 1)]];
      case 'right': return [[rat(1, 1), rat(0, 1)], [rat(1, 1), rat(1, 1)]];
      case 'bottom': return [[rat(0, 1), rat(0, 1)], [rat(1, 1), rat(0, 1)]];
      case 'top': return [[rat(0, 1), rat(1, 1)], [rat(1, 1), rat(1, 1)]];
      default: throw new Error(`unknown side ${side}`);
    }
  }

  static onSideLine(side, v) {
    const u = ratVal(v.uv[0]);
    const w = ratVal(v.uv[1]);
    if (side === 'left') return u === 0;
    if (side === 'right') return u === 1;
    if (side === 'bottom') return w === 0;
    if (side === 'top') return w === 1;
    return false;
  }

  // Arc parameter t (dyadic) along a side for a vertex on that side.
  static sideTRat(side, uv) {
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

  static uvAtT(side, t) {
    switch (side) {
      case 'left': return [rat(0, 1), t];
      case 'right': return [rat(1, 1), t];
      case 'bottom': return [t, rat(0, 1)];
      case 'top': return [t, rat(1, 1)];
      default: throw new Error(side);
    }
  }

  _seed() {
    for (const pm of this.patches) {
      if (pm.patch.singularNodes.length > 0 && this.scene.fanSeed) {
        this._seedFan(pm);
      } else {
        this._seedQuad(pm);
      }
    }
    for (const weld of this.welds) this._registerWeldEndpoints(weld);
  }

  _seedQuad(pm) {
    const c = pm.getOrCreateVertex([rat(0, 1), rat(0, 1)]);
    const b = pm.getOrCreateVertex([rat(1, 1), rat(0, 1)]);
    const a = pm.getOrCreateVertex([rat(1, 1), rat(1, 1)]);
    const d = pm.getOrCreateVertex([rat(0, 1), rat(1, 1)]);
    pm.addTriangle([c, b, a], 0);
    pm.addTriangle([c, a, d], 0);
  }

  // Fan seed for a repeated pole node on the v=1 edge.  One shared pole
  // vertex (canonical u=1/2) replaces the whole collapsed top row; an
  // equatorial ring of dyadic u samples gives radial triangles, and the
  // u=0/u=1 seam is welded into a single 3D identity by shared position.
  _seedFan(pm) {
    const N = 8; // dyadic ring subdivisions
    const ring = [];
    for (let i = 0; i <= N; i++) {
      ring.push(pm.getOrCreateVertex([rat(i, N), rat(0, 1)]));
    }
    // Equatorial u=0 and u=1 are the same physical meridian seam.
    ring[N].p = ring[0].p;
    ring[N].seamTwin = ring[0].id;
    const pole = pm.getOrCreateVertex([rat(1, 2), rat(1, 1)]);
    // Intermediate top-row vertices that would collapse onto the pole are
    // aliased to the single pole vertex position/identity.
    for (let i = 1; i < N; i++) {
      const collapse = pm.getOrCreateVertex([rat(i, N), rat(1, 1)], pole.p);
      collapse.collapsedTo = pole.id;
    }
    for (let i = 0; i < N; i++) {
      const top = i === 0
        ? pm.getOrCreateVertex([rat(0, 1), rat(1, 1)], pole.p)
        : pm.getOrCreateVertex([rat(i, N), rat(1, 1)]);
      const topNext = pm.getOrCreateVertex([rat(i + 1, N), rat(1, 1)]);
      // Each radial quad splits into two triangles sharing the pole.
      pm.addTriangle([ring[i], ring[i + 1], topNext], 0);
      pm.addTriangle([ring[i], topNext, top], 0);
    }
  }

  _registerWeldEndpoints(weld) {
    const pmA = this.patchByName.get(weld.patchA);
    const pmB = this.patchByName.get(weld.patchB);
    const [, e1] = Mesher.sideUV(weld.sideA);
    const tEnds = [rat(0, 1), rat(1, 1)];
    for (const t of tEnds) {
      const uvA = Mesher.uvAtT(weld.sideA, t);
      const vA = pmA.getOrCreateVertex(uvA);
      weld.canonical.set(ratKey(t), vA.p);
      const tB = weld.flip ? rat(t.d - t.n, t.d) : t;
      const uvB = Mesher.uvAtT(weld.sideB, tB);
      pmB.getOrCreateVertex(uvB, vA.p);
    }
    weld.level = 0;
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
  // Longest-edge bisection with conforming neighbour propagation.
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

  _weldForEdge(pm, a, b) {
    for (const side of ['left', 'right', 'bottom', 'top']) {
      const entry = this.weldByPatchSide.get(`${pm.patch.name}:${side}`);
      if (entry && Mesher.onSideLine(side, a) && Mesher.onSideLine(side, b)) {
        return { ...entry, side };
      }
    }
    return null;
  }

  _midpointVertex(pm, a, b) {
    const uv = [ratMid(a.uv[0], b.uv[0]), ratMid(a.uv[1], b.uv[1])];
    const existing = pm.vertices.get(uvKey(uv));
    if (existing) return existing;
    const weldEntry = this._weldForEdge(pm, a, b);
    if (weldEntry) {
      const { weld, local, side } = weldEntry;
      const tLocal = Mesher.sideTRat(side, uv);
      const tA = local === 'A' ? tLocal : (weld.flip ? rat(tLocal.d - tLocal.n, tLocal.d) : tLocal);
      const keyA = ratKey(tA);
      let canonicalPos = weld.canonical.get(keyA);
      if (!canonicalPos) {
        const uvA = Mesher.uvAtT(weld.sideA, tA);
        const pmA = this.patchByName.get(weld.patchA);
        const vA = pmA.getOrCreateVertex(uvA);
        canonicalPos = vA.p;
        weld.canonical.set(keyA, canonicalPos);
      }
      return pm.getOrCreateVertex(uv, canonicalPos);
    }
    return pm.getOrCreateVertex(uv);
  }

  _splitTriangle(pm, tri, stack, guard) {
    guard = guard || 0;
    if (!tri.alive || tri.blocked || guard > MAX_DEPTH + 4) return;
    const [a, b] = this._longestEdge(tri);
    const neighbour = pm.neighbourAcross(a, b, tri.id);
    // Rivara rule: only a *shallower* neighbour across the longest edge is
    // split first.  An equal-depth neighbour shares the edge conformingly and
    // must not recurse, otherwise the two triangles trigger each other.
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
  // Weld level enforcement (shared subdivision identity).
  // -------------------------------------------------------------------------

  _triangleWeldSide(pm, tri) {
    for (const side of ['left', 'right', 'bottom', 'top']) {
      if (this.weldByPatchSide.has(`${pm.patch.name}:${side}`) &&
          tri.v.some((v) => Mesher.onSideLine(side, v))) {
        return side;
      }
    }
    return null;
  }

  _boundaryMinLevel(pm, side) {
    let min = Infinity;
    for (const tri of pm.aliveTriangles()) {
      if (tri.v.some((v) => Mesher.onSideLine(side, v))) min = Math.min(min, tri.depth);
    }
    return min === Infinity ? 0 : min;
  }

  _ensureWeldLevel(pm, side, requiredLevel, reasonDetail) {
    const entry = this.weldByPatchSide.get(`${pm.patch.name}:${side}`);
    if (!entry) return;
    const { weld, local } = entry;
    if (requiredLevel <= weld.forcedLevel) return;
    const old = weld.forcedLevel;
    weld.forcedLevel = requiredLevel;
    weld.level = Math.max(weld.level, requiredLevel);
    this.events.push({
      type: 'weld-propagate', weld: weld.id, fromLevel: old,
      toLevel: requiredLevel, reason: reasonDetail,
    });
    const otherPM = local === 'A' ? this.patchByName.get(weld.patchB) : this.patchByName.get(weld.patchA);
    const otherSide = local === 'A' ? weld.sideB : weld.sideA;
    this._forceBoundaryLevel(otherPM, otherSide, requiredLevel, weld);
  }

  _forceBoundaryLevel(pm, side, level, weld) {
    let guard = 0;
    while (this._boundaryMinLevel(pm, side) < level && guard < MAX_DEPTH + 2) {
      let pick = null;
      for (const tri of pm.aliveTriangles()) {
        if (!tri.v.some((v) => Mesher.onSideLine(side, v))) continue;
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

  // -------------------------------------------------------------------------
  // Adaptive driver.
  // -------------------------------------------------------------------------

  _effectiveTol(pm) {
    return {
      position: pm.patch.tol.position * pm.coarsenRelax,
      normal: pm.patch.tol.normal * pm.coarsenRelax,
    };
  }

  _evaluate(pm, tri) {
    const verdict = evaluateTriangle(pm.patch, tri, tri.depth, MAX_DEPTH, this._effectiveTol(pm));
    tri.verdict = verdict;
    return verdict;
  }

  run() {
    this._seed();
    for (let pass = 0; pass < MAX_DEPTH + 4; pass++) {
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
      // Worst first keeps budget behaviour deterministic and weld-critical
      // triangles ahead of gentle ones.
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

  // Any edge shared by triangles whose depths differ by more than one is
  // non-conforming (a hanging vertex).  Split the shallower side until the
  // mesh is conforming again.  Returns the number of repair splits.
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
    const side = this._triangleWeldSide(pm, tri);
    for (const r of verdict.reasons) {
      pm.reasonCounts[r.code] = (pm.reasonCounts[r.code] || 0) + 1;
    }
    if (side) {
      this._ensureWeldLevel(pm, side, tri.depth + 1, verdict.reasons[0]?.detail || 'tolerance');
    }
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
    // Never coarsen a locked patch; if everything relevant is locked, reject.
    if (releasable.length === 0 || (lockedRequester && releasable.every((pm) => pm === action.pm))) {
      this.exportRejected = true;
      this.rejectReason = `vertex cap ${this.vertexCap} reached and no unlocked remote patch can release vertices`;
      this.events.push({ type: 'export-rejected', cap: this.vertexCap });
      return false;
    }
    // Pick the remote unlocked patch with the most leaf triangles and relax
    // its effective tolerance, then rebuild that patch from its seed.
    const victim = releasable
      .filter((pm) => pm !== action.pm)
      .sort((a, b) => b.aliveTriangles().length - a.aliveTriangles().length)[0];
    if (!victim) {
      this.exportRejected = true;
      this.rejectReason = 'vertex cap reached; only the requesting patch has refinements to spare';
      return false;
    }
    // A patch may only surrender precision up to MAX_RELAX times its declared
    // tolerance; beyond that the frame cannot be honoured, so reject instead
    // of degrading geometry without limit.
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
    // Double the effective tolerance (declared tolerance is exceeded in this
    // patch and reported as an unsatisfied region) and remesh from scratch.
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
    // Re-attach weld endpoint identities for this patch.
    for (const weld of this.welds) {
      const local = weld.patchA === pm.patch.name ? 'A' : weld.patchB === pm.patch.name ? 'B' : null;
      if (!local) continue;
      const side = local === 'A' ? weld.sideA : weld.sideB;
      for (const [tKey, pos] of weld.canonical.entries()) {
        const [n, dden] = tKey.split('/').map(Number);
        const t = rat(n, dden);
        const tLocal = local === 'B' && weld.flip ? rat(t.d - t.n, t.d) : t;
        const uv = Mesher.uvAtT(side, tLocal);
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
        guaranteed: pm.patch.boundIsGuaranteed,
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
      totalVertices: this.totalVertices(),
      totalTriangles: this.patches.reduce((s, pm) => s + pm.aliveTriangles().length, 0),
      welds: this.welds.map((w) => ({
        id: w.id, level: Math.max(w.level, w.forcedLevel),
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

module.exports = { Mesher, PatchMesh, Weld, edgeKey, uvKey, MAX_DEPTH };
