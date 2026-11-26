'use strict';

// Request scheduling for interactive (dragging) re-meshing.
//
// During a drag the UI fires many mesh requests.  They complete out of order:
// a slower, older request can finish *after* a newer one.  Two guarantees are
// needed and this module enforces them:
//
//   1. Abandoned requests are recorded, not silently dropped.  Every request
//      that was superseded before it completed is kept in `abandoned` with its
//      generation and the reason it lost.
//
//   2. A stale result can never overwrite a newer seam.  Each request carries
//      a monotonically increasing generation; `commit()` refuses any result
//      whose generation is older than the newest committed generation.  So a
//      slow old mesher run cannot clobber the shared-edge state produced by a
//      fresher run, regardless of finishing order.
//
// The scheduler is engine-agnostic: it schedules opaque "jobs" that resolve
// to whatever payload the caller wants (here, a mesh+evidence payload).

class RequestScheduler {
  constructor(opts = {}) {
    this.onAbandoned = opts.onAbandoned || null;
    this.gen = 0;
    this.committedGen = -1;
    this.inflight = new Map(); // gen -> {label, startedAt, state}
    this.abandoned = [];       // superseded-before-completion records
    this.results = [];         // accepted commits, newest last
    this.rejectedStale = [];   // results refused because older than committed
    this.swaps = [];           // metric swaps recorded with generation
  }

  // Begin a new request: supersedes every still-inflight request.  Returns a
  // job descriptor whose gen must be passed to commit().
  begin(label) {
    const gen = ++this.gen;
    for (const [oldGen, job] of this.inflight) {
      job.state = 'abandoned';
      const rec = {
        gen: oldGen,
        label: job.label,
        supersededBy: gen,
        reason: 'superseded-by-newer-request',
        elapsedMs: Date.now() - job.startedAt,
      };
      this.abandoned.push(rec);
      this.inflight.delete(oldGen);
      if (this.onAbandoned) this.onAbandoned(rec);
    }
    const job = { gen, label, startedAt: Date.now(), state: 'pending' };
    this.inflight.set(gen, job);
    return job;
  }

  // Record a metric swap mid-flight.  The swap belongs to the generation that
  // is current when it is issued; requests already abandoned stay abandoned,
  // but the *active* request keeps its gen and will report the new metric.
  noteMetricSwap(job, from, to) {
    const rec = { gen: job.gen, from, to, at: Date.now() };
    this.swaps.push(rec);
    return rec;
  }

  // Commit a completed result.  Refused (and quarantined) if the result's
  // generation precedes the newest committed generation.  On success the
  // job stops being inflight.
  commit(job, payload) {
    const gen = job.gen;
    if (gen < this.committedGen) {
      const rec = {
        gen,
        committedGen: this.committedGen,
        reason: 'stale-result',
        payloadGeneration: payload && payload.generation,
      };
      this.rejectedStale.push(rec);
      return { accepted: false, reason: 'stale-result', record: rec };
    }
    this.committedGen = gen;
    this.inflight.delete(gen);
    const entry = { gen, label: job.label, payload, committedAt: Date.now() };
    this.results.push(entry);
    return { accepted: true, entry };
  }

  // A request errored out.  Frees its slot and records the failure so a slow
  // failing job cannot mask a newer commit.
  fail(job, error) {
    this.inflight.delete(job.gen);
    return { gen: job.gen, error: String(error && error.message || error) };
  }

  isCurrent(job) {
    return job.gen === this.gen;
  }

  report() {
    return {
      issued: this.gen,
      committedGen: this.committedGen,
      inflight: [...this.inflight.values()].map((j) => ({ gen: j.gen, label: j.label, state: j.state })),
      abandoned: this.abandoned.slice(),
      rejectedStale: this.rejectedStale.slice(),
      swaps: this.swaps.slice(),
      committed: this.results.map((r) => ({ gen: r.gen, label: r.label })),
    };
  }
}

module.exports = { RequestScheduler };
