'use strict';

// Public interface for the surface-discretisation module.
//
// Layering (bottom-up):
//
//   sampling.js     surface sampling: probe layouts, element size, dense
//                   supersampling.  Geometry only.
//   estimation.js   local cell error: pluggable metric + replaceable audit
//                   backend.  Blind to seams and budgets.
//   boundary.js     shared-edge coordination: two boundary-subdivision
//                   strategies (unified-cached, negotiated-depth).  Owns
//                   crack constraints only.
//   mesher.js       refinement driver: consumes explicit estimates and
//                   delegates boundaries to the active strategy.
//   scheduler.js    interactive request scheduling: abandoned-request audit
//                   and stale-result commit gate.
//   evidence.js     execution evidence: independent grid error and shared-edge
//                   correspondence recomputed from the exported mesh.
//
// Local error estimation (estimation.js) is separated from cross-patch crack
// constraints (boundary.js).  The driver (mesher.js) is the only place that
// couples them, and only by consuming each through its stable interface.

const sampling = require('./sampling');
const estimation = require('./estimation');
const boundary = require('./boundary');
const mesher = require('./mesher');
const scheduler = require('./scheduler');
const evidence = require('./evidence');
const keys = require('./keys');

module.exports = {
  // Driver.
  Mesher: mesher.Mesher,
  PatchMesh: mesher.PatchMesh,
  Weld: mesher.Weld,
  MAX_DEPTH: mesher.MAX_DEPTH,
  MAX_RELAX: mesher.MAX_RELAX,

  // Sampling.
  sampling,

  // Estimation (metric + audit).
  estimation,
  evaluateTriangle: estimation.evaluateTriangle,
  evaluateSampledOnly: estimation.evaluateSampledOnly,
  resolveEstimator: estimation.resolveEstimator,
  createAuditBackend: estimation.createAuditBackend,

  // Boundary strategies.
  boundary,
  resolveStrategy: boundary.resolveStrategy,
  UnifiedCachedStrategy: boundary.UnifiedCachedStrategy,
  NegotiatedDepthStrategy: boundary.NegotiatedDepthStrategy,

  // Scheduling.
  RequestScheduler: scheduler.RequestScheduler,

  // Evidence.
  evidence,
  buildEvidence: evidence.buildEvidence,
  crossCheck: evidence.crossCheck,

  // Keys.
  keys,
};
