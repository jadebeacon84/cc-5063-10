'use strict';

// Compatibility adapter: surface-discretisation legacy entry point for error
// estimation.
//
// The estimation logic now lives in discretization/estimation.js.  This file
// preserves the historical module surface (evaluateTriangle and the bound
// helpers) so existing callers keep working unchanged, including the exact
// numeric behaviour: it re-exports the same functions, not copies.

const estimation = require('./discretization/estimation');

module.exports = {
  evaluateTriangle: estimation.evaluateTriangle,
  guaranteedPositionBound: estimation.guaranteedPositionBound,
  guaranteedNormalBound: estimation.guaranteedNormalBound,
  sampledErrors: estimation.sampledErrors,
  maxParamEdge: estimation.maxParamEdge,
};
