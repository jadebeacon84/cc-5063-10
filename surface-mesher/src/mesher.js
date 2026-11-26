'use strict';

// Compatibility adapter: legacy entry point for surface discretisation.
//
// The adaptive driver, boundary coordination and error estimation now live in
// discretization/.  This module preserves the historical import surface so
// existing surface-discretisation callers and samples keep running untouched:
//
//   const { Mesher, PatchMesh, Weld, edgeKey, uvKey, MAX_DEPTH } =
//     require('./mesher');
//
// The Mesher offered here is the refactored driver, constructed with the
// default (unified-cached) boundary strategy and the default analytic+sampled
// error metric, which is exactly the behaviour the old Mesher had.  Callers
// that want the new knobs use discretization directly.

const disc = require('./discretization/mesher');

module.exports = {
  Mesher: disc.Mesher,
  PatchMesh: disc.PatchMesh,
  Weld: disc.Weld,
  edgeKey: disc.edgeKey,
  uvKey: disc.uvKey,
  MAX_DEPTH: disc.MAX_DEPTH,
};
