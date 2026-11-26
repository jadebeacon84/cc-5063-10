'use strict';

// Preset scenes.  Every scene is a plain serialisable description so the
// browser UI can request it by name and override tolerances / cap / mode.

const {
  planeSurface, bumpSurface, spikeSurface, polarCapSurface,
  sphereBandSurface, halfBumpPatch, lipStrip,
} = require('./surfaces');

const D2R = Math.PI / 180;

function scenes() {
  return {
    // Single smooth patch; analytic (guaranteed) Hessian bound.
    plane: {
      name: 'plane',
      description: 'Perfectly flat plane: zero curvature must trigger no refinement beyond the seed.',
      build(params) {
        const tol = makeTol(params, 0.01, 10);
        return {
          patches: [planeSurface(tol)],
          welds: [],
          vertexCap: params.vertexCap || 5000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    bump: {
      name: 'bump',
      description: 'Gaussian bump with an analytic, guaranteed Hessian error bound.',
      build(params) {
        const tol = makeTol(params, 0.005, 8);
        return {
          patches: [bumpSurface(tol)],
          welds: [],
          vertexCap: params.vertexCap || 8000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    spike: {
      name: 'spike',
      description: 'Narrow Lorentzian spike: high-curvature local region with an analytic guaranteed bound; refinement concentrates at the peak.',
      build(params) {
        const tol = makeTol(params, 0.004, 10);
        return {
          patches: [spikeSurface(tol, 0.05, 0.3)],
          welds: [],
          vertexCap: params.vertexCap || 25000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    polar: {
      name: 'polar',
      description: 'Sphere polar cap with a repeated pole node: a fan seed gives one pole vertex and radial triangles, refined adaptively by curvature.',
      build(params) {
        const tol = makeTol(params, 0.01, 12);
        return {
          patches: [polarCapSurface(tol)],
          welds: [],
          fanSeed: true,
          vertexCap: params.vertexCap || 30000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    band: {
      name: 'band',
      description: 'Sphere band excluding the poles: a fully non-degenerate curved parameterisation with latitude-varying curvature and an analytic bound.',
      build(params) {
        const tol = makeTol(params, 0.01, 15);
        return {
          patches: [sphereBandSurface(tol)],
          welds: [],
          vertexCap: params.vertexCap || 20000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    // Two adjacent patches with different parameter densities and different
    // declared tolerances, sharing a welded edge.
    weldedPair: {
      name: 'weldedPair',
      description: 'Smooth half-patch welded to a high-curvature half-patch; the shared edge adopts the stricter tolerance and a single sampling identity.',
      build(params) {
        const smoothTol = makeTol(params, 0.02, 12, 'smooth');
        const tightTol = makeTol(params, 0.004, 6, 'tight');
        const left = halfBumpPatch('smooth-half', 0.0, smoothTol);
        const right = halfBumpPatch('curved-half', 0.5, tightTol);
        return {
          patches: [left, right],
          welds: [
            { id: 'seam-1', patchA: 'smooth-half', sideA: 'right', patchB: 'curved-half', sideB: 'left', flip: false },
          ],
          vertexCap: params.vertexCap || 10000,
          mode: params.mode || 'coarsen-remote',
          locks: params.locks || [],
        };
      },
    },

    // A tight-budget welded pair where the cap binds: coarsen-remote vs reject.
    budgetWeld: {
      name: 'budgetWeld',
      description: 'Welded pair under a hard, small vertex cap; far patch must release vertices or the export is refused.',
      build(params) {
        const smoothTol = makeTol(params, 0.02, 12, 'smooth');
        const tightTol = makeTol(params, 0.003, 6, 'tight');
        const left = halfBumpPatch('far-half', 0.0, smoothTol);
        const right = halfBumpPatch('near-half', 0.5, tightTol);
        return {
          patches: [left, right],
          welds: [
            { id: 'seam-budget', patchA: 'far-half', sideA: 'right', patchB: 'near-half', sideB: 'left', flip: false },
          ],
          vertexCap: params.vertexCap || 9000,
          mode: params.mode || 'coarsen-remote',
          locks: params.locks || ['near-half'],
        };
      },
    },

    // Deliberately separated lips: two strips with a physical z-gap, each
    // with its own tolerance, NO weld between them.
    lips: {
      name: 'lips',
      description: 'Two intentionally separated lip strips with a physical gap; independent vertices and independent tolerances, never welded.',
      build(params) {
        const tolA = makeTol(params, 0.01, 10, 'lipA');
        const tolB = makeTol(params, 0.05, 15, 'lipB');
        const upper = lipStrip('lip-upper', 0.03, tolA);
        const lower = lipStrip('lip-lower', -0.03, tolB);
        return {
          patches: [upper, lower],
          welds: [], // explicit: no shared sampling identity
          gap: 0.06,
          vertexCap: params.vertexCap || 4000,
          mode: params.mode || 'coarsen-remote',
          locks: [],
        };
      },
    },

    // Three-patch frame: two welded neighbours plus one separated lip.
    frame: {
      name: 'frame',
      description: 'Welded curved pair plus a detached lip strip: welds must stay crack-free while the lip keeps its own tolerance.',
      build(params) {
        const left = halfBumpPatch('frame-left', 0.0, makeTol(params, 0.01, 10, 'left'));
        const right = halfBumpPatch('frame-right', 0.5, makeTol(params, 0.004, 6, 'right'));
        const lip = lipStrip('frame-lip', 0.35, makeTol(params, 0.06, 12, 'lip'));
        return {
          patches: [left, right, lip],
          welds: [
            { id: 'frame-seam', patchA: 'frame-left', sideA: 'right', patchB: 'frame-right', sideB: 'left', flip: false },
          ],
          vertexCap: params.vertexCap || 30000,
          mode: params.mode || 'coarsen-remote',
          locks: params.locks || [],
        };
      },
    },
  };
}

function makeTol(params, defPos, defNormalDeg, key) {
  const posOverride = key && params.tolOverrides && params.tolOverrides[key]
    ? params.tolOverrides[key].position : undefined;
  const nrmOverride = key && params.tolOverrides && params.tolOverrides[key]
    ? params.tolOverrides[key].normal : undefined;
  return {
    position: posOverride ?? params.positionTol ?? defPos,
    normal: (nrmOverride ?? params.normalTolDeg ?? defNormalDeg) * D2R,
  };
}

function buildScene(name, params = {}) {
  const lib = scenes();
  const scene = lib[name];
  if (!scene) throw new Error(`unknown scene ${name}`);
  const built = scene.build(params);
  built.name = name;
  built.description = scene.description;
  if (params.errorMetric) built.errorMetric = params.errorMetric;
  if (params.boundaryStrategy) built.boundaryStrategy = params.boundaryStrategy;
  return built;
}

module.exports = { scenes, buildScene };
