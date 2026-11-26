'use strict';

// HTTP backend for the surface discretisation editor.  Pure Node standard
// library; every mesh is produced by the real Mesher, never canned data.

const http = require('http');
const fs = require('fs');
const path = require('path');
const { URL } = require('url');
const { buildScene, scenes } = require('./scenes');
const { Mesher } = require('./discretization');
const { RequestScheduler } = require('./discretization');
const { buildEvidence, crossCheck } = require('./discretization/evidence');
const { resolveEstimator } = require('./discretization/estimation');
const { extractMesh, seamVertexIds, verify, writeOBJ, writeSTL } = require('./export');

function readBody(req) {
  return new Promise((resolve, reject) => {
    let data = '';
    req.on('data', (c) => {
      data += c;
      if (data.length > 5e6) reject(new Error('body too large'));
    });
    req.on('end', () => {
      try {
        resolve(data ? JSON.parse(data) : {});
      } catch (e) {
        reject(e);
      }
    });
    req.on('error', reject);
  });
}

// Serialise enough geometry for the three.js viewer plus full metrics and
// independent execution evidence (grid error + shared-edge correspondence).
function serialiseMesh(mesher, result) {
  const mesh = extractMesh(mesher);
  const verification = verify(mesher, mesh, { gridN: 18 });
  const evidence = buildEvidence(mesher, mesh, verification);
  return {
    summary: {
      rejected: result.rejected,
      rejectReason: result.rejectReason,
      vertexCap: result.vertexCap,
      mode: result.mode,
      boundaryStrategy: result.boundaryStrategy,
      errorMetric: result.errorMetric,
      totalVertices: result.totalVertices,
      globalVertices: mesh.vertices.length,
      totalTriangles: result.totalTriangles,
    },
    welds: result.welds,
    events: result.events,
    metricSwaps: result.metricSwaps,
    patches: result.patches,
    verification,
    evidence,
    geometry: {
      positions: flatten(mesh.vertices),
      faces: mesh.faces,
      facePatches: mesh.facePatches,
      faceUnsatisfied: mesh.faceUnsatisfied,
      seamVertexIds: seamVertexIds(mesher, mesh),
      patchNames: mesher.patches.map((pm) => pm.patch.name),
    },
  };
}

function flatten(vertices) {
  const out = [];
  for (const v of vertices) out.push(v[0], v[1], v[2]);
  return out;
}

function handleMesh(params) {
  const scene = buildScene(params.scene || 'bump', params);
  const mesher = new Mesher(scene, {
    estimator: params.errorMetric,
    boundaryStrategy: params.boundaryStrategy,
  });
  const result = mesher.run();
  return { payload: serialiseMesh(mesher, result), mesher, result, scene };
}

// Simulated interactive drag: several mesh requests are issued while the
// operator changes the error metric mid-flight.  Requests complete out of
// order so a slow old run resolves after a newer one.  The scheduler records
// abandoned requests and refuses the stale result, proving the old seam never
// overwrites the new one.
function handleDragSwap(params) {
  const scheduler = new RequestScheduler();
  const jobs = (params.requests && params.requests.length)
    ? params.requests
    : [
      // Oldest request is also the slowest, so it finishes last.
      { label: 'drag-1', scene: params.scene || 'weldedPair', metric: 'analytic-sampled', delay: 160 },
      { label: 'drag-2', scene: params.scene || 'weldedPair', metric: 'sampled-only', delay: 70 },
      { label: 'drag-3', scene: params.scene || 'weldedPair', metric: 'analytic-sampled', delay: 10 },
    ];

  // Issue all requests first (later ones supersede earlier ones), compute each
  // result synchronously, then commit on a common clock so completion order is
  // governed by `delay` rather than by compute time.
  const issued = jobs.map((jobSpec, idx) => {
    const job = scheduler.begin(jobSpec.label);
    scheduler.noteMetricSwap(job, idx === 0 ? null : jobs[idx - 1].metric, jobSpec.metric);
    const scene = buildScene(jobSpec.scene || params.scene || 'weldedPair', {
      errorMetric: jobSpec.metric,
      boundaryStrategy: params.boundaryStrategy,
    });
    const mesher = new Mesher(scene);
    const result = mesher.run();
    const mesh = extractMesh(mesher);
    const verification = verify(mesher, mesh, { gridN: 10 });
    const evidence = buildEvidence(mesher, mesh, verification);
    const payload = {
      generation: job.gen,
      label: jobSpec.label,
      metric: result.errorMetric,
      triangles: result.totalTriangles,
      seamCrackFree: evidence.summary.allSeamsFree,
    };
    return { jobSpec, job, payload };
  });

  return new Promise((resolve) => {
    const completions = [];
    let remaining = issued.length;
    for (const { jobSpec, job, payload } of issued) {
      setTimeout(() => {
        const commit = scheduler.commit(job, payload);
        completions.push({
          label: jobSpec.label, gen: job.gen, metric: jobSpec.metric,
          accepted: commit.accepted, reason: commit.reason || null,
        });
        if (--remaining === 0) {
          resolve({ scheduler: scheduler.report(), completions });
        }
      }, jobSpec.delay || 0);
    }
  });
}

function createServer(outDir) {
  const MIME = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
  };

  const server = http.createServer(async (req, res) => {
    const url = new URL(req.url, `http://${req.headers.host}`);
    try {
      if (req.method === 'GET' && url.pathname === '/api/scenes') {
        const lib = scenes();
        const list = Object.values(lib).map((s) => ({ name: s.name, description: s.description }));
        return json(res, 200, {
          scenes: list,
          errorMetrics: Object.keys(require('./discretization/estimation').estimators),
          boundaryStrategies: Object.keys(require('./discretization/boundary').strategies),
        });
      }

      if (req.method === 'POST' && url.pathname === '/api/mesh') {
        const params = await readBody(req);
        const { payload } = handleMesh(params);
        return json(res, 200, payload);
      }

      if (req.method === 'POST' && url.pathname === '/api/evidence') {
        const params = await readBody(req);
        const scene = buildScene(params.scene || 'bump', params);
        const mesher = new Mesher(scene, {
          estimator: params.errorMetric,
          boundaryStrategy: params.boundaryStrategy,
        });
        const result = mesher.run();
        const check = crossCheck(mesher, result, { gridN: params.gridN || 24 });
        return json(res, 200, {
          summary: {
            rejected: result.rejected,
            errorMetric: result.errorMetric,
            boundaryStrategy: result.boundaryStrategy,
          },
          evidence: check.evidence,
          ok: check.ok,
          problems: check.problems,
        });
      }

      if (req.method === 'POST' && url.pathname === '/api/drag-swap') {
        const params = await readBody(req);
        const report = await handleDragSwap(params);
        return json(res, 200, report);
      }

      if (req.method === 'POST' && url.pathname === '/api/export') {
        const params = await readBody(req);
        const format = (params.format || 'obj').toLowerCase();
        const { mesher, result, scene } = handleMesh(params);
        if (result.rejected) {
          return json(res, 409, {
            error: 'export_refused',
            reason: result.rejectReason,
            unsatisfied: result.patches.map((p) => ({ name: p.name, count: p.unsatisfied.length })),
          });
        }
        const mesh = extractMesh(mesher);
        fs.mkdirSync(outDir, { recursive: true });
        const base = `${scene.name}-${Date.now()}`;
        const file = format === 'stl'
          ? writeSTL(mesh, path.join(outDir, `${base}.stl`), scene.name)
          : writeOBJ(mesh, path.join(outDir, `${base}.obj`), scene.name);
        const stat = fs.statSync(file);
        return json(res, 200, {
          file: path.basename(file),
          format,
          bytes: stat.size,
          vertices: mesh.vertices.length,
          triangles: mesh.faces.length,
          downloadUrl: `/download/${path.basename(file)}`,
        });
      }

      if (req.method === 'GET' && url.pathname.startsWith('/download/')) {
        const name = path.basename(url.pathname);
        const file = path.join(outDir, name);
        if (!fs.existsSync(file)) return json(res, 404, { error: 'not found' });
        res.writeHead(200, {
          'Content-Type': 'application/octet-stream',
          'Content-Disposition': `attachment; filename="${name}"`,
        });
        return fs.createReadStream(file).pipe(res);
      }

      // Static files.
      let rel = url.pathname === '/' ? '/index.html' : url.pathname;
      const file = path.join(__dirname, '..', 'web', path.normalize(rel).replace(/^(\.\.[/\\])+/, ''));
      if (fs.existsSync(file) && fs.statSync(file).isFile()) {
        res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream' });
        return fs.createReadStream(file).pipe(res);
      }
      // three.js from the local npm install (offline-friendly).
      if (rel.startsWith('/vendor/three')) {
        const threePath = path.join(__dirname, '..', 'node_modules', 'three', 'build', 'three.module.js');
        res.writeHead(200, { 'Content-Type': 'text/javascript; charset=utf-8' });
        return fs.createReadStream(threePath).pipe(res);
      }
      return json(res, 404, { error: 'not found', path: rel });
    } catch (err) {
      return json(res, 500, { error: String(err && err.message || err) });
    }
  });

  return server;
}

function json(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8' });
  res.end(body);
}

if (require.main === module) {
  const port = Number(process.env.PORT || 8080);
  const outDir = path.join(__dirname, '..', 'output');
  createServer(outDir).listen(port, () => {
    console.log(`Surface discretisation editor: http://localhost:${port}`);
    console.log(`Exported mesh files are written to ${outDir}`);
  });
}

module.exports = { createServer, serialiseMesh };
