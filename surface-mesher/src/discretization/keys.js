'use strict';

// Vertex / edge identity keys, shared by the mesher and the boundary
// coordinator so neither has to require the other.

const { ratKey } = require('../math');

function uvKey(uv) {
  return `${ratKey(uv[0])}|${ratKey(uv[1])}`;
}

function edgeKey(v1, v2) {
  const a = uvKey(v1.uv);
  const b = uvKey(v2.uv);
  return a < b ? `${a}#${b}` : `${b}#${a}`;
}

module.exports = { uvKey, edgeKey };
