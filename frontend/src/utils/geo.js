// Reference origin for flat x/y (meters) → WGS84 (lng/lat) conversion.
// Adjust REF_LAT/REF_LNG to match the real location of your fields.
const REF_LAT = -17.783327
const REF_LNG = -63.182140
const LAT_PER_METER = 1 / 111_000
const LNG_PER_METER = 1 / (111_000 * Math.cos((REF_LAT * Math.PI) / 180))

/** Convert flat [x, y] (meters) to GeoJSON [lng, lat]. */
export function xyToLngLat(x, y) {
  return [REF_LNG + x * LNG_PER_METER, REF_LAT + y * LAT_PER_METER]
}

/** Ensure a ring of [x,y] pairs is closed (first === last). Works in any coordinate space. */
export function ensureClosed(pts) {
  if (!pts.length) return pts
  const first = pts[0], last = pts[pts.length - 1]
  return (first[0] !== last[0] || first[1] !== last[1]) ? [...pts, pts[0]] : [...pts]
}

/** Build a GeoJSON FeatureCollection with one Polygon from a field object. */
export function fieldToGeoJSON(field) {
  const closeRing = (pts) => ensureClosed(pts.map(([x, y]) => xyToLngLat(x, y)))

  const coordinates = [
    closeRing(field.coordinates),
    ...(field.obstacles ?? []).map(closeRing),
  ]

  return {
    type: 'FeatureCollection',
    features: [{ type: 'Feature', geometry: { type: 'Polygon', coordinates }, properties: {} }],
  }
}

/** Build a GeoJSON FeatureCollection with one LineString from a waypoint array. */
export function waypointsToGeoJSON(waypoints) {
  const coordinates = waypoints.map((wp) => xyToLngLat(wp.x, wp.y))
  return {
    type: 'FeatureCollection',
    features: [{ type: 'Feature', geometry: { type: 'LineString', coordinates }, properties: {} }],
  }
}

/** Convert WGS84 [lng, lat] back to flat [x, y] meters. */
export function lngLatToXy(lng, lat) {
  return [
    (lng - REF_LNG) / LNG_PER_METER,
    (lat - REF_LAT) / LAT_PER_METER,
  ]
}

/**
 * Returns true if the segment (a→b) properly crosses segment (c→d).
 * "Properly" means they cross in their interiors — shared endpoints don't count.
 */
function segmentsCross(a, b, c, d) {
  const cross = (o, p, q) => (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
  const d1 = cross(c, d, a), d2 = cross(c, d, b)
  const d3 = cross(a, b, c), d4 = cross(a, b, d)
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
         ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
}

/**
 * Returns true if adding newPt to the polygon pts would create a self-intersection.
 *
 * Two checks:
 *  1. Forward edge  pts[n-1] → newPt  vs all non-adjacent existing edges
 *     (excludes pts[n-2]→pts[n-1] which shares vertex pts[n-1])
 *  2. Closing edge  newPt → pts[0]    vs all non-adjacent existing edges
 *     (excludes pts[0]→pts[1] adjacent to pts[0],
 *      and pts[n-2]→pts[n-1] adjacent to pts[n-1] which is adjacent to newPt via fwd edge)
 *     Only relevant when n ≥ 3 (triangles can never self-intersect on close).
 */
export function wouldSelfIntersect(pts, newPt) {
  const n = pts.length
  if (n < 2) return false

  // 1. Forward edge: pts[n-1] → newPt
  //    Check against existing edges except the adjacent one (pts[n-2]→pts[n-1])
  for (let i = 0; i < n - 1; i++) {
    if (segmentsCross(pts[n - 1], newPt, pts[i], pts[i + 1])) return true
  }

  // 2. Closing edge: newPt → pts[0]
  //    Check against existing edges that are non-adjacent to both endpoints:
  //    skip pts[0]→pts[1] (adjacent at pts[0]) and pts[n-2]→pts[n-1] (adjacent at pts[n-1])
  if (n >= 3) {
    for (let i = 1; i < n - 1; i++) {
      if (segmentsCross(newPt, pts[0], pts[i], pts[i + 1])) return true
    }
  }

  return false
}

/**
 * Unsigned area in m² for a flat [x,y] open ring (shoelace formula).
 */
export function polygonArea(pts) {
  let a = 0
  const n = pts.length
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n
    a += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
  }
  return Math.abs(a) / 2
}

/**
 * Ray-casting point-in-polygon test. pts = open ring [[x,y],...].
 * Returns true if [px,py] is strictly inside the polygon.
 */
export function pointInPolygon([px, py], poly) {
  let inside = false
  const n = poly.length
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const [xi, yi] = poly[i]
    const [xj, yj] = poly[j]
    if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) {
      inside = !inside
    }
  }
  return inside
}

/** Returns true if any edge of polyA properly crosses any edge of polyB (open rings). */
function polygonEdgesCross(polyA, polyB) {
  const na = polyA.length, nb = polyB.length
  for (let i = 0; i < na; i++) {
    for (let j = 0; j < nb; j++) {
      if (segmentsCross(polyA[i], polyA[(i + 1) % na], polyB[j], polyB[(j + 1) % nb])) return true
    }
  }
  return false
}

/**
 * Returns true if inner polygon is fully contained within outer polygon.
 * Fails if any edge of inner crosses outer, or any vertex of inner is outside outer.
 */
export function isPolygonInside(inner, outer) {
  if (polygonEdgesCross(inner, outer)) return false
  return inner.every(pt => pointInPolygon(pt, outer))
}

/**
 * Returns true if two polygons share any interior area:
 * crossing edges, one fully inside the other, or identical.
 */
export function polygonsOverlap(polyA, polyB) {
  if (polygonEdgesCross(polyA, polyB)) return true
  if (pointInPolygon(polyA[0], polyB)) return true
  if (pointInPolygon(polyB[0], polyA)) return true
  return false
}

/**
 * Split a flat waypoint array into typed runs for layered rendering.
 *
 * Returns:
 *   sweepRuns     — array of [{x,y}] arrays, one per contiguous spray run
 *   ferryRuns     — array of [{x,y}] arrays, one per contiguous ferry run (inside field)
 *   deadheadRuns  — array of [{x,y}] arrays, one per contiguous deadhead run (outside field)
 *   basePoints    — array of {x,y} for each base/recharge waypoint
 *
 * Runs share their endpoint with the adjacent run so lines connect visually.
 */
/**
 * Groups waypoints into typed runs. Each run is { points: [{x,y},...], cycleIndex: number }.
 * Sweep and deadhead runs carry their cycle index so the renderer can apply per-cycle colors.
 * Ferry runs also carry the cycle index but renderers may ignore it (ferry uses a fixed color).
 */
export function waypointsByType(waypoints) {
  const sweepRuns = []
  const ferryRuns = []
  const deadheadRuns = []
  const basePoints = []

  let sweepBuf = null
  let ferryBuf = null
  let deadheadBuf = null

  const flush = (buf, list, pt) => {
    if (!buf) return
    if (pt !== undefined) buf.points.push(pt)
    if (buf.points.length >= 1) list.push(buf)
  }

  for (const wp of waypoints ?? []) {
    const pt = { x: wp.x, y: wp.y }
    const ci = wp.cycle_index ?? 0

    if (wp.waypoint_type === 'base') {
      flush(sweepBuf, sweepRuns); sweepBuf = null
      flush(ferryBuf, ferryRuns); ferryBuf = null
      flush(deadheadBuf, deadheadRuns); deadheadBuf = null
      basePoints.push(pt)
    } else if (wp.waypoint_type === 'sweep') {
      flush(ferryBuf, ferryRuns, pt); ferryBuf = null
      flush(deadheadBuf, deadheadRuns, pt); deadheadBuf = null
      if (!sweepBuf) sweepBuf = { points: [pt], cycleIndex: ci }
      else sweepBuf.points.push(pt)
    } else if (wp.waypoint_type === 'ferry') {
      flush(sweepBuf, sweepRuns, pt); sweepBuf = null
      flush(deadheadBuf, deadheadRuns, pt); deadheadBuf = null
      if (!ferryBuf) ferryBuf = { points: [pt], cycleIndex: ci }
      else ferryBuf.points.push(pt)
    } else if (wp.waypoint_type === 'deadhead') {
      flush(sweepBuf, sweepRuns, pt); sweepBuf = null
      flush(ferryBuf, ferryRuns, pt); ferryBuf = null
      if (!deadheadBuf) deadheadBuf = { points: [pt], cycleIndex: ci }
      else deadheadBuf.points.push(pt)
    }
  }

  flush(sweepBuf, sweepRuns); sweepBuf = null
  flush(ferryBuf, ferryRuns); ferryBuf = null
  flush(deadheadBuf, deadheadRuns); deadheadBuf = null

  return { sweepRuns, ferryRuns, deadheadRuns, basePoints }
}

/** Build a GeoJSON FeatureCollection of obstacle polygons (for separate red rendering). */
export function obstacleToGeoJSON(field) {
  const obstacles = field?.obstacles ?? []
  return {
    type: 'FeatureCollection',
    features: obstacles
      .filter(obs => obs.length >= 3)
      .map(obs => ({
        type: 'Feature',
        geometry: {
          type: 'Polygon',
          coordinates: [ensureClosed(obs.map(([x, y]) => xyToLngLat(x, y)))],
        },
        properties: {},
      })),
  }
}

/**
 * Build a GeoJSON FeatureCollection with one closed LineString from a list of
 * flat [x, y] coords (safe-zone polygon outline from backend metrics).
 */
export function safeZoneToGeoJSON(coords) {
  if (!Array.isArray(coords) || coords.length < 3) {
    return { type: 'FeatureCollection', features: [] }
  }
  const ring = ensureClosed(coords.map(([x, y]) => xyToLngLat(x, y)))
  return {
    type: 'FeatureCollection',
    features: [{
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: ring },
      properties: {},
    }],
  }
}

/** Compute the bounding box [[minLng, minLat], [maxLng, maxLat]] of a field. */
export function fieldBounds(field) {
  const lnglats = field.coordinates.map(([x, y]) => xyToLngLat(x, y))
  const lngs = lnglats.map(([lng]) => lng)
  const lats = lnglats.map(([, lat]) => lat)
  return [
    [Math.min(...lngs), Math.min(...lats)],
    [Math.max(...lngs), Math.max(...lats)],
  ]
}
