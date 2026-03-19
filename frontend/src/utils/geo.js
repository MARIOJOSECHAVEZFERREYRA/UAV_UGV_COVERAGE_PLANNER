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
