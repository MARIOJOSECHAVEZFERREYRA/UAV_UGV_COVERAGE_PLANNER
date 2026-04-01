import { MODE } from './modes.js'

export const EMPTY_FC = {
  type: 'FeatureCollection',
  features: [],
}

export function formatDistanceMeters(distance) {
  return distance >= 100 ? `${distance.toFixed(0)}m` : `${distance.toFixed(1)}m`
}

export function getEdgeSegments(points, closed = true) {
  if (!Array.isArray(points) || points.length < 2) {
    return []
  }

  const count = closed ? points.length : points.length - 1

  return Array.from({ length: count }, (_, index) => {
    const p1 = points[index]
    const p2 = points[(index + 1) % points.length]

    return {
      index,
      p1,
      p2,
      mid: [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2],
      length: Math.hypot(p2[0] - p1[0], p2[1] - p1[1]),
    }
  })
}

export function isClosedPreview(points) {
  return Array.isArray(points) && points.length >= 3
}

export function getPreviewLinePoints(points) {
  if (!Array.isArray(points) || points.length < 2) {
    return []
  }

  return isClosedPreview(points) ? [...points, points[0]] : [...points]
}

export function getTrajectoryOpacity(highlight) {
  return {
    sweepOpacity: highlight === 'ferry' ? 0.12 : 0.92,
    ferryOpacity: highlight === 'sweep' ? 0.12 : 0.85,
  }
}

export function getDrawColor(drawMode) {
  return drawMode === MODE.DRAW_OBSTACLE ? '#ef4444' : '#38BDF8'
}

export function getEdgeLabelGroups(field, previewPoints) {
  const groups = []

  if (field?.coordinates?.length >= 2) {
    groups.push({ points: field.coordinates, closed: true })
  }

  for (const obstacle of field?.obstacles ?? []) {
    if (obstacle?.length >= 2) {
      groups.push({ points: obstacle, closed: true })
    }
  }

  if (previewPoints?.length >= 2) {
    groups.push({
      points: previewPoints,
      closed: isClosedPreview(previewPoints),
    })
  }

  return groups
}

export function toLineFeatureCollection(runs, pointToCoordinates) {
  if (!Array.isArray(runs) || runs.length === 0) {
    return EMPTY_FC
  }

  const features = runs
    .filter(run => Array.isArray(run) && run.length >= 2)
    .map(run => ({
      type: 'Feature',
      geometry: {
        type: 'LineString',
        coordinates: run.map(pointToCoordinates),
      },
      properties: {},
    }))

  return {
    type: 'FeatureCollection',
    features,
  }
}

export function toPointFeatureCollection(points, pointToCoordinates) {
  if (!Array.isArray(points) || points.length === 0) {
    return EMPTY_FC
  }

  return {
    type: 'FeatureCollection',
    features: points.map(point => ({
      type: 'Feature',
      geometry: {
        type: 'Point',
        coordinates: pointToCoordinates(point),
      },
      properties: {},
    })),
  }
}