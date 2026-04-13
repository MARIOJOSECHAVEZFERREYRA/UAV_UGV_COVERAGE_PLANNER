import { MODE } from './modes.js'
import { CYCLE_PALETTE, DRAW } from './colors.js'

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
    sweepOpacity: highlight && highlight !== 'sweep' ? 0.12 : 0.92,
    ferryOpacity: highlight && highlight !== 'ferry' ? 0.12 : 0.85,
    deadheadOpacity: highlight && highlight !== 'deadhead' ? 0.12 : 0.75,
  }
}

export function getCycleColor(cycleIndex = 0) {
  return CYCLE_PALETTE[(cycleIndex ?? 0) % CYCLE_PALETTE.length]
}

export function getDrawColor(drawMode) {
  if (drawMode === MODE.DRAW_OBSTACLE) return DRAW.obstacle
  if (drawMode === MODE.DRAW_UGV_ROUTE) return DRAW.ugv
  return DRAW.field
}

export function getEdgeLabelGroups(field, previewPoints, previewIsOpen = false) {
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
      closed: previewIsOpen ? false : isClosedPreview(previewPoints),
    })
  }

  return groups
}

function getUniqueBasePoints(basePoints) {
  const seen = new Set()

  return (basePoints ?? []).filter(point => {
    const key = `${point.x.toFixed(1)},${point.y.toFixed(1)}`

    if (seen.has(key)) {
      return false
    }

    seen.add(key)
    return true
  })
}

export function getMissionMarkers(waypoints, basePoints) {
  const markers = []
  const firstWaypoint = waypoints?.find(waypoint => waypoint.waypoint_type !== 'base')

  if (firstWaypoint) {
    markers.push({
      key: 'marker-s',
      label: 'S',
      color: '#27ae60',
      point: { x: firstWaypoint.x, y: firstWaypoint.y },
    })
  }

  const uniqueBases = getUniqueBasePoints(basePoints)
  const isMobileMission = uniqueBases.length > 1

  uniqueBases.forEach((point, index) => {
    const isLast = index === uniqueBases.length - 1

    markers.push({
      key: `marker-base-${index}`,
      label: !isMobileMission ? 'Base' : isLast ? 'E' : `R${index + 1}`,
      color: isLast ? '#8b5cf6' : '#e67e22',
      point,
    })
  })

  return markers
}

export function toLineFeatureCollection(runs, pointToCoordinates, getColor = null) {
  if (!Array.isArray(runs) || runs.length === 0) {
    return EMPTY_FC
  }

  const features = runs
    .filter(run => {
      const pts = run.points ?? run
      return Array.isArray(pts) && pts.length >= 2
    })
    .map(run => {
      const pts = run.points ?? run
      const props = {}
      if (getColor) props.color = getColor(run)
      if (typeof run.cycleIndex === 'number') props.cycleIndex = run.cycleIndex
      return {
        type: 'Feature',
        geometry: {
          type: 'LineString',
          coordinates: pts.map(pointToCoordinates),
        },
        properties: props,
      }
    })

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
    features: points.map((point, index) => ({
      type: 'Feature',
      geometry: {
        type: 'Point',
        coordinates: pointToCoordinates(point),
      },
      properties: { index },
    })),
  }
}
