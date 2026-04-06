import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import { TRAJ, DRAW, CYCLE_PALETTE } from '../utils/colors.js'
import {
  fieldToGeoJSON,
  obstacleToGeoJSON,
  safeZoneToGeoJSON,
  waypointsByType,
  xyToLngLat,
  fieldBounds,
} from '../utils/geo.js'
import { MODE } from '../utils/modes.js'
import {
  EMPTY_FC,
  formatDistanceMeters,
  getEdgeLabelGroups,
  getEdgeSegments,
  getPreviewLinePoints,
  getTrajectoryOpacity,
  toLineFeatureCollection,
  toPointFeatureCollection,
} from '../utils/viewScene.js'
import ZoomControls from './ZoomControls.jsx'
import VehicleOverlay from './VehicleOverlay.jsx'

const UGV_COLOR = DRAW.ugv

function makeRvMarkerEl(label, bg = '#e67e22') {
  const el = document.createElement('div')
  el.style.cssText = [
    'width:22px', 'height:22px',
    'border-radius:50%',
    `background:${bg}`,
    'border:2px solid #ffffff',
    'display:flex', 'align-items:center', 'justify-content:center',
    'font-size:9px', 'font-weight:700', 'color:#ffffff',
    'font-family:inherit',
    'pointer-events:none',
    'box-shadow:0 1px 4px rgba(0,0,0,.45)',
  ].join(';')
  el.textContent = label
  return el
}

function makeLabelEl(text) {
  const el = document.createElement('div')
  el.style.cssText = [
    'background:rgba(0,0,0,.55)',
    'color:#fff',
    'font-size:11px',
    'padding:1px 5px',
    'border-radius:3px',
    'white-space:nowrap',
    'pointer-events:none',
    'font-family:inherit',
    'line-height:1.4',
  ].join(';')
  el.textContent = text
  return el
}

const MAP_STYLE = {
  version: 8,
  sources: {
    satellite: {
      type: 'raster',
      tiles: ['https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'],
      tileSize: 256,
      attribution: '© Google',
    },
  },
  layers: [{ id: 'satellite', type: 'raster', source: 'satellite' }],
}

const arrayPointToLngLat = ([x, y]) => xyToLngLat(x, y)
const objectPointToLngLat = ({ x, y }) => xyToLngLat(x, y)

export default function MapView({
  field,
  previewPoints,
  waypoints,
  basePoint,
  ugvRoute,
  safeZone,
  vehicles,
  drawMode,
  highlight,
  onMapClick,
  onMapRightClick,
}) {
  const containerRef = useRef(null)
  const mapRef = useRef(null)
  const edgeLabelsRef = useRef([])
  const rvMarkersRef = useRef([])
  const [ready, setReady] = useState(false)
  const [hoveredCycle, setHoveredCycle] = useState(null)
  const [selectedCycle, setSelectedCycle] = useState(null)

  useEffect(() => {
    if (mapRef.current) {
      return
    }

    const map = new maplibregl.Map({
      container: containerRef.current,
      style: MAP_STYLE,
      center: [-63.182, -17.783],
      zoom: 16,
      minZoom: 3,
      dragRotate: false,
    })

    mapRef.current = map
    map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right')

    map.on('load', () => {
      map.addSource('field', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'field-fill',
        type: 'fill',
        source: 'field',
        paint: {
          'fill-color': DRAW.field,
          'fill-opacity': 0.08,
        },
      })
      map.addLayer({
        id: 'field-border',
        type: 'line',
        source: 'field',
        paint: {
          'line-color': DRAW.field,
          'line-width': 2,
          'line-opacity': 0.9,
        },
      })

      map.addSource('sweep-trajectory', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'sweep-trajectory',
        type: 'line',
        source: 'sweep-trajectory',
        paint: {
          'line-color': ['coalesce', ['get', 'color'], TRAJ.sweep],
          'line-width': 2,
          'line-opacity': 0.92,
        },
      })

      map.addSource('ferry-trajectory', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'ferry-trajectory',
        type: 'line',
        source: 'ferry-trajectory',
        paint: {
          'line-color': TRAJ.ferry,
          'line-width': 1.6,
          'line-opacity': 0.85,
          'line-dasharray': [4, 3],
        },
      })

      map.addSource('deadhead-trajectory', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'deadhead-trajectory',
        type: 'line',
        source: 'deadhead-trajectory',
        paint: {
          'line-color': ['coalesce', ['get', 'color'], TRAJ.deadhead],
          'line-width': 1.6,
          'line-opacity': 0.75,
          'line-dasharray': [2, 2],
        },
      })

      map.addSource('base-markers', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'base-markers',
        type: 'circle',
        source: 'base-markers',
        paint: {
          'circle-radius': 7,
          'circle-color': DRAW.obstacle,
          'circle-stroke-width': 2,
          'circle-stroke-color': '#fff',
        },
      })

      map.addSource('base-preview', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'base-preview',
        type: 'circle',
        source: 'base-preview',
        paint: {
          'circle-radius': 8,
          'circle-color': DRAW.obstacle,
          'circle-stroke-width': 2.5,
          'circle-stroke-color': '#fff',
          'circle-opacity': 0.9,
        },
      })

      map.addSource('draw-fill', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-fill',
        type: 'fill',
        source: 'draw-fill',
        paint: {
          'fill-color': DRAW.field,
          'fill-opacity': 0.1,
        },
      })

      map.addSource('draw-lines', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-lines',
        type: 'line',
        source: 'draw-lines',
        paint: {
          'line-color': '#ffffff',
          'line-width': 1.5,
          'line-opacity': 0.75,
          'line-dasharray': [5, 3],
        },
      })

      map.addSource('draw-dots', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-dots',
        type: 'circle',
        source: 'draw-dots',
        paint: {
          'circle-radius': 4,
          'circle-color': '#ffffff',
          'circle-stroke-width': 2,
          'circle-stroke-color': DRAW.field,
        },
      })

      // UGV route layers — committed route shown as orange dashed line + dots
      map.addSource('ugv-route', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'ugv-route',
        type: 'line',
        source: 'ugv-route',
        paint: {
          'line-color': UGV_COLOR,
          'line-width': 2.5,
          'line-opacity': 0.9,
          'line-dasharray': [6, 3],
        },
      })

      map.addSource('ugv-waypoints', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'ugv-waypoints',
        type: 'circle',
        source: 'ugv-waypoints',
        paint: {
          'circle-radius': 5,
          'circle-color': UGV_COLOR,
          'circle-stroke-width': 2,
          'circle-stroke-color': '#ffffff',
        },
      })

      // Obstacle layers — separate red fill/border (not just holes in field-fill)
      map.addSource('obstacles', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'obstacle-fill',
        type: 'fill',
        source: 'obstacles',
        paint: { 'fill-color': DRAW.obstacle, 'fill-opacity': 0.15 },
      })
      map.addLayer({
        id: 'obstacle-border',
        type: 'line',
        source: 'obstacles',
        paint: { 'line-color': DRAW.obstacle, 'line-width': 1.5, 'line-opacity': 0.9 },
      })

      // Safe zone boundary — dashed red outline showing spray margin
      map.addSource('safe-zone', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'safe-zone',
        type: 'line',
        source: 'safe-zone',
        paint: {
          'line-color': '#ef4444',
          'line-width': 1.5,
          'line-opacity': 0.65,
          'line-dasharray': [4, 3],
        },
      })

      setReady(true)
    })

    return () => {
      edgeLabelsRef.current.forEach(marker => marker.remove())
      edgeLabelsRef.current = []
      rvMarkersRef.current.forEach(marker => marker.remove())
      rvMarkersRef.current = []
      map.remove()
      mapRef.current = null
    }
  }, [])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current

    const handleClick = event => {
      onMapClick?.(event.lngLat.lng, event.lngLat.lat)
    }

    const handleRightClick = event => {
      event.preventDefault()
      onMapRightClick?.()
    }

    map.on('click', handleClick)
    map.on('contextmenu', handleRightClick)
    map.getCanvas().style.cursor = drawMode !== MODE.NONE ? 'crosshair' : ''

    return () => {
      map.off('click', handleClick)
      map.off('contextmenu', handleRightClick)
    }
  }, [ready, drawMode, onMapClick, onMapRightClick])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current

    if (!field) {
      map.getSource('field').setData(EMPTY_FC)
      map.getSource('obstacles').setData(EMPTY_FC)
      return
    }

    map.getSource('field').setData(fieldToGeoJSON(field))
    map.getSource('obstacles').setData(obstacleToGeoJSON(field))

    const [[minLng, minLat], [maxLng, maxLat]] = fieldBounds(field)
    map.fitBounds([[minLng, minLat], [maxLng, maxLat]], {
      padding: 60,
      duration: 800,
    })
  }, [ready, field])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current
    const { sweepRuns, ferryRuns, deadheadRuns, basePoints } = waypointsByType(waypoints)

    const cycleColor = run => CYCLE_PALETTE[(run.cycleIndex ?? 0) % CYCLE_PALETTE.length]

    map.getSource('sweep-trajectory').setData(
      toLineFeatureCollection(sweepRuns, objectPointToLngLat, cycleColor)
    )

    map.getSource('ferry-trajectory').setData(
      toLineFeatureCollection(ferryRuns, objectPointToLngLat)
    )

    map.getSource('deadhead-trajectory').setData(
      toLineFeatureCollection(deadheadRuns, objectPointToLngLat, cycleColor)
    )

    map.getSource('base-markers').setData(
      toPointFeatureCollection(basePoints, objectPointToLngLat)
    )

    // Mission markers: S (start), R1…Rn (rendezvous), E (end), Base (static)
    rvMarkersRef.current.forEach(m => m.remove())
    rvMarkersRef.current = []

    // "S" — first non-base waypoint = UAV start position
    const firstWp = waypoints?.find(wp => wp.waypoint_type !== 'base')
    if (firstWp) {
      const el = makeRvMarkerEl('S', '#27ae60')
      const [lng, lat] = xyToLngLat(firstWp.x, firstWp.y)
      rvMarkersRef.current.push(
        new maplibregl.Marker({ element: el, anchor: 'center' }).setLngLat([lng, lat]).addTo(map)
      )
    }

    // Deduplicate base points to find unique rendezvous / end positions.
    // Mobile missions have distinct locations; static collapse to one point.
    const seen = new Set()
    const uniqueBases = basePoints.filter(pt => {
      const key = `${pt.x.toFixed(1)},${pt.y.toFixed(1)}`
      if (seen.has(key)) return false
      seen.add(key)
      return true
    })

    const isMobile = uniqueBases.length > 1
    uniqueBases.forEach((pt, i) => {
      const isLast = i === uniqueBases.length - 1
      // Mobile: intermediate stops are R1, R2…; last position is E
      // Static: single fixed base labelled "Base"
      const label = !isMobile ? 'Base' : isLast ? 'E' : `R${i + 1}`
      const color = isLast ? '#8b5cf6' : '#e67e22'
      const el = makeRvMarkerEl(label, color)
      const [lng, lat] = xyToLngLat(pt.x, pt.y)
      rvMarkersRef.current.push(
        new maplibregl.Marker({ element: el, anchor: 'center' }).setLngLat([lng, lat]).addTo(map)
      )
    })
  }, [ready, waypoints])

  // Cycle hover / click interaction on trajectory layers
  useEffect(() => {
    if (!ready) return
    const map = mapRef.current
    const trajLayers = ['sweep-trajectory', 'deadhead-trajectory']

    const onEnter = (e) => {
      const ci = e.features?.[0]?.properties?.cycleIndex ?? null
      setHoveredCycle(ci)
      map.getCanvas().style.cursor = 'pointer'
    }
    const onLeave = () => {
      setHoveredCycle(null)
      map.getCanvas().style.cursor = ''
    }
    const onClickLayer = (e) => {
      const ci = e.features?.[0]?.properties?.cycleIndex ?? null
      setSelectedCycle(prev => prev === ci ? null : ci)
      e.originalEvent.stopPropagation()
    }
    const onClickMap = () => setSelectedCycle(null)

    for (const layer of trajLayers) {
      map.on('mousemove', layer, onEnter)
      map.on('mouseleave', layer, onLeave)
      map.on('click', layer, onClickLayer)
    }
    map.on('click', onClickMap)

    return () => {
      for (const layer of trajLayers) {
        map.off('mousemove', layer, onEnter)
        map.off('mouseleave', layer, onLeave)
        map.off('click', layer, onClickLayer)
      }
      map.off('click', onClickMap)
    }
  }, [ready])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current

    if (!basePoint) {
      map.getSource('base-preview').setData(EMPTY_FC)
      return
    }

    map.getSource('base-preview').setData(
      toPointFeatureCollection([basePoint], arrayPointToLngLat)
    )
  }, [ready, basePoint])

  useEffect(() => {
    if (!ready) return
    const data = highlight === 'ferry' ? safeZoneToGeoJSON(safeZone) : EMPTY_FC
    mapRef.current.getSource('safe-zone').setData(data)
  }, [ready, safeZone, highlight])

  useEffect(() => {
    if (!ready) return
    const map = mapRef.current
    const { sweepOpacity, ferryOpacity, deadheadOpacity } = getTrajectoryOpacity(highlight)
    const activeCycle = selectedCycle ?? hoveredCycle

    const cycleExpr = (base) => activeCycle !== null
      ? ['case', ['==', ['get', 'cycleIndex'], activeCycle], base, 0.08]
      : base

    map.setPaintProperty('sweep-trajectory', 'line-opacity', cycleExpr(sweepOpacity))
    map.setPaintProperty('ferry-trajectory', 'line-opacity', ferryOpacity)
    map.setPaintProperty('deadhead-trajectory', 'line-opacity', cycleExpr(deadheadOpacity))
  }, [ready, highlight, hoveredCycle, selectedCycle])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current
    const points = previewPoints ?? []
    const isUgvRoute = drawMode === MODE.DRAW_UGV_ROUTE

    if (isUgvRoute) {
      // UGV route preview: render directly on the committed orange layers so
      // the color is correct and nothing changes visually when Finish is clicked.
      if (points.length >= 2) {
        map.getSource('ugv-route').setData(
          toLineFeatureCollection([points], arrayPointToLngLat)
        )
        map.getSource('ugv-waypoints').setData(
          toPointFeatureCollection(points, arrayPointToLngLat)
        )
      } else {
        map.getSource('ugv-route').setData(EMPTY_FC)
        map.getSource('ugv-waypoints').setData(EMPTY_FC)
      }
      // Keep draw-* layers clear during UGV drawing
      map.getSource('draw-lines').setData(EMPTY_FC)
      map.getSource('draw-fill').setData(EMPTY_FC)
      map.getSource('draw-dots').setData(EMPTY_FC)
      return
    }

    // Normal polygon / obstacle drawing preview
    const previewLinePoints = getPreviewLinePoints(points)

    if (previewLinePoints.length >= 2) {
      map.getSource('draw-lines').setData(
        toLineFeatureCollection([previewLinePoints], arrayPointToLngLat)
      )
    } else {
      map.getSource('draw-lines').setData(EMPTY_FC)
    }

    if (points.length >= 3) {
      map.getSource('draw-fill').setData({
        type: 'Feature',
        geometry: {
          type: 'Polygon',
          coordinates: [[...points, points[0]].map(arrayPointToLngLat)],
        },
        properties: {},
      })
    } else {
      map.getSource('draw-fill').setData(EMPTY_FC)
    }

    map.getSource('draw-dots').setData(
      toPointFeatureCollection(points, arrayPointToLngLat)
    )
  }, [ready, previewPoints, drawMode])

  useEffect(() => {
    if (!ready) return
    // While drawing, the preview effect owns ugv-route/ugv-waypoints.
    // Only take over once drawing is finished.
    if (drawMode === MODE.DRAW_UGV_ROUTE) return

    const map = mapRef.current

    if (!ugvRoute || ugvRoute.length < 2) {
      map.getSource('ugv-route').setData(EMPTY_FC)
      map.getSource('ugv-waypoints').setData(EMPTY_FC)
      return
    }

    map.getSource('ugv-route').setData(
      toLineFeatureCollection([ugvRoute], arrayPointToLngLat)
    )
    map.getSource('ugv-waypoints').setData(
      toPointFeatureCollection(ugvRoute, arrayPointToLngLat)
    )
  }, [ready, ugvRoute, drawMode])

  useEffect(() => {
    if (!ready) {
      return
    }

    edgeLabelsRef.current.forEach(marker => marker.remove())
    edgeLabelsRef.current = []

    const groups = getEdgeLabelGroups(field, previewPoints, drawMode === MODE.DRAW_UGV_ROUTE)

    for (const group of groups) {
      for (const segment of getEdgeSegments(group.points, group.closed)) {
        const marker = new maplibregl.Marker({
          element: makeLabelEl(formatDistanceMeters(segment.length)),
          anchor: 'center',
        })
          .setLngLat(arrayPointToLngLat(segment.mid))
          .addTo(mapRef.current)

        edgeLabelsRef.current.push(marker)
      }
    }
  }, [ready, field, previewPoints, drawMode])

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <div ref={containerRef} style={{ width: '100%', height: '100%' }} />

      <VehicleOverlay
        renderer="map"
        vehicles={vehicles}
        mapRef={mapRef}
        ready={ready}
      />

      <ZoomControls
        onZoomIn={() => mapRef.current?.zoomIn()}
        onZoomOut={() => mapRef.current?.zoomOut()}
      />
    </div>
  )
}