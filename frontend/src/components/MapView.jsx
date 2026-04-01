import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import {
  fieldToGeoJSON,
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
  vehicles,
  drawMode,
  highlight,
  onMapClick,
  onMapRightClick,
}) {
  const containerRef = useRef(null)
  const mapRef = useRef(null)
  const edgeLabelsRef = useRef([])
  const [ready, setReady] = useState(false)

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
          'fill-color': '#38BDF8',
          'fill-opacity': 0.08,
        },
      })
      map.addLayer({
        id: 'field-border',
        type: 'line',
        source: 'field',
        paint: {
          'line-color': '#38BDF8',
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
          'line-color': '#22c55e',
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
          'line-color': '#f59e0b',
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
          'line-color': '#ef4444',
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
          'circle-color': '#ef4444',
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
          'circle-color': '#ef4444',
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
          'fill-color': '#38BDF8',
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
          'circle-stroke-color': '#38BDF8',
        },
      })

      setReady(true)
    })

    return () => {
      edgeLabelsRef.current.forEach(marker => marker.remove())
      edgeLabelsRef.current = []
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
      return
    }

    map.getSource('field').setData(fieldToGeoJSON(field))

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

    map.getSource('sweep-trajectory').setData(
      toLineFeatureCollection(sweepRuns, objectPointToLngLat)
    )

    map.getSource('ferry-trajectory').setData(
      toLineFeatureCollection(ferryRuns, objectPointToLngLat)
    )

    map.getSource('deadhead-trajectory').setData(
      toLineFeatureCollection(deadheadRuns, objectPointToLngLat)
    )

    map.getSource('base-markers').setData(
      toPointFeatureCollection(basePoints, objectPointToLngLat)
    )
  }, [ready, waypoints])

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
    if (!ready) {
      return
    }

    const map = mapRef.current
    const { sweepOpacity, ferryOpacity, deadheadOpacity } = getTrajectoryOpacity(highlight)

    map.setPaintProperty('sweep-trajectory', 'line-opacity', sweepOpacity)
    map.setPaintProperty('ferry-trajectory', 'line-opacity', ferryOpacity)
    map.setPaintProperty('deadhead-trajectory', 'line-opacity', deadheadOpacity)
  }, [ready, highlight])

  useEffect(() => {
    if (!ready) {
      return
    }

    const map = mapRef.current
    const points = previewPoints ?? []
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
  }, [ready, previewPoints])

  useEffect(() => {
    if (!ready) {
      return
    }

    edgeLabelsRef.current.forEach(marker => marker.remove())
    edgeLabelsRef.current = []

    const groups = getEdgeLabelGroups(field, previewPoints)

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
  }, [ready, field, previewPoints])

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