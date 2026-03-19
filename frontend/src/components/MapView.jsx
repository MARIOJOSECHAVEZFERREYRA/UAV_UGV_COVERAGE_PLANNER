import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import { fieldToGeoJSON, waypointsToGeoJSON, xyToLngLat, fieldBounds } from '../utils/geo'

const EMPTY_FC = { type: 'FeatureCollection', features: [] }

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

const VEHICLE_COLORS = { uav: '#FF5722', ugv: '#3F51B5' }

function dotsFC(points) {
  if (!points?.length) return EMPTY_FC
  return {
    type: 'Feature',
    geometry: { type: 'MultiPoint', coordinates: points.map(([x, y]) => xyToLngLat(x, y)) },
    properties: {},
  }
}

export default function MapView({ field, previewPoints, waypoints, vehicles, drawMode, onMapClick, onMapRightClick }) {
  const containerRef = useRef(null)
  const mapRef       = useRef(null)
  const markersRef   = useRef({})
  const [ready, setReady] = useState(false)

  // ── Init map ──────────────────────────────────────────────────────────────
  useEffect(() => {
    if (mapRef.current) return

    const map = new maplibregl.Map({
      container: containerRef.current,
      style: MAP_STYLE,
      center: [-63.182, -17.783],
      zoom: 16,
    })
    mapRef.current = map
    map.addControl(new maplibregl.NavigationControl(), 'top-right')
    map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right')

    map.on('load', () => {
      // ── Field polygon ──
      map.addSource('field', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'field-fill', type: 'fill', source: 'field',
        paint: { 'fill-color': '#38BDF8', 'fill-opacity': 0.08 },
      })
      map.addLayer({
        id: 'field-border', type: 'line', source: 'field',
        paint: { 'line-color': '#38BDF8', 'line-width': 2, 'line-opacity': 0.9 },
      })

      // ── Planned trajectory ──
      map.addSource('trajectory', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'trajectory', type: 'line', source: 'trajectory',
        paint: { 'line-color': '#FBBF24', 'line-width': 1.8, 'line-opacity': 0.9 },
      })

      // ── Draw preview fill (polygon in progress) ──
      map.addSource('draw-fill', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-fill', type: 'fill', source: 'draw-fill',
        paint: { 'fill-color': '#38BDF8', 'fill-opacity': 0.1 },
      })

      // ── Draw preview outline ──
      map.addSource('draw-lines', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-lines', type: 'line', source: 'draw-lines',
        paint: {
          'line-color': '#ffffff',
          'line-width': 1.5,
          'line-opacity': 0.75,
          'line-dasharray': [5, 3],
        },
      })

      // ── Draw preview dots ──
      map.addSource('draw-dots', { type: 'geojson', data: EMPTY_FC })
      map.addLayer({
        id: 'draw-dots', type: 'circle', source: 'draw-dots',
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
      Object.values(markersRef.current).forEach(m => m.remove())
      markersRef.current = {}
      map.remove()
      mapRef.current = null
    }
  }, [])

  // ── Click handlers ────────────────────────────────────────────────────────
  useEffect(() => {
    if (!ready) return
    const map = mapRef.current
    const handleClick      = (e) => onMapClick?.(e.lngLat.lng, e.lngLat.lat)
    const handleRightClick = (e) => { e.preventDefault(); onMapRightClick?.() }
    map.on('click', handleClick)
    map.on('contextmenu', handleRightClick)
    map.getCanvas().style.cursor = drawMode !== 'none' ? 'crosshair' : ''
    return () => { map.off('click', handleClick); map.off('contextmenu', handleRightClick) }
  }, [ready, drawMode, onMapClick, onMapRightClick])

  // ── Field polygon ─────────────────────────────────────────────────────────
  useEffect(() => {
    if (!ready) return
    const map = mapRef.current
    if (!field) { map.getSource('field').setData(EMPTY_FC); return }
    map.getSource('field').setData(fieldToGeoJSON(field))
    const [[minLng, minLat], [maxLng, maxLat]] = fieldBounds(field)
    map.fitBounds([[minLng, minLat], [maxLng, maxLat]], { padding: 60, duration: 800 })
  }, [ready, field])

  // ── Trajectory ────────────────────────────────────────────────────────────
  useEffect(() => {
    if (!ready) return
    mapRef.current.getSource('trajectory').setData(
      waypoints?.length ? waypointsToGeoJSON(waypoints) : EMPTY_FC
    )
  }, [ready, waypoints])

  // ── Draw preview ──────────────────────────────────────────────────────────
  useEffect(() => {
    if (!ready) return
    const map = mapRef.current
    const pts = previewPoints ?? []
    const coords = pts.map(([x, y]) => xyToLngLat(x, y))

    // Outline: close ring when >= 3 pts
    if (pts.length >= 2) {
      const lineCoords = pts.length >= 3 ? [...coords, coords[0]] : coords
      map.getSource('draw-lines').setData({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: lineCoords },
        properties: {},
      })
    } else {
      map.getSource('draw-lines').setData(EMPTY_FC)
    }

    // Fill polygon preview when >= 3 pts
    if (pts.length >= 3) {
      map.getSource('draw-fill').setData({
        type: 'Feature',
        geometry: { type: 'Polygon', coordinates: [[...coords, coords[0]]] },
        properties: {},
      })
    } else {
      map.getSource('draw-fill').setData(EMPTY_FC)
    }

    map.getSource('draw-dots').setData(pts.length > 0 ? dotsFC(pts) : EMPTY_FC)
  }, [ready, previewPoints])

  // ── Vehicle markers ───────────────────────────────────────────────────────
  useEffect(() => {
    if (!ready) return
    const active = new Set()
    for (const v of vehicles ?? []) {
      active.add(v.vehicle_id)
      const lnglat = xyToLngLat(v.x, v.y)
      if (markersRef.current[v.vehicle_id]) {
        markersRef.current[v.vehicle_id].setLngLat(lnglat)
      } else {
        const el = document.createElement('div')
        el.style.cssText = `
          width: 14px; height: 14px; border-radius: 50%;
          background: ${VEHICLE_COLORS[v.vehicle_id] ?? '#9E9E9E'};
          border: 2px solid #fff; box-shadow: 0 0 8px rgba(0,0,0,.6);
        `
        markersRef.current[v.vehicle_id] = new maplibregl.Marker({ element: el })
          .setLngLat(lnglat)
          .setPopup(new maplibregl.Popup({ offset: 16 }).setText(v.vehicle_id.toUpperCase()))
          .addTo(mapRef.current)
      }
    }
    for (const [id, marker] of Object.entries(markersRef.current)) {
      if (!active.has(id)) { marker.remove(); delete markersRef.current[id] }
    }
  }, [ready, vehicles])

  return <div ref={containerRef} style={{ width: '100%', height: '100%' }} />
}
