import { useState, useMemo, useCallback } from 'react'
import MapView from './components/MapView.jsx'
import MissionPanel from './components/MissionPanel.jsx'
import { useTelemetry } from './hooks/useTelemetry.js'
import { lngLatToXy } from './utils/geo.js'
import { MODE } from './utils/modes.js'

export default function App() {
  const [mode, setMode]               = useState(MODE.NONE)
  const [activePts, setActivePts]     = useState([])   // in-progress polygon OR obstacle points
  const [drawnPolygon, setDrawnPolygon] = useState(null)  // committed field boundary [[x,y],...]
  const [obstacles, setObstacles]     = useState([])    // committed obstacles [[[x,y],...],...]
  const [waypoints, setWaypoints]     = useState([])
  const [activeMission, setActiveMission] = useState(null)

  const { vehicles, connected } = useTelemetry(activeMission?.id, activeMission?.status)

  const activeField = useMemo(() => {
    if (drawnPolygon?.length >= 3)
      return { coordinates: drawnPolygon, obstacles }
    return null
  }, [drawnPolygon, obstacles])

  const previewPoints = mode !== MODE.NONE ? activePts : []

  // ── Shared reset ──────────────────────────────────────────────────────────
  function resetDrawingAndMission() {
    setActivePts([])
    setMode(MODE.NONE)
    setWaypoints([])
    setActiveMission(null)
  }

  // ── Map interaction ───────────────────────────────────────────────────────
  const handleMapClick = useCallback((lng, lat) => {
    const [x, y] = lngLatToXy(lng, lat)
    if (mode !== MODE.NONE) setActivePts(pts => [...pts, [x, y]])
  }, [mode])

  const handleMapRightClick = useCallback(() => {
    if (mode !== MODE.NONE) setActivePts(pts => pts.slice(0, -1))
  }, [mode])

  // ── Draw modes ────────────────────────────────────────────────────────────
  function handleToggleDrawPolygon() {
    if (mode === MODE.DRAW_POLYGON) {
      if (activePts.length >= 3) setDrawnPolygon([...activePts])
      setActivePts([])
      setMode(MODE.NONE)
    } else {
      setMode(MODE.DRAW_POLYGON)
      setActivePts([])
    }
  }

  function handleToggleDrawObstacle() {
    if (mode === MODE.DRAW_OBSTACLE) {
      if (activePts.length >= 3) setObstacles(obs => [...obs, [...activePts]])
      setActivePts([])
      setMode(MODE.NONE)
    } else {
      setMode(MODE.DRAW_OBSTACLE)
      setActivePts([])
    }
  }

  // ── Field loading ─────────────────────────────────────────────────────────
  function handleLoadField(fieldData) {
    setDrawnPolygon(fieldData.boundary)
    setObstacles(fieldData.obstacles ?? [])
    resetDrawingAndMission()
  }

  function handleClear() {
    setDrawnPolygon(null)
    setObstacles([])
    resetDrawingAndMission()
  }

  function handleMissionReady(mission, wps) {
    setWaypoints(wps ?? [])
    setActiveMission(mission)
  }

  return (
    <div style={{ display: 'flex', height: '100vh', width: '100vw' }}>
      <div style={{ width: 360, minWidth: 360, borderRight: '1px solid #263238', zIndex: 10 }}>
        <MissionPanel
          mode={mode}
          activeField={activeField}
          drawingPtsCount={activePts.length}
          onToggleDrawPolygon={handleToggleDrawPolygon}
          onToggleDrawObstacle={handleToggleDrawObstacle}
          onLoadField={handleLoadField}
          onClear={handleClear}
          onMissionReady={handleMissionReady}
        />
      </div>

      <div style={{ flex: 1, position: 'relative' }}>
        <MapView
          field={activeField}
          previewPoints={previewPoints}
          waypoints={waypoints}
          vehicles={vehicles}
          drawMode={mode}
          onMapClick={handleMapClick}
          onMapRightClick={handleMapRightClick}
        />

        {activeMission && (
          <div style={{
            position: 'absolute', top: 12, left: 12, zIndex: 10,
            background: connected ? '#1b5e2088' : '#37474f88',
            backdropFilter: 'blur(4px)',
            border: `1px solid ${connected ? '#4CAF50' : '#607D8B'}`,
            borderRadius: 8, padding: '6px 14px', fontSize: 12, color: '#e8eaf6',
          }}>
            {connected ? '● LIVE — UAV & UGV streaming' : '○ Telemetry disconnected'}
          </div>
        )}

        {mode !== MODE.NONE && (
          <div style={{
            position: 'absolute', bottom: 24, left: '50%', transform: 'translateX(-50%)',
            background: '#0d1117cc', backdropFilter: 'blur(6px)',
            border: '1px solid #21262d', borderRadius: 8,
            padding: '8px 18px', fontSize: 13, color: '#58a6ff', zIndex: 10,
          }}>
            {mode === MODE.DRAW_POLYGON
              ? 'Click to add points  ·  Right-click to undo  ·  Press "Finish" when done'
              : 'Click to add obstacle points  ·  Right-click to undo  ·  Press "Finish Obstacle" when done'}
          </div>
        )}
      </div>
    </div>
  )
}
