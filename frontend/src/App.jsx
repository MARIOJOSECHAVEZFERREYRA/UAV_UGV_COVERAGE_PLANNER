import { useState, useMemo, useCallback, useEffect } from 'react'
import MapView from './components/MapView.jsx'
import PolygonCanvas from './components/PolygonCanvas.jsx'
import MissionPanel from './components/MissionPanel.jsx'
import { useTelemetry } from './hooks/useTelemetry.js'
import { lngLatToXy, wouldSelfIntersect } from './utils/geo.js'
import { MODE } from './utils/modes.js'

export default function App() {
  const [view, setView]               = useState('map')
  const [panelOpen, setPanelOpen]     = useState(true)
  const [mode, setMode]               = useState(MODE.NONE)
  const [activePts, setActivePts]     = useState([])
  const [drawnPolygon, setDrawnPolygon] = useState(null)
  const [obstacles, setObstacles]     = useState([])
  const [waypoints, setWaypoints]     = useState([])
  const [activeMission, setActiveMission] = useState(null)
  const [intersectionWarning, setIntersectionWarning] = useState(false)

  const { vehicles, connected } = useTelemetry(activeMission?.id, activeMission?.status)

  const activeField = useMemo(() => {
    if (drawnPolygon?.length >= 3)
      return { coordinates: drawnPolygon, obstacles }
    return null
  }, [drawnPolygon, obstacles])

  const previewPoints = mode !== MODE.NONE ? activePts : []

  function resetDrawingAndMission() {
    setActivePts([])
    setMode(MODE.NONE)
    setWaypoints([])
    setActiveMission(null)
  }

  useEffect(() => {
    if (!intersectionWarning) return
    const t = setTimeout(() => setIntersectionWarning(false), 2000)
    return () => clearTimeout(t)
  }, [intersectionWarning])

  const addPoint = useCallback((x, y) => {
    if (mode === MODE.NONE) return
    if (wouldSelfIntersect(activePts, [x, y])) {
      setIntersectionWarning(true)
      return
    }
    setActivePts(pts => [...pts, [x, y]])
  }, [mode, activePts])

  const undoPoint = useCallback(() => {
    if (mode !== MODE.NONE) setActivePts(pts => pts.slice(0, -1))
  }, [mode])

  const handleMapClick = useCallback((lng, lat) => {
    const [x, y] = lngLatToXy(lng, lat)
    addPoint(x, y)
  }, [addPoint])

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
      <div style={{ position: 'relative', flexShrink: 0, width: panelOpen ? 360 : 0, transition: 'width 0.25s ease', zIndex: 10 }}>
        {/* Clip wrapper: fills the outer div and hides overflowing content */}
        <div style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, overflow: 'hidden', borderRight: '1px solid #263238' }}>
          <div style={{ width: 360, height: '100%' }}>
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
        </div>
        {/* Toggle button: outside the clip wrapper so siempre es visible */}
        <button
          onClick={() => setPanelOpen(v => !v)}
          style={{
            position: 'absolute', top: '50%', right: -14,
            transform: 'translateY(-50%)',
            width: 14, height: 48, padding: 0,
            background: '#161b22', border: '1px solid #263238',
            borderLeft: 'none', borderRadius: '0 5px 5px 0',
            cursor: 'pointer', color: '#8b949e', fontSize: 10,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            zIndex: 20,
          }}
        >
          {panelOpen ? '\u2039' : '\u203a'}
        </button>
      </div>

      <div style={{ flex: 1, position: 'relative' }}>
        <ViewToggle view={view} onChangeView={setView} />

        {view === 'map' ? (
          <MapView
            field={activeField}
            previewPoints={previewPoints}
            waypoints={waypoints}
            vehicles={vehicles}
            drawMode={mode}
            onMapClick={handleMapClick}
            onMapRightClick={undoPoint}
          />
        ) : (
          <PolygonCanvas
            field={activeField}
            previewPoints={previewPoints}
            waypoints={waypoints}
            drawMode={mode}
            onCanvasClick={addPoint}
            onCanvasRightClick={undoPoint}
          />
        )}

        {activeMission && (
          <TelemetryBadge connected={connected} />
        )}

        {intersectionWarning && <IntersectionWarning />}

        {mode !== MODE.NONE && (
          <DrawingHint mode={mode} />
        )}
      </div>
    </div>
  )
}

const VIEW_TOGGLE_CONTAINER = {
  position: 'absolute', top: 12, left: 12, zIndex: 20,
  display: 'flex', borderRadius: 6, overflow: 'hidden',
  border: '1px solid #374151',
}

function ViewToggle({ view, onChangeView }) {
  return (
    <div style={VIEW_TOGGLE_CONTAINER}>
      {['map', 'canvas'].map(v => (
        <button
          key={v}
          onClick={() => onChangeView(v)}
          style={{
            padding: '5px 14px',
            fontSize: 12,
            fontFamily: 'inherit',
            cursor: 'pointer',
            border: 'none',
            background: view === v ? '#38BDF8' : '#1f2937',
            color: view === v ? '#0f172a' : '#94a3b8',
            fontWeight: view === v ? 600 : 400,
          }}
        >
          {v === 'map' ? 'Mapa' : 'Canvas'}
        </button>
      ))}
    </div>
  )
}

function TelemetryBadge({ connected }) {
  return (
    <div style={{
      position: 'absolute', bottom: 16, left: 16, zIndex: 10,
      background: connected ? '#1b5e2088' : '#37474f88',
      backdropFilter: 'blur(4px)',
      border: `1px solid ${connected ? '#4CAF50' : '#607D8B'}`,
      borderRadius: 8, padding: '6px 14px', fontSize: 12, color: '#e8eaf6',
    }}>
      {connected ? '● LIVE -- UAV & UGV streaming' : '○ Telemetry disconnected'}
    </div>
  )
}

function IntersectionWarning() {
  return (
    <div style={{
      position: 'absolute', top: 52, left: '50%', transform: 'translateX(-50%)',
      zIndex: 30, pointerEvents: 'none',
      background: '#3d0f0fcc', backdropFilter: 'blur(6px)',
      border: '1px solid #f85149', borderRadius: 8,
      padding: '8px 18px', fontSize: 13, color: '#ffa198',
      whiteSpace: 'nowrap',
    }}>
      Forma invalida: los vertices no pueden cruzarse
    </div>
  )
}

const DRAWING_HINT_STYLE = {
  position: 'absolute', bottom: 24, left: '50%', transform: 'translateX(-50%)',
  background: '#0d1117cc', backdropFilter: 'blur(6px)',
  border: '1px solid #21262d', borderRadius: 8,
  padding: '8px 18px', fontSize: 13, color: '#58a6ff', zIndex: 10,
}

function DrawingHint({ mode }) {
  const text = mode === MODE.DRAW_POLYGON
    ? 'Click to add points  ·  Right-click to undo  ·  Press "Finish" when done'
    : 'Click to add obstacle points  ·  Right-click to undo  ·  Press "Finish Obstacle" when done'

  return <div style={DRAWING_HINT_STYLE}>{text}</div>
}
