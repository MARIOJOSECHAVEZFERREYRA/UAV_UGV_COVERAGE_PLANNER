import { useState, useMemo, useCallback } from 'react'
import { C } from './utils/colors.js'
import MapView from './components/MapView.jsx'
import PolygonCanvas from './components/PolygonCanvas.jsx'
import MissionPanel from './components/MissionPanel.jsx'
import DroneSpecsView from './components/DroneSpecsView.jsx'
import ViewToggle from './components/ViewToggle.jsx'
import SimulationBadge from './components/SimulationBadge.jsx'
import IntersectionWarning from './components/IntersectionWarning.jsx'
import LayerToggle from './components/LayerToggle.jsx'
import DrawingHint from './components/DrawingHint.jsx'
import { useSimulation } from './hooks/useSimulation.js'
import { useMissionState } from './hooks/useMissionState.js'
import { useFieldEditor } from './hooks/useFieldEditor.js'
import { MODE } from './utils/modes.js'

export default function App() {
  const [view, setView] = useState('map')
  const [droneSpecsName, setDroneSpecsName] = useState(null)
  const [panelOpen, setPanelOpen] = useState(true)

  const [simEnabled, setSimEnabled] = useState(false)

  const {
    waypoints,
    activeMission,
    highlight,
    setHighlight,
    resetMission,
    dismissSimulation,
    handleMissionReady: _handleMissionReady,
  } = useMissionState()

  const handleMissionReady = useCallback((mission, wps) => {
    setSimEnabled(false)
    _handleMissionReady(mission, wps)
  }, [_handleMissionReady])

  const {
    mode,
    drawingPoints,
    activeField,
    basePoint,
    ugvRoute,
    intersectionWarning,
    addPoint,
    undoPoint,
    handleMapClick,
    handleToggleDrawPolygon,
    handleToggleDrawObstacle,
    handleToggleSetBasePoint,
    handleToggleDrawUgvRoute,
    handleLoadField,
    handleClear,
  } = useFieldEditor(resetMission)

  const {
    vehicles, connected, simTimeS,
    playbackSpeed, isPaused,
    setPlayback, pause, resume, restart, disconnect,
  } = useSimulation(activeMission?.id, activeMission?.status, simEnabled)

  const previewPoints = mode !== MODE.NONE ? drawingPoints : []

  const safeZone = useMemo(() => {
    if (!activeMission?.metrics_json) return null
    try {
      const m = JSON.parse(activeMission.metrics_json)
      return m._safe_polygon ?? null
    } catch { return null }
  }, [activeMission])

  const drawingLengthM = useMemo(() => {
    if (mode !== MODE.DRAW_UGV_ROUTE || drawingPoints.length < 2) return 0
    let total = 0
    for (let i = 0; i < drawingPoints.length - 1; i++) {
      const [x1, y1] = drawingPoints[i]
      const [x2, y2] = drawingPoints[i + 1]
      total += Math.hypot(x2 - x1, y2 - y1)
    }
    return total
  }, [mode, drawingPoints])

  return (
    <div style={{ display: 'flex', height: '100vh', width: '100vw' }}>
      <div
        style={{
          position: 'relative',
          flexShrink: 0,
          width: panelOpen ? 360 : 0,
          transition: 'width 0.25s ease',
          zIndex: 10,
        }}
      >
        <div
          style={{
            position: 'absolute',
            inset: 0,
            overflow: 'hidden',
            borderRight: `1px solid ${C.border}`,
          }}
        >
          <div style={{ width: 360, height: '100%' }}>
            <MissionPanel
              mode={mode}
              activeField={activeField}
              drawingPtsCount={drawingPoints.length}
              drawingLengthM={drawingLengthM}
              basePoint={basePoint}
              ugvRoute={ugvRoute}
              onToggleDrawPolygon={handleToggleDrawPolygon}
              onToggleDrawObstacle={handleToggleDrawObstacle}
              onToggleSetBasePoint={handleToggleSetBasePoint}
              onToggleDrawUgvRoute={handleToggleDrawUgvRoute}
              onLoadField={handleLoadField}
              onClear={handleClear}
              onMissionReady={handleMissionReady}
              onStartSim={() => setSimEnabled(true)}
              onStopSim={() => { disconnect(); setSimEnabled(false) }}
              simEnabled={simEnabled}
              onViewDroneSpecs={setDroneSpecsName}
            />
          </div>
        </div>

        <button
          type="button"
          onClick={() => setPanelOpen(value => !value)}
          aria-label={panelOpen ? 'Collapse sidebar' : 'Expand sidebar'}
          aria-expanded={panelOpen}
          title={panelOpen ? 'Collapse sidebar' : 'Expand sidebar'}
          style={{
            position: 'absolute',
            top: '50%',
            right: -14,
            transform: 'translateY(-50%)',
            width: 14,
            height: 48,
            padding: 0,
            background: C.surface,
            border: `1px solid ${C.border}`,
            borderLeft: 'none',
            borderRadius: '0 5px 5px 0',
            cursor: 'pointer',
            color: C.muted,
            fontSize: 10,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            zIndex: 20,
          }}
        >
          {panelOpen ? '\u2039' : '\u203a'}
        </button>
      </div>

      <div style={{ flex: 1, position: 'relative' }}>
        {droneSpecsName ? (
          <DroneSpecsView
            droneName={droneSpecsName}
            onBack={() => setDroneSpecsName(null)}
          />
        ) : (
          <>
            <ViewToggle view={view} onChangeView={setView} />

            {view === 'map' ? (
              <MapView
                field={activeField}
                previewPoints={previewPoints}
                waypoints={waypoints}
                basePoint={basePoint}
                ugvRoute={ugvRoute}
                vehicles={vehicles}
                drawMode={mode}
                highlight={highlight}
                safeZone={safeZone}
                onMapClick={handleMapClick}
                onMapRightClick={undoPoint}
              />
            ) : (
              <PolygonCanvas
                field={activeField}
                previewPoints={previewPoints}
                waypoints={waypoints}
                basePoint={basePoint}
                ugvRoute={ugvRoute}
                vehicles={vehicles}
                drawMode={mode}
                highlight={highlight}
                safeZone={safeZone}
                onCanvasClick={addPoint}
                onCanvasRightClick={undoPoint}
              />
            )}

            {waypoints?.length > 0 && (
              <LayerToggle
                highlight={highlight}
                onHighlight={setHighlight}
              />
            )}

            {activeMission && simEnabled && (
              <SimulationBadge
                connected={connected}
                simTimeS={simTimeS}
                playbackSpeed={playbackSpeed}
                isPaused={isPaused}
                onPlaybackChange={setPlayback}
                onPause={pause}
                onResume={resume}
                onRestart={restart}
                onDismiss={() => { disconnect(); setSimEnabled(false) }}
              />
            )}

            {intersectionWarning && <IntersectionWarning message={intersectionWarning} />}

            {mode !== MODE.NONE && <DrawingHint mode={mode} />}
          </>
        )}
      </div>
    </div>
  )
}