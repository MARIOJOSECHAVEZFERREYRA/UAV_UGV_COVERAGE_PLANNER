import { useState } from 'react'
import MapView from './components/MapView.jsx'
import PolygonCanvas from './components/PolygonCanvas.jsx'
import MissionPanel from './components/MissionPanel.jsx'
import DroneSpecsView from './components/DroneSpecsView.jsx'
import ViewToggle from './components/ViewToggle.jsx'
import TelemetryBadge from './components/TelemetryBadge.jsx'
import IntersectionWarning from './components/IntersectionWarning.jsx'
import LayerToggle from './components/LayerToggle.jsx'
import DrawingHint from './components/DrawingHint.jsx'
import { useTelemetry } from './hooks/useTelemetry.js'
import { useMissionState } from './hooks/useMissionState.js'
import { useFieldEditor } from './hooks/useFieldEditor.js'
import { MODE } from './utils/modes.js'

export default function App() {
  const [view, setView] = useState('map')
  const [droneSpecsName, setDroneSpecsName] = useState(null)
  const [panelOpen, setPanelOpen] = useState(true)

  const {
    waypoints,
    activeMission,
    highlight,
    setHighlight,
    resetMission,
    handleMissionReady,
  } = useMissionState()

  const {
    mode,
    drawingPoints,
    activeField,
    basePoint,
    intersectionWarning,
    addPoint,
    undoPoint,
    handleMapClick,
    handleToggleDrawPolygon,
    handleToggleDrawObstacle,
    handleToggleSetBasePoint,
    handleLoadField,
    handleClear,
  } = useFieldEditor(resetMission)

  const { vehicles, connected } = useTelemetry(
    activeMission?.id,
    activeMission?.status
  )

  const previewPoints = mode !== MODE.NONE ? drawingPoints : []

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
            borderRight: '1px solid #263238',
          }}
        >
          <div style={{ width: 360, height: '100%' }}>
            <MissionPanel
              mode={mode}
              activeField={activeField}
              drawingPtsCount={drawingPoints.length}
              basePoint={basePoint}
              onToggleDrawPolygon={handleToggleDrawPolygon}
              onToggleDrawObstacle={handleToggleDrawObstacle}
              onToggleSetBasePoint={handleToggleSetBasePoint}
              onLoadField={handleLoadField}
              onClear={handleClear}
              onMissionReady={handleMissionReady}
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
            background: '#161b22',
            border: '1px solid #263238',
            borderLeft: 'none',
            borderRadius: '0 5px 5px 0',
            cursor: 'pointer',
            color: '#8b949e',
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
                vehicles={vehicles}
                drawMode={mode}
                highlight={highlight}
                onMapClick={handleMapClick}
                onMapRightClick={undoPoint}
              />
            ) : (
              <PolygonCanvas
                field={activeField}
                previewPoints={previewPoints}
                waypoints={waypoints}
                basePoint={basePoint}
                vehicles={vehicles}
                drawMode={mode}
                highlight={highlight}
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

            {activeMission && (
              <TelemetryBadge connected={connected} />
            )}

            {intersectionWarning && <IntersectionWarning />}

            {mode !== MODE.NONE && <DrawingHint mode={mode} />}
          </>
        )}
      </div>
    </div>
  )
}