import { useEffect, useRef, useState } from 'react'
import { ensureClosed } from '../utils/geo'
import { MODE } from '../utils/modes'

const C = {
  bg:       '#0d1117',
  surface:  '#161b22',
  border:   '#21262d',
  accent:   '#58a6ff',
  accentDim:'#1f6feb',
  text:     '#e6edf3',
  muted:    '#8b949e',
  success:  '#3fb950',
  warning:  '#d29922',
  error:    '#f85149',
  danger:   '#da3633',
}

const STATUS_COLOR = {
  pending:   C.warning,
  running:   C.accent,
  completed: C.success,
  failed:    C.error,
}

const s = {
  panel: {
    display: 'flex', flexDirection: 'column', gap: 0,
    background: C.bg, height: '100%', overflowY: 'auto',
    fontFamily: "'Inter', 'Segoe UI', system-ui, sans-serif",
  },
  header: {
    padding: '18px 16px 14px',
    borderBottom: `1px solid ${C.border}`,
  },
  logo: { display: 'flex', alignItems: 'center', gap: 8, marginBottom: 2 },
  logoIcon: { fontSize: 18 },
  logoText: { fontSize: 15, fontWeight: 700, color: C.text, letterSpacing: 0.3 },
  subtitle: { fontSize: 11, color: C.muted, letterSpacing: 0.2 },
  section: {
    padding: '14px 16px',
    borderBottom: `1px solid ${C.border}`,
    display: 'flex', flexDirection: 'column', gap: 10,
  },
  sectionLabel: {
    fontSize: 10, fontWeight: 700, color: C.muted,
    letterSpacing: 1.8, textTransform: 'uppercase',
  },
  label: { fontSize: 11, color: C.muted, marginBottom: 3 },
  input: {
    width: '100%', padding: '7px 10px', boxSizing: 'border-box',
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 6, color: C.text, fontSize: 12,
    outline: 'none',
  },
  select: {
    width: '100%', padding: '7px 10px',
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 6, color: C.text, fontSize: 12,
    outline: 'none',
  },
  btnRow: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 },
  stat: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: 12 },
  statKey: { color: C.muted },
  statVal: { color: C.text, fontWeight: 600, fontVariantNumeric: 'tabular-nums' },
  badge: {
    display: 'inline-flex', alignItems: 'center', gap: 5,
    padding: '2px 9px', borderRadius: 20, fontSize: 11, fontWeight: 600,
  },
  errorBox: {
    background: '#f8514912', border: `1px solid ${C.error}`,
    borderRadius: 6, padding: '8px 10px', fontSize: 11, color: '#ffa198',
  },
}

function Btn({ children, onClick, variant = 'default', active = false, disabled = false, fullWidth = true }) {
  const variants = {
    default:  { bg: C.surface,   border: C.border,    color: C.text },
    primary:  { bg: C.accentDim, border: C.accent,    color: '#ffffff' },
    success:  { bg: '#1a3a1a',   border: '#238636',   color: C.success },
    danger:   { bg: '#3d0f0f',   border: C.danger,    color: '#ff7b72' },
    active:   { bg: '#1f4e79',   border: C.accent,    color: C.accent },
    warning:  { bg: '#3d2a00',   border: '#bb8009',   color: '#e3b341' },
  }
  const v = active ? variants.active : variants[variant]
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      style={{
        display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 6,
        padding: '8px 12px', border: `1px solid ${v.border}`, borderRadius: 6,
        background: v.bg, color: v.color,
        fontSize: 12, fontWeight: 500, cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.45 : 1, width: fullWidth ? '100%' : 'auto',
        transition: 'opacity 0.15s, background 0.15s',
        fontFamily: 'inherit',
      }}
    >
      {children}
    </button>
  )
}

function Divider() {
  return <div style={{ height: 1, background: C.border }} />
}

function Spinner() {
  return (
    <span style={{
      display: 'inline-block', width: 11, height: 11,
      border: '2px solid #ffffff40', borderTopColor: '#ffffff',
      borderRadius: '50%', animation: 'spin 0.7s linear infinite',
    }} />
  )
}

function HeaderSection() {
  return (
    <div style={s.header}>
      <div style={s.logo}>
        <span style={s.logoText}>UAV-UAG Planner</span>
      </div>
      <div style={s.subtitle}>UAV-UAG Mission Planning Tool</div>
    </div>
  )
}

function AircraftSection({ drones, drone, onDroneChange }) {
  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Aircraft</div>
      <div>
        <div style={s.label}>Drone model</div>
        <select style={s.select} value={drone} onChange={e => onDroneChange(e.target.value)}>
          {drones.map(d => <option key={d.name} value={d.name}>{d.name}</option>)}
        </select>
      </div>
    </div>
  )
}

function FieldSection({
  mode, activeField, drawingPtsCount,
  onToggleDrawPolygon, onToggleDrawObstacle,
  onLoadField, onClear,
}) {
  const fileRef = useRef(null)
  const isDrawingPolygon  = mode === MODE.DRAW_POLYGON
  const isDrawingObstacle = mode === MODE.DRAW_OBSTACLE
  const hasField          = !!activeField

  function handleLoadJSON(e) {
    const file = e.target.files[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = (ev) => {
      try { onLoadField(JSON.parse(ev.target.result)) }
      catch { alert('Invalid JSON file') }
    }
    reader.readAsText(file)
    e.target.value = ''
  }

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Field</div>

      <div style={s.btnRow}>
        <Btn
          variant={isDrawingPolygon ? 'active' : 'success'}
          active={isDrawingPolygon}
          onClick={onToggleDrawPolygon}
        >
          {isDrawingPolygon
            ? `Finish (${drawingPtsCount} pts)`
            : 'Draw Field'}
        </Btn>
        <Btn
          variant={isDrawingObstacle ? 'warning' : 'default'}
          active={isDrawingObstacle}
          disabled={!hasField && !isDrawingObstacle}
          onClick={onToggleDrawObstacle}
        >
          {isDrawingObstacle ? 'Finish Obstacle' : 'Add Obstacle'}
        </Btn>
      </div>

      <div style={s.btnRow}>
        <Btn variant='default' onClick={() => fileRef.current.click()}>
          Load JSON
        </Btn>
        <Btn variant='danger' disabled={!hasField && mode === MODE.NONE} onClick={onClear}>
          Clear All
        </Btn>
      </div>

      <input ref={fileRef} type="file" accept=".json" style={{ display: 'none' }} onChange={handleLoadJSON} />

      {activeField?.obstacles?.length > 0 && (
        <div style={{ fontSize: 11, color: C.muted, display: 'flex', alignItems: 'center', gap: 5 }}>
          <span style={{ color: '#e36d2e' }}>+</span>
          {activeField.obstacles.length} obstacle{activeField.obstacles.length > 1 ? 's' : ''} defined
        </div>
      )}
    </div>
  )
}

function ParametersSection({ missionName, setName, sprayWidth, setSprayWidth, swathRange, strategy, setStrategy }) {
  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Mission Parameters</div>

      <div>
        <div style={s.label}>Mission name</div>
        <input style={s.input} value={missionName} onChange={e => setName(e.target.value)} />
      </div>

      <div>
        <div style={{ ...s.label, display: 'flex', justifyContent: 'space-between' }}>
          <span>Spray width (m)</span>
          <span style={{ color: C.muted }}>
            {swathRange.min} – {swathRange.max} m
          </span>
        </div>
        <input
          style={s.input} type="number"
          min={swathRange.min} max={swathRange.max} step={0.5}
          value={sprayWidth} onChange={e => setSprayWidth(e.target.value)}
        />
      </div>

      <div>
        <div style={s.label}>Planning strategy</div>
        <select style={s.select} value={strategy} onChange={e => setStrategy(e.target.value)}>
          <option value="genetic">Genetic Algorithm (GA)</option>
          <option value="simple">Simple Grid</option>
        </select>
      </div>
    </div>
  )
}

function ResultsSection({ mission, onExport }) {
  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Results</div>

      <div style={s.stat}>
        <span style={s.statKey}>Status</span>
        <span style={{
          ...s.badge,
          background: STATUS_COLOR[mission.status] + '20',
          color: STATUS_COLOR[mission.status],
          border: `1px solid ${STATUS_COLOR[mission.status]}40`,
        }}>
          {mission.status}
        </span>
      </div>

      {mission.status === 'completed' && <>
        <Divider />
        <div style={s.stat}>
          <span style={s.statKey}>Optimal angle</span>
          <span style={s.statVal}>{mission.best_angle?.toFixed(1)}°</span>
        </div>
        <div style={s.stat}>
          <span style={s.statKey}>Total distance</span>
          <span style={s.statVal}>{(mission.total_distance / 1000).toFixed(2)} km</span>
        </div>
        {mission.coverage_area && (
          <div style={s.stat}>
            <span style={s.statKey}>Coverage area</span>
            <span style={s.statVal}>{(mission.coverage_area / 10000).toFixed(2)} ha</span>
          </div>
        )}
        {mission.n_cycles != null && (
          <div style={s.stat}>
            <span style={s.statKey}>Flight cycles</span>
            <span style={s.statVal}>{mission.n_cycles}</span>
          </div>
        )}
        <div style={s.stat}>
          <span style={s.statKey}>Waypoints</span>
          <span style={s.statVal}>{mission.waypoints?.length}</span>
        </div>
        <Divider />
        <Btn variant='default' onClick={onExport}>
          Export Waypoints (JSON)
        </Btn>
      </>}

      {mission.status === 'failed' && (
        <div style={s.errorBox}>{mission.error_message}</div>
      )}
    </div>
  )
}

export default function MissionPanel({
  mode, activeField, drawingPtsCount,
  onToggleDrawPolygon, onToggleDrawObstacle,
  onLoadField, onClear, onMissionReady,
}) {
  const [drones, setDrones]         = useState([])
  const [drone, setDrone]           = useState('')
  const [swathRange, setSwathRange] = useState({ min: 1, max: 20 })
  const [sprayWidth, setSprayWidth] = useState(10)
  const [strategy, setStrategy]     = useState('genetic')
  const [missionName, setName]      = useState('Mission 1')
  const [mission, setMission]       = useState(null)
  const [loading, setLoading]       = useState(false)
  const [error, setError]           = useState(null)
  const pollRef = useRef(null)

  useEffect(() => () => clearInterval(pollRef.current), [])

  useEffect(() => {
    fetch('/drones').then(r => r.json()).then(data => {
      setDrones(data)
      if (data.length) {
        setDrone(data[0].name)
        setSwathRange({ min: data[0].min_swath, max: data[0].max_swath })
        setSprayWidth(data[0].default_swath)
      }
    }).catch(console.error)
  }, [])

  function handleDroneChange(name) {
    setDrone(name)
    const d = drones.find(d => d.name === name)
    if (d) {
      setSwathRange({ min: d.min_swath, max: d.max_swath })
      setSprayWidth(d.default_swath)
    }
  }

  async function handleCompute() {
    if (!activeField) return
    setLoading(true); setError(null); setMission(null)
    clearInterval(pollRef.current)

    try {
      const coords = ensureClosed(activeField.coordinates)
      const obs    = (activeField.obstacles ?? []).map(ensureClosed)

      const res = await fetch('/mission/compute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: missionName,
          field: { coordinates: coords, obstacles: obs },
          spray_width: Number(sprayWidth),
          strategy,
          drone_name: drone,
        }),
      })
      if (!res.ok) { const e = await res.json(); throw new Error(e.detail ?? res.statusText) }
      const data = await res.json()
      setMission(data)

      pollRef.current = setInterval(async () => {
        const r = await fetch(`/mission/${data.id}`)
        const m = await r.json()
        setMission(m)
        if (m.status === 'completed' || m.status === 'failed') {
          clearInterval(pollRef.current)
          if (m.status === 'completed') onMissionReady(m, m.waypoints)
        }
      }, 1500)
    } catch (e) {
      setError(e.message)
    } finally {
      setLoading(false)
    }
  }

  function handleExport() {
    if (!mission?.waypoints?.length) return
    const data = {
      mission_name: mission.name, drone,
      strategy: mission.strategy,
      best_angle_deg: mission.best_angle,
      total_distance_m: mission.total_distance,
      coverage_area_m2: mission.coverage_area,
      spray_width_m: mission.spray_width,
      waypoints: mission.waypoints.map(w => ({
        sequence: w.sequence, x: w.x, y: w.y, type: w.waypoint_type,
      })),
    }
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })
    const url  = URL.createObjectURL(blob)
    const a    = document.createElement('a')
    a.href = url
    a.download = `${mission.name.replace(/\s+/g, '_')}.json`
    a.click(); URL.revokeObjectURL(url)
  }

  const hasField   = !!activeField
  const canCompute = hasField && !loading && mode === MODE.NONE

  return (
    <div style={s.panel}>
      <HeaderSection />

      <AircraftSection
        drones={drones}
        drone={drone}
        onDroneChange={handleDroneChange}
      />

      <FieldSection
        mode={mode}
        activeField={activeField}
        drawingPtsCount={drawingPtsCount}
        onToggleDrawPolygon={onToggleDrawPolygon}
        onToggleDrawObstacle={onToggleDrawObstacle}
        onLoadField={onLoadField}
        onClear={onClear}
      />

      <ParametersSection
        missionName={missionName}
        setName={setName}
        sprayWidth={sprayWidth}
        setSprayWidth={setSprayWidth}
        swathRange={swathRange}
        strategy={strategy}
        setStrategy={setStrategy}
      />

      <div style={{ padding: '14px 16px', borderBottom: `1px solid ${C.border}` }}>
        {error && <div style={{ ...s.errorBox, marginBottom: 10 }}>{error}</div>}
        <Btn variant='primary' disabled={!canCompute} onClick={handleCompute}>
          {loading
            ? <><Spinner /> Computing…</>
            : 'Compute Mission'}
        </Btn>
        {!hasField && (
          <div style={{ fontSize: 11, color: C.muted, textAlign: 'center', marginTop: 7 }}>
            Draw or load a field to continue
          </div>
        )}
      </div>

      {mission && (
        <ResultsSection mission={mission} onExport={handleExport} />
      )}
    </div>
  )
}
