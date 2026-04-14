import { useEffect, useRef, useState } from 'react'
import { ensureClosed } from '../utils/geo'
import { MODE } from '../utils/modes'
import { C } from '../utils/colors'

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
    background: `${C.error}12`, border: `1px solid ${C.error}`,
    borderRadius: 6, padding: '8px 10px', fontSize: 11, color: '#ffa198',
  },
}

function Btn({ children, onClick, variant = 'default', active = false, disabled = false, fullWidth = true }) {
  const variants = {
    default:  { bg: C.surface,   border: C.border,    color: C.text },
    primary:  { bg: C.accentDim, border: C.accent,    color: '#ffffff' },
    success:  { bg: `${C.success}18`, border: `${C.success}80`, color: C.success },
    danger:   { bg: `${C.danger}25`,  border: C.danger,         color: '#ff7b72' },
    active:   { bg: `${C.accent}28`,  border: C.accent,         color: C.accent },
    warning:  { bg: `${C.warning}28`, border: `${C.warning}90`, color: '#e3b341' },
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
        <span style={s.logoText}>UAV-UAG Mission Planner</span>
      </div>
    </div>
  )
}

function AircraftSection({ drones, drone, onDroneChange, onViewSpecs }) {
  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Aircraft</div>
      <div>
        <div style={s.label}>Drone model</div>
        <div style={{ display: 'flex', gap: 6 }}>
          <select style={{ ...s.select, flex: 1 }} value={drone} onChange={e => onDroneChange(e.target.value)}>
            {drones.map(d => <option key={d.name} value={d.name}>{d.name}</option>)}
          </select>
          <button
            onClick={() => drone && onViewSpecs(drone)}
            disabled={!drone}
            title="View drone specifications"
            style={{
              flexShrink: 0,
              width: 32, height: 32,
              display: 'flex', alignItems: 'center', justifyContent: 'center',
              background: C.surface, border: `1px solid ${C.border}`,
              borderRadius: 6, color: C.accent,
              fontSize: 13, fontWeight: 700,
              cursor: drone ? 'pointer' : 'not-allowed',
              opacity: drone ? 1 : 0.4,
              fontFamily: 'inherit',
            }}
          >
            i
          </button>
        </div>
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
          {isDrawingPolygon ? `Finish (${drawingPtsCount} pts)` : 'Draw Field'}
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
        <Btn variant='default' onClick={() => fileRef.current.click()}>Load JSON</Btn>
        <Btn variant='danger' disabled={!hasField && mode === MODE.NONE} onClick={onClear}>Clear All</Btn>
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

const MISSION_TYPES = [
  {
    value: 'static',
    label: 'Static Base',
    description: 'UAV returns to a fixed base station to recharge and refill.',
  },
  {
    value: 'mobile',
    label: 'Mobile UGV',
    description: 'UGV moves along a route; UAV meets it at rendezvous points.',
  },
]

function GroundSection({
  missionType, onMissionTypeChange,
  mode, basePoint, onToggleSetBasePoint,
  ugvRoute, drawingPtsCount, drawingLengthM,
  ugvSpeed, setUgvSpeed, ugvTService, setUgvTService,
  onToggleDrawUgvRoute,
}) {
  const isSettingBase = mode === MODE.SET_BASE_POINT
  const isDrawingUgv  = mode === MODE.DRAW_UGV_ROUTE
  const hasRoute      = ugvRoute && ugvRoute.length >= 2

  const routeLengthM = hasRoute
    ? ugvRoute.slice(1).reduce((acc, pt, i) => acc + Math.hypot(pt[0] - ugvRoute[i][0], pt[1] - ugvRoute[i][1]), 0)
    : 0
  const fmtM = m => m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${m.toFixed(0)} m`

  const isMobile = missionType === 'mobile'
  const typeInfo  = MISSION_TYPES.find(t => t.value === missionType)

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Ground Operations</div>

      {/* Mode toggle */}
      <div style={{
        display: 'grid', gridTemplateColumns: '1fr 1fr',
        borderRadius: 7, overflow: 'hidden',
        border: `1px solid ${C.border}`,
      }}>
        {MISSION_TYPES.map((opt, i) => {
          const active = missionType === opt.value
          return (
            <button
              key={opt.value}
              onClick={() => onMissionTypeChange(opt.value)}
              style={{
                padding: '10px 8px',
                background: active ? C.accentDim : C.surface,
                border: 'none',
                borderLeft: i > 0 ? `1px solid ${C.border}` : 'none',
                color: active ? '#fff' : C.muted,
                fontSize: 11, fontWeight: active ? 700 : 500,
                cursor: 'pointer', fontFamily: 'inherit',
                transition: 'background 0.15s, color 0.15s',
                lineHeight: 1.3,
              }}
            >
              {opt.label}
            </button>
          )
        })}
      </div>

      <div style={{ fontSize: 11, color: C.muted, lineHeight: 1.5 }}>
        {typeInfo?.description}
      </div>

      <Divider />

      {/* Base / start point */}
      <Btn
        variant={isSettingBase ? 'active' : 'default'}
        active={isSettingBase}
        onClick={onToggleSetBasePoint}
      >
        {isSettingBase
          ? 'Click on map…'
          : basePoint
            ? `Move ${isMobile ? 'UGV start' : 'base station'}`
            : `Set ${isMobile ? 'UGV start' : 'base station'}`}
      </Btn>

      {basePoint && (
        <div style={{ fontSize: 11, color: C.muted }}>
          {isMobile ? 'Start' : 'Base'}: ({basePoint[0].toFixed(1)}, {basePoint[1].toFixed(1)})
        </div>
      )}

      {/* Mobile-only UGV route controls */}
      {isMobile && (
        <>
          <Divider />

          <Btn
            variant={isDrawingUgv ? 'active' : hasRoute ? 'warning' : 'default'}
            active={isDrawingUgv}
            onClick={onToggleDrawUgvRoute}
          >
            {isDrawingUgv
              ? drawingLengthM > 0
                ? `Finish Route (${fmtM(drawingLengthM)})`
                : `Finish Route (${drawingPtsCount} pts)`
              : hasRoute
                ? `Redraw UGV Route (${fmtM(routeLengthM)})`
                : 'Draw UGV Route'}
          </Btn>

          {hasRoute && !isDrawingUgv && (
            <div style={{ fontSize: 11, color: C.muted }}>
              Route: {fmtM(routeLengthM)} · {ugvRoute.length} waypoints
            </div>
          )}

          <div style={s.btnRow}>
            <div>
              <div style={s.label}>UGV speed (m/s)</div>
              <input
                style={s.input} type="number"
                min={0.1} max={10} step={0.1}
                value={ugvSpeed} onChange={e => setUgvSpeed(e.target.value)}
              />
            </div>
            <div>
              <div style={s.label}>Service time (s)</div>
              <input
                style={s.input} type="number"
                min={30} max={1800} step={30}
                value={ugvTService} onChange={e => setUgvTService(e.target.value)}
              />
            </div>
          </div>
        </>
      )}
    </div>
  )
}

function RangeHint({ min, max, unit = '' }) {
  if (min == null || max == null) return null
  return (
    <span style={{ color: C.muted, fontSize: 10 }}>
      {min}{unit}–{max}{unit}
    </span>
  )
}

function ParamInput({ label, value, onChange, min, max, step = 0.5, hint }) {
  return (
    <div>
      <div style={{ ...s.label, display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        {hint}
      </div>
      <input
        style={s.input} type="number"
        min={min} max={max} step={step}
        value={value} onChange={e => onChange(e.target.value)}
      />
    </div>
  )
}

function ParametersSection({
  sprayWidth, setSprayWidth,
  appRate, setAppRate,
  speed, setSpeed,
  margin, setMargin,
  defaults,
  strategy, setStrategy,
}) {
  const d = defaults

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Mission Parameters</div>

      <div style={s.btnRow}>
        <ParamInput
          label="Spray width (m)"
          value={sprayWidth} onChange={setSprayWidth}
          min={d?.spray_swath_min_m} max={d?.spray_swath_max_m}
          hint={<RangeHint min={d?.spray_swath_min_m} max={d?.spray_swath_max_m} unit=" m" />}
        />
        <ParamInput
          label="App rate (L/ha)"
          value={appRate} onChange={setAppRate}
          min={d?.app_rate_min_l_ha} max={d?.app_rate_max_l_ha}
          hint={<RangeHint min={d?.app_rate_min_l_ha} max={d?.app_rate_max_l_ha} />}
        />
      </div>

      <div style={s.btnRow}>
        <ParamInput
          label="Speed (m/s)"
          value={speed} onChange={setSpeed}
          min={d?.speed_min_ms} max={d?.speed_max_ms}
          hint={<RangeHint min={d?.speed_min_ms} max={d?.speed_max_ms} unit=" m/s" />}
        />
        <ParamInput
          label="Margin (m)"
          value={margin} onChange={setMargin}
          min={0}
        />
      </div>

      <div>
        <div style={s.label}>Planning strategy</div>
        <select style={s.select} value={strategy} onChange={e => setStrategy(e.target.value)}>
          <option value="grid">Grid Search (exhaustivo, recomendado)</option>
          <option value="genetic">Genetic Algorithm (GA)</option>
        </select>
      </div>
    </div>
  )
}

function MetricRow({ label, value }) {
  return (
    <div style={s.stat}>
      <span style={s.statKey}>{label}</span>
      <span style={s.statVal}>{value}</span>
    </div>
  )
}

function ResultsSection({ mission, onExport, onStartSim, onStopSim, simEnabled }) {
  const metrics = (() => {
    try { return mission.metrics_json ? JSON.parse(mission.metrics_json) : null }
    catch { return null }
  })()

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

        {/* Geometry */}
        <MetricRow
          label="Optimal angle"
          value={`${mission.best_angle?.toFixed(1)}°`}
        />
        {mission.coverage_area && (
          <MetricRow
            label="Coverage area"
            value={`${(mission.coverage_area / 10000).toFixed(2)} ha`}
          />
        )}
        {mission.n_cycles != null && (
          <MetricRow label="Flight cycles" value={mission.n_cycles} />
        )}
        <MetricRow label="Waypoints" value={mission.waypoints?.length} />

        {/* Mission metrics from MissionAnalyzer */}
        {metrics && <>
          <Divider />
          <MetricRow
            label="Spray distance"
            value={`${metrics.spray_dist_km?.toFixed(3)} km`}
          />
          <MetricRow
            label="Dead distance"
            value={`${metrics.dead_dist_km?.toFixed(3)} km`}
          />
          <MetricRow
            label="Efficiency"
            value={`${metrics.efficiency_ratio?.toFixed(1)} %`}
          />
          <MetricRow
            label="Flight time"
            value={`${metrics.flight_time_min?.toFixed(1)} min`}
          />
          <MetricRow
            label="Total op. time"
            value={`${metrics.total_op_time_min?.toFixed(1)} min`}
          />
          <MetricRow
            label="Productivity"
            value={`${metrics.productivity_ha_hr?.toFixed(2)} ha/hr`}
          />
          <MetricRow
            label="Applied dosage"
            value={`${metrics.real_dosage_l_ha?.toFixed(1)} L/ha`}
          />
        </>}

        {metrics?.rv_n_rendezvous != null && metrics.rv_n_rendezvous > 0 && <>
          <Divider />
          <MetricRow label="Rendezvous stops" value={metrics.rv_n_rendezvous} />
          <MetricRow
            label="UAV wait (total)"
            value={`${metrics.rv_wait_min?.toFixed(1)} min`}
          />
        </>}

        <Divider />
        {simEnabled ? (
          <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
            <span style={{
              flex: 1, fontSize: 11, color: C.success, fontWeight: 600,
              display: 'flex', alignItems: 'center', gap: 5,
            }}>
              <span style={{
                width: 7, height: 7, borderRadius: '50%',
                background: C.success, display: 'inline-block',
                boxShadow: `0 0 5px ${C.success}`,
              }} />
              Simulation active
            </span>
            <Btn variant='danger' fullWidth={false} onClick={onStopSim}>Stop</Btn>
          </div>
        ) : (
          <Btn variant='primary' onClick={onStartSim}>Run Simulation</Btn>
        )}
        <Btn variant='default' onClick={onExport}>Export Mission</Btn>
      </>}

      {mission.status === 'failed' && (
        <div style={s.errorBox}>{mission.error_message}</div>
      )}
    </div>
  )
}

function ExportDialog({ onConfirm, onCancel }) {
  const [name, setName] = useState('Mission')
  return (
    <div style={{
      position: 'fixed', inset: 0, zIndex: 1000,
      background: 'rgba(0,0,0,0.55)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: C.bg, border: `1px solid ${C.border}`,
        borderRadius: 10, padding: '20px 22px', width: 300,
        display: 'flex', flexDirection: 'column', gap: 14,
        boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
      }}>
        <div style={{ fontSize: 13, fontWeight: 700, color: C.text }}>Export Mission</div>
        <div>
          <div style={s.label}>File name</div>
          <input
            style={s.input}
            value={name}
            onChange={e => setName(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && name.trim() && onConfirm(name.trim())}
            autoFocus
          />
        </div>
        <div style={{ display: 'flex', gap: 8 }}>
          <Btn variant='default' onClick={onCancel}>Cancel</Btn>
          <Btn variant='primary' disabled={!name.trim()} onClick={() => onConfirm(name.trim())}>
            Download JSON
          </Btn>
        </div>
      </div>
    </div>
  )
}

export default function MissionPanel({
  mode, activeField, drawingPtsCount, drawingLengthM, basePoint,
  ugvRoute,
  onToggleDrawPolygon, onToggleDrawObstacle, onToggleSetBasePoint,
  onToggleDrawUgvRoute,
  onLoadField, onClear, onMissionReady, onStartSim, onStopSim, simEnabled, onViewDroneSpecs,
}) {
  const [drones, setDrones]         = useState([])
  const [drone, setDrone]           = useState('')
  const [missionType, setMissionType] = useState('static')
  // Mission parameter ranges (populated from /drones/{name}/defaults)
  const [defaults, setDefaults]     = useState(null)
  // User-editable mission parameters
  const [sprayWidth, setSprayWidth] = useState('')
  const [speed, setSpeed]           = useState('')
  const [appRate, setAppRate]       = useState('')
  const [margin, setMargin]         = useState('')
  const [strategy, setStrategy]     = useState('grid')
  const [ugvSpeed, setUgvSpeed]     = useState(2.0)
  const [ugvTService, setUgvTService] = useState(300)
  const [mission, setMission]       = useState(null)
  const [loading, setLoading]       = useState(false)
  const [error, setError]           = useState(null)
  const [showExport, setShowExport] = useState(false)
  const pollRef = useRef(null)

  useEffect(() => () => clearInterval(pollRef.current), [])

  useEffect(() => {
    fetch('/drones').then(r => r.json()).then(data => {
      setDrones(data)
      if (data.length) fetchAndApplyDefaults(data[0].name)
    }).catch(console.error)
  }, [])

  function fetchAndApplyDefaults(name) {
    setDrone(name)
    fetch(`/drones/${encodeURIComponent(name)}/defaults`)
      .then(r => r.json())
      .then(d => {
        setDefaults(d)
        setSprayWidth(d.swath_m)
        setSpeed(d.speed_ms)
        setAppRate(d.app_rate_l_ha)
        setMargin(d.margin_m)
      })
      .catch(console.error)
  }

  function handleDroneChange(name) {
    fetchAndApplyDefaults(name)
  }

  async function handleCompute() {
    if (!activeField) return
    if (!basePoint) {
      setError('Please set a base point before computing the mission.')
      return
    }
    const sw = Number(sprayWidth)
    const ar = Number(appRate)
    const sp = Number(speed)
    const mg = Number(margin)
    if (!sw || sw <= 0) { setError('Spray width must be > 0.'); return }
    if (defaults?.spray_swath_min_m != null && sw < defaults.spray_swath_min_m)
      { setError(`Spray width too low (min ${defaults.spray_swath_min_m} m).`); return }
    if (defaults?.spray_swath_max_m != null && sw > defaults.spray_swath_max_m)
      { setError(`Spray width too high (max ${defaults.spray_swath_max_m} m).`); return }
    if (!ar || ar <= 0) { setError('Application rate must be > 0.'); return }
    if (defaults?.app_rate_min_l_ha != null && ar < defaults.app_rate_min_l_ha)
      { setError(`App rate too low (min ${defaults.app_rate_min_l_ha} L/ha).`); return }
    if (defaults?.app_rate_max_l_ha != null && ar > defaults.app_rate_max_l_ha)
      { setError(`App rate too high (max ${defaults.app_rate_max_l_ha} L/ha).`); return }
    if (!sp || sp <= 0) { setError('Speed must be > 0.'); return }
    if (defaults?.speed_min_ms != null && sp < defaults.speed_min_ms)
      { setError(`Speed too low (min ${defaults.speed_min_ms} m/s).`); return }
    if (defaults?.speed_max_ms != null && sp > defaults.speed_max_ms)
      { setError(`Speed too high (max ${defaults.speed_max_ms} m/s).`); return }
    if (isNaN(mg) || mg < 0) { setError('Margin must be >= 0.'); return }
    setLoading(true); setError(null); setMission(null)
    clearInterval(pollRef.current)

    try {
      const coords = ensureClosed(activeField.coordinates)
      const obs    = (activeField.obstacles ?? []).map(ensureClosed)

      const fieldPayload = {
        coordinates: coords,
        obstacles: obs,
        base_point: basePoint,
      }

      if (missionType === 'mobile' && ugvRoute && ugvRoute.length >= 2) {
        fieldPayload.ugv_polyline  = ugvRoute
        fieldPayload.ugv_speed     = Number(ugvSpeed)
        fieldPayload.ugv_t_service = Number(ugvTService)
      }

      const res = await fetch('/mission/compute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: `Mission ${new Date().toISOString().slice(0, 16).replace('T', ' ')}`,
          field: fieldPayload,
          spray_width: Number(sprayWidth),
          strategy,
          drone_name: drone,
          app_rate: Number(appRate),
          cruise_speed_ms: Number(speed),
          margin_m: Number(margin),
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

  function handleExport(name) {
    if (!mission?.waypoints?.length) return
    const metrics = (() => {
      try { return mission.metrics_json ? JSON.parse(mission.metrics_json) : {} }
      catch { return {} }
    })()
    const data = {
      mission_name: name, drone,
      strategy: mission.strategy,
      best_angle_deg: mission.best_angle,
      n_cycles: mission.n_cycles,
      total_distance_m: mission.total_distance,
      coverage_area_m2: mission.coverage_area,
      spray_width_m: mission.spray_width,
      metrics,
      waypoints: mission.waypoints.map(w => ({
        sequence: w.sequence, x: w.x, y: w.y, type: w.waypoint_type,
        cycle_index: w.cycle_index,
      })),
    }
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })
    const url  = URL.createObjectURL(blob)
    const a    = document.createElement('a')
    a.href = url
    a.download = `${name.replace(/\s+/g, '_')}.json`
    a.click()
    URL.revokeObjectURL(url)
    setShowExport(false)
  }

  const hasField      = !!activeField
  const hasBasePoint  = !!basePoint
  const needsUgvRoute = missionType === 'mobile'
  const hasUgvRoute   = ugvRoute && ugvRoute.length >= 2
  const canCompute    = hasField && hasBasePoint && !loading && mode === MODE.NONE
    && !(needsUgvRoute && !hasUgvRoute)

  return (
    <div style={s.panel}>
      <HeaderSection />

      <AircraftSection
        drones={drones}
        drone={drone}
        onDroneChange={handleDroneChange}
        onViewSpecs={onViewDroneSpecs}
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

      <GroundSection
        missionType={missionType}
        onMissionTypeChange={setMissionType}
        mode={mode}
        basePoint={basePoint}
        onToggleSetBasePoint={onToggleSetBasePoint}
        ugvRoute={ugvRoute}
        drawingPtsCount={drawingPtsCount}
        drawingLengthM={drawingLengthM}
        ugvSpeed={ugvSpeed}
        setUgvSpeed={setUgvSpeed}
        ugvTService={ugvTService}
        setUgvTService={setUgvTService}
        onToggleDrawUgvRoute={onToggleDrawUgvRoute}
      />

      <ParametersSection
        sprayWidth={sprayWidth}
        setSprayWidth={setSprayWidth}
        appRate={appRate}
        setAppRate={setAppRate}
        speed={speed}
        setSpeed={setSpeed}
        margin={margin}
        setMargin={setMargin}
        defaults={defaults}
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
        {mode === MODE.DRAW_UGV_ROUTE && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            Finish the UGV route before computing
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && !hasField && (
          <div style={{ fontSize: 11, color: C.muted, textAlign: 'center', marginTop: 7 }}>
            Draw or load a field to continue
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && hasField && !hasBasePoint && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            {missionType === 'mobile' ? 'Set UGV start position before computing' : 'Set a base station before computing'}
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && hasField && hasBasePoint && needsUgvRoute && !hasUgvRoute && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            Draw the UGV route to enable mobile rendezvous
          </div>
        )}
      </div>

      {mission && (
        <ResultsSection
          mission={mission}
          onExport={() => setShowExport(true)}
          onStartSim={onStartSim}
          onStopSim={onStopSim}
          simEnabled={simEnabled}
        />
      )}

      {showExport && (
        <ExportDialog
          onConfirm={handleExport}
          onCancel={() => setShowExport(false)}
        />
      )}
    </div>
  )
}
