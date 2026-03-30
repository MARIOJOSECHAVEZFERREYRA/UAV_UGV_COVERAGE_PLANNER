import { useRef, useCallback, useState, useEffect } from 'react'
import { MODE } from '../utils/modes'
import ZoomControls from './ZoomControls'

const W      = 1000
const OX     = W / 2
const OY_SVG = W / 2
const GRID_STEP = 50
const ZOOM_MIN  = 0.3
const ZOOM_MAX  = 200

function mToSVG(x, y) { return [OX + x, OY_SVG - y] }

function clientToSVG(e, svg) {
  const ctm = svg.getScreenCTM()
  if (!ctm) return null
  const pt = svg.createSVGPoint()
  pt.x = e.clientX
  pt.y = e.clientY
  return pt.matrixTransform(ctm.inverse())
}

function safe(vb, fallbackH) {
  return {
    x: isFinite(vb.x) ? vb.x : 0,
    y: isFinite(vb.y) ? vb.y : 0,
    w: isFinite(vb.w) && vb.w > 0 ? vb.w : W,
    h: isFinite(vb.h) && vb.h > 0 ? vb.h : (fallbackH ?? W * 0.7),
  }
}

function niceMeters(raw) {
  const steps = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
  return steps.find(s => s >= raw * 0.6) ?? steps[steps.length - 1]
}

function toPoints(pts) {
  return pts.map(([x, y]) => mToSVG(x, y).join(',')).join(' ')
}

function fmtDist(d) {
  return d >= 100 ? `${d.toFixed(0)}m` : `${d.toFixed(1)}m`
}

function EdgeLabels({ pts, px, color, closed = true }) {
  if (!pts || pts.length < 2) return null
  const n = pts.length
  const count = closed ? n : n - 1
  return Array.from({ length: count }, (_, i) => {
    const p1 = pts[i]
    const p2 = pts[(i + 1) % n]
    const [sx1, sy1] = mToSVG(p1[0], p1[1])
    const [sx2, sy2] = mToSVG(p2[0], p2[1])
    const d = Math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    return (
      <text key={i}
        x={(sx1 + sx2) / 2} y={(sy1 + sy2) / 2 - 5 * px}
        fontSize={11 * px} textAnchor="middle"
        fill={color} stroke="#fff" strokeWidth={3 * px} paintOrder="stroke fill"
        style={{ userSelect: 'none', pointerEvents: 'none' }}
      >
        {fmtDist(d)}
      </text>
    )
  })
}

function buildGridLines(sv, px) {
  const lines = []
  for (let sx = Math.floor(sv.x / GRID_STEP) * GRID_STEP; sx <= sv.x + sv.w; sx += GRID_STEP) {
    const isAxis = Math.abs(sx - OX) < 0.5
    lines.push(
      <line key={`v${sx}`} x1={sx} y1={sv.y} x2={sx} y2={sv.y + sv.h}
        stroke={isAxis ? '#cbd5e1' : '#e2e8f0'}
        strokeWidth={(isAxis ? 1.5 : 0.5) * px} />
    )
  }
  for (let sy = Math.floor(sv.y / GRID_STEP) * GRID_STEP; sy <= sv.y + sv.h; sy += GRID_STEP) {
    const isAxis = Math.abs(sy - OY_SVG) < 0.5
    lines.push(
      <line key={`h${sy}`} x1={sv.x} y1={sy} x2={sv.x + sv.w} y2={sy}
        stroke={isAxis ? '#cbd5e1' : '#e2e8f0'}
        strokeWidth={(isAxis ? 1.5 : 0.5) * px} />
    )
  }
  return lines
}

function buildGridLabels(sv, px, zoom) {
  const labelStep = GRID_STEP * Math.max(1, Math.ceil(2 / zoom))
  const labels = []
  for (let sx = Math.ceil(sv.x / labelStep) * labelStep; sx <= sv.x + sv.w; sx += labelStep) {
    const mx = Math.round(sx - OX); if (mx === 0) continue
    labels.push(
      <text key={`lx${sx}`} x={sx} y={OY_SVG + 12 * px}
        fontSize={9 * px} fill="#94a3b8" textAnchor="middle">{mx}m</text>
    )
  }
  for (let sy = Math.ceil(sv.y / labelStep) * labelStep; sy <= sv.y + sv.h; sy += labelStep) {
    const my = Math.round(OY_SVG - sy); if (my === 0) continue
    labels.push(
      <text key={`ly${sy}`} x={OX + 4 * px} y={sy + 3 * px}
        fontSize={9 * px} fill="#94a3b8">{my}m</text>
    )
  }
  return labels
}

const SCALE_BAR_STYLE = {
  backgroundColor: 'hsla(0,0%,100%,.75)',
  borderLeft: '2px solid #333',
  borderRight: '2px solid #333',
  borderBottom: '2px solid #333',
  boxSizing: 'border-box',
  color: '#333',
  fontSize: 10,
  fontFamily: 'inherit',
  padding: '0 5px',
  textAlign: 'center',
}

export default function PolygonCanvas({
  field, previewPoints, waypoints, drawMode,
  onCanvasClick, onCanvasRightClick,
}) {
  const wrapRef  = useRef(null)
  const svgRef   = useRef(null)
  const dragRef  = useRef(null)
  const movedRef = useRef(false)
  const initRef  = useRef(false)

  const [vb, setVb]         = useState({ x: 0, y: OY_SVG - W * 0.7 / 2, w: W, h: W * 0.7 })
  const [isPanning, setIsPanning] = useState(false)
  const [pxW, setPxW]       = useState(800)

  const sv   = safe(vb)
  const zoom = W / sv.w
  const px   = 1 / zoom

  useEffect(() => {
    const el = wrapRef.current
    if (!el) return
    const update = () => {
      const r = el.getBoundingClientRect()
      if (r.width === 0 || r.height === 0) return
      setPxW(r.width)
      setVb(prev => {
        const s = safe(prev)
        const newH = s.w * (r.height / r.width)
        if (!initRef.current) {
          initRef.current = true
          return { x: OX - s.w / 2, y: OY_SVG - newH / 2, w: s.w, h: newH }
        }
        const cy = s.y + s.h / 2
        return { ...s, y: cy - newH / 2, h: newH }
      })
    }
    update()
    const ro = new ResizeObserver(update)
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  const applyZoom = useCallback((factor, cx, cy) => {
    setVb(prev => {
      const s = safe(prev)
      const newZoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, (W / s.w) * factor))
      const newW = W / newZoom
      const newH = newW * (s.h / s.w)
      const pivotX = cx ?? s.x + s.w / 2
      const pivotY = cy ?? s.y + s.h / 2
      const x = pivotX - ((pivotX - s.x) / s.w) * newW
      const y = pivotY - ((pivotY - s.y) / s.h) * newH
      return isFinite(x) && isFinite(y) ? { x, y, w: newW, h: newH } : prev
    })
  }, [])

  useEffect(() => {
    const svg = svgRef.current
    const onWheel = (e) => {
      e.preventDefault()
      const pt = clientToSVG(e, svg)
      if (!pt) return
      applyZoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, pt.x, pt.y)
    }
    svg.addEventListener('wheel', onWheel, { passive: false })
    return () => svg.removeEventListener('wheel', onWheel)
  }, [applyZoom])

  const handleMouseDown = useCallback((e) => {
    if (e.button !== 0 || drawMode !== MODE.NONE) return
    const rect = svgRef.current.getBoundingClientRect()
    if (!rect.width) return
    movedRef.current = false
    dragRef.current = {
      clientX: e.clientX, clientY: e.clientY,
      vbX: vb.x, vbY: vb.y,
      scaleX: vb.w / rect.width,
      scaleY: vb.h / rect.height,
    }
    setIsPanning(true)
  }, [drawMode, vb])

  const handleMouseMove = useCallback((e) => {
    if (!dragRef.current) return
    const dx = e.clientX - dragRef.current.clientX
    const dy = e.clientY - dragRef.current.clientY
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) movedRef.current = true
    const x = dragRef.current.vbX - dx * dragRef.current.scaleX
    const y = dragRef.current.vbY - dy * dragRef.current.scaleY
    if (isFinite(x) && isFinite(y)) setVb(prev => ({ ...prev, x, y }))
  }, [])

  const handleMouseUp = useCallback(() => {
    dragRef.current = null
    setIsPanning(false)
  }, [])

  const handleClick = useCallback((e) => {
    if (movedRef.current) { movedRef.current = false; return }
    if (drawMode === MODE.NONE) return
    const pt = clientToSVG(e, svgRef.current)
    if (!pt) return
    onCanvasClick?.(pt.x - OX, OY_SVG - pt.y)
  }, [drawMode, onCanvasClick])

  const handleContextMenu = useCallback((e) => {
    e.preventDefault()
    if (movedRef.current) { movedRef.current = false; return }
    if (drawMode === MODE.NONE) return
    onCanvasRightClick?.()
  }, [drawMode, onCanvasRightClick])

  const gridLines  = buildGridLines(sv, px)
  const gridLabels = buildGridLabels(sv, px, zoom)

  const metersPerPixel = sv.w / pxW
  const barMeters      = niceMeters(100 * metersPerPixel)
  const barPx          = barMeters / metersPerPixel
  const barLabel       = barMeters >= 1000 ? `${barMeters / 1000} km` : `${barMeters} m`

  const drawColor = drawMode === MODE.DRAW_OBSTACLE ? '#ef4444' : '#38BDF8'
  const cursor    = isPanning ? 'grabbing' : drawMode !== MODE.NONE ? 'crosshair' : 'default'

  return (
    <div ref={wrapRef} style={{ position: 'relative', width: '100%', height: '100%' }}>
      <svg
        ref={svgRef}
        viewBox={`${sv.x} ${sv.y} ${sv.w} ${sv.h}`}
        preserveAspectRatio="none"
        style={{ width: '100%', height: '100%', cursor, display: 'block', outline: 'none' }}
        onClick={handleClick}
        onContextMenu={handleContextMenu}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        <rect x={sv.x} y={sv.y} width={sv.w} height={sv.h} fill="#ffffff" stroke="none" />
        {gridLines}
        {gridLabels}

        {field?.coordinates?.length >= 3 && (
          <polygon points={toPoints(field.coordinates)}
            fill="#38BDF8" fillOpacity={0.12}
            stroke="#38BDF8" strokeWidth={2 * px} strokeLinejoin="round" />
        )}
        {field?.coordinates?.length >= 2 && (
          <EdgeLabels pts={field.coordinates} px={px} color="#38BDF8" closed />
        )}
        {field?.obstacles?.map((obs, i) => obs.length >= 3 && (
          <polygon key={i} points={toPoints(obs)}
            fill="#ef4444" fillOpacity={0.15}
            stroke="#ef4444" strokeWidth={1.5 * px} strokeLinejoin="round" />
        ))}
        {field?.obstacles?.map((obs, i) => obs.length >= 2 && (
          <EdgeLabels key={`ol${i}`} pts={obs} px={px} color="#ef4444" closed />
        ))}
        {waypoints?.length >= 2 && (
          <polyline points={waypoints.map(wp => mToSVG(wp.x, wp.y).join(',')).join(' ')}
            fill="none" stroke="#FBBF24" strokeWidth={1.8 * px} strokeOpacity={0.9} />
        )}
        {waypoints?.map((wp, i) => {
          const [sx, sy] = mToSVG(wp.x, wp.y)
          return <circle key={i} cx={sx} cy={sy} r={3 * px} fill="#FBBF24" stroke="#fff" strokeWidth={px} />
        })}
        {previewPoints?.length >= 3 && (
          <polygon points={toPoints(previewPoints)} fill={drawColor} fillOpacity={0.1} stroke="none" />
        )}
        {previewPoints?.length >= 2 && (
          <polyline
            points={[...previewPoints, previewPoints[0]].map(([x, y]) => mToSVG(x, y).join(',')).join(' ')}
            fill="none" stroke={drawColor} strokeWidth={1.5 * px}
            strokeDasharray={`${6 * px} ${3 * px}`} />
        )}
        {previewPoints?.length >= 2 && (
          <EdgeLabels pts={previewPoints} px={px} color={drawColor} closed={previewPoints.length >= 3} />
        )}
        {previewPoints?.map(([x, y], i) => {
          const [sx, sy] = mToSVG(x, y)
          return <circle key={i} cx={sx} cy={sy} r={5 * px}
            fill="#ffffff" stroke={drawColor} strokeWidth={2 * px} />
        })}
        <circle cx={OX} cy={OY_SVG} r={3 * px} fill="#94a3b8" />
      </svg>

      <ZoomControls onZoomIn={() => applyZoom(1.5)} onZoomOut={() => applyZoom(1 / 1.5)} />

      <div style={{ position: 'absolute', bottom: 10, right: 10 }}>
        <div style={{ ...SCALE_BAR_STYLE, width: barPx }}>
          {barLabel}
        </div>
      </div>
    </div>
  )
}
