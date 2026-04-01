import { useRef, useCallback, useState, useEffect } from 'react'
import { MODE } from '../utils/modes.js'
import { waypointsByType } from '../utils/geo.js'
import {
  getDrawColor,
  getPreviewLinePoints,
  getTrajectoryOpacity,
} from '../utils/viewScene.js'
import ZoomControls from './ZoomControls.jsx'
import EdgeLabelsSvg from './EdgeLabelsSvg.jsx'
import BaseMarkerSvg from './BaseMarkerSvg.jsx'
import VehicleOverlay from './VehicleOverlay.jsx'

const W = 1000
const OX = W / 2
const OY_SVG = W / 2
const GRID_STEP = 50
const ZOOM_MIN = 0.3
const ZOOM_MAX = 200

function mToSVG(x, y) {
  return [OX + x, OY_SVG - y]
}

function clientToSVG(event, svg) {
  const ctm = svg.getScreenCTM()

  if (!ctm) {
    return null
  }

  const point = svg.createSVGPoint()
  point.x = event.clientX
  point.y = event.clientY

  return point.matrixTransform(ctm.inverse())
}

function safe(viewBox, fallbackHeight) {
  return {
    x: Number.isFinite(viewBox.x) ? viewBox.x : 0,
    y: Number.isFinite(viewBox.y) ? viewBox.y : 0,
    w: Number.isFinite(viewBox.w) && viewBox.w > 0 ? viewBox.w : W,
    h: Number.isFinite(viewBox.h) && viewBox.h > 0 ? viewBox.h : (fallbackHeight ?? W * 0.7),
  }
}

function niceMeters(raw) {
  const steps = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
  return steps.find(step => step >= raw * 0.6) ?? steps[steps.length - 1]
}

function toPoints(points) {
  return points.map(([x, y]) => mToSVG(x, y).join(',')).join(' ')
}

function buildGridLines(viewBox, px) {
  const lines = []

  for (
    let sx = Math.floor(viewBox.x / GRID_STEP) * GRID_STEP;
    sx <= viewBox.x + viewBox.w;
    sx += GRID_STEP
  ) {
    const isAxis = Math.abs(sx - OX) < 0.5

    lines.push(
      <line
        key={`v${sx}`}
        x1={sx}
        y1={viewBox.y}
        x2={sx}
        y2={viewBox.y + viewBox.h}
        stroke={isAxis ? '#cbd5e1' : '#e2e8f0'}
        strokeWidth={(isAxis ? 1.5 : 0.5) * px}
      />
    )
  }

  for (
    let sy = Math.floor(viewBox.y / GRID_STEP) * GRID_STEP;
    sy <= viewBox.y + viewBox.h;
    sy += GRID_STEP
  ) {
    const isAxis = Math.abs(sy - OY_SVG) < 0.5

    lines.push(
      <line
        key={`h${sy}`}
        x1={viewBox.x}
        y1={sy}
        x2={viewBox.x + viewBox.w}
        y2={sy}
        stroke={isAxis ? '#cbd5e1' : '#e2e8f0'}
        strokeWidth={(isAxis ? 1.5 : 0.5) * px}
      />
    )
  }

  return lines
}

function buildGridLabels(viewBox, px, zoom) {
  const labelStep = GRID_STEP * Math.max(1, Math.ceil(2 / zoom))
  const labels = []

  for (
    let sx = Math.ceil(viewBox.x / labelStep) * labelStep;
    sx <= viewBox.x + viewBox.w;
    sx += labelStep
  ) {
    const mx = Math.round(sx - OX)

    if (mx === 0) {
      continue
    }

    labels.push(
      <text
        key={`lx${sx}`}
        x={sx}
        y={OY_SVG + 12 * px}
        fontSize={9 * px}
        fill="#94a3b8"
        textAnchor="middle"
      >
        {mx}m
      </text>
    )
  }

  for (
    let sy = Math.ceil(viewBox.y / labelStep) * labelStep;
    sy <= viewBox.y + viewBox.h;
    sy += labelStep
  ) {
    const my = Math.round(OY_SVG - sy)

    if (my === 0) {
      continue
    }

    labels.push(
      <text
        key={`ly${sy}`}
        x={OX + 4 * px}
        y={sy + 3 * px}
        fontSize={9 * px}
        fill="#94a3b8"
      >
        {my}m
      </text>
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

const SWEEP_COLOR = '#22c55e'
const FERRY_COLOR = '#f59e0b'

export default function PolygonCanvas({
  field,
  previewPoints,
  waypoints,
  basePoint,
  vehicles,
  drawMode,
  highlight,
  onCanvasClick,
  onCanvasRightClick,
}) {
  const wrapRef = useRef(null)
  const svgRef = useRef(null)
  const dragRef = useRef(null)
  const movedRef = useRef(false)
  const initRef = useRef(false)

  const [viewBox, setViewBox] = useState({
    x: 0,
    y: OY_SVG - W * 0.7 / 2,
    w: W,
    h: W * 0.7,
  })
  const [isPanning, setIsPanning] = useState(false)
  const [pixelWidth, setPixelWidth] = useState(800)

  const safeViewBox = safe(viewBox)
  const zoom = W / safeViewBox.w
  const px = 1 / zoom

  const previewLinePoints = getPreviewLinePoints(previewPoints ?? [])
  const drawColor = getDrawColor(drawMode)
  const { sweepOpacity, ferryOpacity, deadheadOpacity } = getTrajectoryOpacity(highlight)
  const { sweepRuns, ferryRuns, deadheadRuns, basePoints } = waypointsByType(waypoints)

  useEffect(() => {
    const element = wrapRef.current

    if (!element) {
      return
    }

    const update = () => {
      const rect = element.getBoundingClientRect()

      if (rect.width === 0 || rect.height === 0) {
        return
      }

      setPixelWidth(rect.width)

      setViewBox(prev => {
        const current = safe(prev)
        const newHeight = current.w * (rect.height / rect.width)

        if (!initRef.current) {
          initRef.current = true
          return {
            x: OX - current.w / 2,
            y: OY_SVG - newHeight / 2,
            w: current.w,
            h: newHeight,
          }
        }

        const cy = current.y + current.h / 2

        return {
          ...current,
          y: cy - newHeight / 2,
          h: newHeight,
        }
      })
    }

    update()

    const resizeObserver = new ResizeObserver(update)
    resizeObserver.observe(element)

    return () => resizeObserver.disconnect()
  }, [])

  const applyZoom = useCallback((factor, cx, cy) => {
    setViewBox(prev => {
      const current = safe(prev)
      const newZoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, (W / current.w) * factor))
      const newW = W / newZoom
      const newH = newW * (current.h / current.w)
      const pivotX = cx ?? current.x + current.w / 2
      const pivotY = cy ?? current.y + current.h / 2
      const x = pivotX - ((pivotX - current.x) / current.w) * newW
      const y = pivotY - ((pivotY - current.y) / current.h) * newH

      return Number.isFinite(x) && Number.isFinite(y)
        ? { x, y, w: newW, h: newH }
        : prev
    })
  }, [])

  useEffect(() => {
    const svg = svgRef.current

    const onWheel = event => {
      event.preventDefault()

      const point = clientToSVG(event, svg)

      if (!point) {
        return
      }

      applyZoom(event.deltaY < 0 ? 1.15 : 1 / 1.15, point.x, point.y)
    }

    svg.addEventListener('wheel', onWheel, { passive: false })

    return () => svg.removeEventListener('wheel', onWheel)
  }, [applyZoom])

  const handleMouseDown = useCallback((event) => {
    if (event.button !== 0 || drawMode !== MODE.NONE) {
      return
    }

    const rect = svgRef.current.getBoundingClientRect()

    if (!rect.width) {
      return
    }

    movedRef.current = false

    dragRef.current = {
      clientX: event.clientX,
      clientY: event.clientY,
      viewBoxX: safeViewBox.x,
      viewBoxY: safeViewBox.y,
      scaleX: safeViewBox.w / rect.width,
      scaleY: safeViewBox.h / rect.height,
    }

    setIsPanning(true)
  }, [drawMode, safeViewBox])

  const handleMouseMove = useCallback((event) => {
    if (!dragRef.current) {
      return
    }

    const dx = event.clientX - dragRef.current.clientX
    const dy = event.clientY - dragRef.current.clientY

    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
      movedRef.current = true
    }

    const x = dragRef.current.viewBoxX - dx * dragRef.current.scaleX
    const y = dragRef.current.viewBoxY - dy * dragRef.current.scaleY

    if (Number.isFinite(x) && Number.isFinite(y)) {
      setViewBox(prev => ({ ...prev, x, y }))
    }
  }, [])

  const handleMouseUp = useCallback(() => {
    dragRef.current = null
    setIsPanning(false)
  }, [])

  const handleClick = useCallback((event) => {
    if (movedRef.current) {
      movedRef.current = false
      return
    }

    if (drawMode === MODE.NONE) {
      return
    }

    const point = clientToSVG(event, svgRef.current)

    if (!point) {
      return
    }

    onCanvasClick?.(point.x - OX, OY_SVG - point.y)
  }, [drawMode, onCanvasClick])

  const handleContextMenu = useCallback((event) => {
    event.preventDefault()

    if (movedRef.current) {
      movedRef.current = false
      return
    }

    if (drawMode === MODE.NONE) {
      return
    }

    onCanvasRightClick?.()
  }, [drawMode, onCanvasRightClick])

  const gridLines = buildGridLines(safeViewBox, px)
  const gridLabels = buildGridLabels(safeViewBox, px, zoom)

  const metersPerPixel = safeViewBox.w / pixelWidth
  const barMeters = niceMeters(100 * metersPerPixel)
  const barPx = barMeters / metersPerPixel
  const barLabel = barMeters >= 1000 ? `${barMeters / 1000} km` : `${barMeters} m`

  const cursor = isPanning
    ? 'grabbing'
    : drawMode !== MODE.NONE
    ? 'crosshair'
    : 'default'

  return (
    <div ref={wrapRef} style={{ position: 'relative', width: '100%', height: '100%' }}>
      <svg
        ref={svgRef}
        viewBox={`${safeViewBox.x} ${safeViewBox.y} ${safeViewBox.w} ${safeViewBox.h}`}
        preserveAspectRatio="none"
        style={{
          width: '100%',
          height: '100%',
          cursor,
          display: 'block',
          outline: 'none',
        }}
        onClick={handleClick}
        onContextMenu={handleContextMenu}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        <rect
          x={safeViewBox.x}
          y={safeViewBox.y}
          width={safeViewBox.w}
          height={safeViewBox.h}
          fill="#ffffff"
          stroke="none"
        />

        {gridLines}
        {gridLabels}

        {field?.coordinates?.length >= 3 && (
          <polygon
            points={toPoints(field.coordinates)}
            fill="#38BDF8"
            fillOpacity={0.12}
            stroke="#38BDF8"
            strokeWidth={2 * px}
            strokeLinejoin="round"
          />
        )}

        {field?.coordinates?.length >= 2 && (
          <EdgeLabelsSvg
            points={field.coordinates}
            px={px}
            color="#38BDF8"
            closed
            toSvg={mToSVG}
          />
        )}

        {field?.obstacles?.map((obstacle, index) =>
          obstacle.length >= 3 ? (
            <polygon
              key={index}
              points={toPoints(obstacle)}
              fill="#ef4444"
              fillOpacity={0.15}
              stroke="#ef4444"
              strokeWidth={1.5 * px}
              strokeLinejoin="round"
            />
          ) : null
        )}

        {field?.obstacles?.map((obstacle, index) =>
          obstacle.length >= 2 ? (
            <EdgeLabelsSvg
              key={`obstacle-label-${index}`}
              points={obstacle}
              px={px}
              color="#ef4444"
              closed
              toSvg={mToSVG}
            />
          ) : null
        )}

        {ferryRuns.map((run, index) => (
          <polyline
            key={`ferry-${index}`}
            points={run.map(({ x, y }) => mToSVG(x, y).join(',')).join(' ')}
            fill="none"
            stroke={FERRY_COLOR}
            strokeWidth={1.6 * px}
            strokeOpacity={ferryOpacity}
            strokeDasharray={`${6 * px} ${4 * px}`}
          />
        ))}

        {deadheadRuns.map((run, index) => (
          <polyline
            key={`deadhead-${index}`}
            points={run.map(({ x, y }) => mToSVG(x, y).join(',')).join(' ')}
            fill="none"
            stroke="#ef4444"
            strokeWidth={1.6 * px}
            strokeOpacity={deadheadOpacity}
            strokeDasharray={`${3 * px} ${2 * px}`}
          />
        ))}

        {sweepRuns.map((run, index) => (
          <polyline
            key={`sweep-${index}`}
            points={run.map(({ x, y }) => mToSVG(x, y).join(',')).join(' ')}
            fill="none"
            stroke={SWEEP_COLOR}
            strokeWidth={2 * px}
            strokeOpacity={sweepOpacity}
          />
        ))}

        {basePoints.map((point, index) => {
          const [sx, sy] = mToSVG(point.x, point.y)

          return (
            <BaseMarkerSvg
              key={`base-${index}`}
              sx={sx}
              sy={sy}
              r={6 * px}
              strokeWidth={1.5 * px}
            />
          )
        })}

        {basePoint && (() => {
          const [sx, sy] = mToSVG(basePoint[0], basePoint[1])

          return (
            <BaseMarkerSvg
              sx={sx}
              sy={sy}
              r={7 * px}
              strokeWidth={1.8 * px}
            />
          )
        })()}

        {previewPoints?.length >= 3 && (
          <polygon
            points={toPoints(previewPoints)}
            fill={drawColor}
            fillOpacity={0.1}
            stroke="none"
          />
        )}

        {previewLinePoints.length >= 2 && (
          <polyline
            points={previewLinePoints.map(([x, y]) => mToSVG(x, y).join(',')).join(' ')}
            fill="none"
            stroke={drawColor}
            strokeWidth={1.5 * px}
            strokeDasharray={`${6 * px} ${3 * px}`}
          />
        )}

        {previewPoints?.length >= 2 && (
          <EdgeLabelsSvg
            points={previewPoints}
            px={px}
            color={drawColor}
            closed={previewPoints.length >= 3}
            toSvg={mToSVG}
          />
        )}

        {previewPoints?.map(([x, y], index) => {
          const [sx, sy] = mToSVG(x, y)

          return (
            <circle
              key={index}
              cx={sx}
              cy={sy}
              r={5 * px}
              fill="#ffffff"
              stroke={drawColor}
              strokeWidth={2 * px}
            />
          )
        })}

        <VehicleOverlay
          renderer="svg"
          vehicles={vehicles}
          toSvg={mToSVG}
          px={px}
        />

        <circle cx={OX} cy={OY_SVG} r={3 * px} fill="#94a3b8" />
      </svg>

      <ZoomControls
        onZoomIn={() => applyZoom(1.5)}
        onZoomOut={() => applyZoom(1 / 1.5)}
      />

      <div style={{ position: 'absolute', bottom: 10, right: 10 }}>
        <div style={{ ...SCALE_BAR_STYLE, width: barPx }}>
          {barLabel}
        </div>
      </div>
    </div>
  )
}