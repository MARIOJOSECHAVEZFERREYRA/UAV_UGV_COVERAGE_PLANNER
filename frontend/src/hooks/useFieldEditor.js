import { useState, useMemo, useCallback, useEffect } from 'react'
import { lngLatToXy, wouldSelfIntersect } from '../utils/geo.js'
import { MODE } from '../utils/modes.js'

export function useFieldEditor(resetMission) {
  const [mode, setMode] = useState(MODE.NONE)
  const [drawingPoints, setDrawingPoints] = useState([])
  const [drawnPolygon, setDrawnPolygon] = useState(null)
  const [obstacles, setObstacles] = useState([])
  const [basePoint, setBasePoint] = useState(null)
  const [ugvRoute, setUgvRoute] = useState(null)
  const [intersectionWarning, setIntersectionWarning] = useState(false)

  const activeField = useMemo(() => {
    if (drawnPolygon?.length >= 3) {
      return {
        coordinates: drawnPolygon,
        obstacles,
      }
    }
    return null
  }, [drawnPolygon, obstacles])

  useEffect(() => {
    let timeoutId = null

    if (intersectionWarning) {
      timeoutId = setTimeout(() => {
        setIntersectionWarning(false)
      }, 2000)
    }

    return () => {
      if (timeoutId !== null) {
        clearTimeout(timeoutId)
      }
    }
  }, [intersectionWarning])

  const resetEditorState = useCallback(() => {
    setDrawingPoints([])
    setMode(MODE.NONE)
    setIntersectionWarning(false)
  }, [])

  const resetDrawingAndMission = useCallback(() => {
    resetEditorState()
    resetMission()
  }, [resetEditorState, resetMission])

  const addPoint = useCallback((x, y) => {
    if (mode === MODE.NONE) {
      return
    }

    if (mode === MODE.SET_BASE_POINT) {
      setBasePoint([x, y])
      setMode(MODE.NONE)
      return
    }

    // UGV route is an open polyline — no intersection check needed
    if (mode === MODE.DRAW_UGV_ROUTE) {
      setDrawingPoints(points => [...points, [x, y]])
      return
    }

    if (wouldSelfIntersect(drawingPoints, [x, y])) {
      setIntersectionWarning(true)
      return
    }

    setDrawingPoints(points => [...points, [x, y]])
  }, [mode, drawingPoints])

  const undoPoint = useCallback(() => {
    if (mode !== MODE.NONE) {
      setDrawingPoints(points => points.slice(0, -1))
    }
  }, [mode])

  const handleMapClick = useCallback((lng, lat) => {
    const [x, y] = lngLatToXy(lng, lat)
    addPoint(x, y)
  }, [addPoint])

  const handleToggleDrawPolygon = useCallback(() => {
    if (mode === MODE.DRAW_POLYGON) {
      if (drawingPoints.length >= 3) {
        setDrawnPolygon([...drawingPoints])
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      setIntersectionWarning(false)
      return
    }

    setMode(MODE.DRAW_POLYGON)
    setDrawingPoints([])
    setIntersectionWarning(false)
  }, [mode, drawingPoints])

  const handleToggleDrawUgvRoute = useCallback(() => {
    if (mode === MODE.DRAW_UGV_ROUTE) {
      if (drawingPoints.length >= 2) {
        setUgvRoute([...drawingPoints])
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      return
    }
    setMode(MODE.DRAW_UGV_ROUTE)
    setDrawingPoints([])
  }, [mode, drawingPoints])

  const handleToggleSetBasePoint = useCallback(() => {
    setIntersectionWarning(false)
    setDrawingPoints([])
    setMode(currentMode =>
      currentMode === MODE.SET_BASE_POINT ? MODE.NONE : MODE.SET_BASE_POINT
    )
  }, [])

  const handleToggleDrawObstacle = useCallback(() => {
    if (mode === MODE.DRAW_OBSTACLE) {
      if (drawingPoints.length >= 3) {
        setObstacles(currentObstacles => [...currentObstacles, [...drawingPoints]])
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      setIntersectionWarning(false)
      return
    }

    setMode(MODE.DRAW_OBSTACLE)
    setDrawingPoints([])
    setIntersectionWarning(false)
  }, [mode, drawingPoints])

  const handleLoadField = useCallback((fieldData) => {
    setDrawnPolygon(fieldData.boundary)
    setObstacles(fieldData.obstacles ?? [])
    setBasePoint(fieldData.basePoint ?? null)
    resetDrawingAndMission()
  }, [resetDrawingAndMission])

  const handleClear = useCallback(() => {
    setDrawnPolygon(null)
    setObstacles([])
    setBasePoint(null)
    setUgvRoute(null)
    resetDrawingAndMission()
  }, [resetDrawingAndMission])

  return {
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
    handleToggleSetBasePoint,
    handleToggleDrawObstacle,
    handleToggleDrawUgvRoute,
    handleLoadField,
    handleClear,
  }
}