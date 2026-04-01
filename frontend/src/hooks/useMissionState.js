import { useState, useCallback } from 'react'

export function useMissionState() {
  const [waypoints, setWaypoints] = useState([])
  const [activeMission, setActiveMission] = useState(null)
  const [highlight, setHighlight] = useState('all')

  const resetMission = useCallback(() => {
    setWaypoints([])
    setActiveMission(null)
    setHighlight('all')
  }, [])

  const handleMissionReady = useCallback((mission, wps) => {
    setWaypoints(wps ?? [])
    setActiveMission(mission)
    setHighlight('all')
  }, [])

  return {
    waypoints,
    activeMission,
    highlight,
    setHighlight,
    resetMission,
    handleMissionReady,
  }
}