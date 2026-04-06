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

  // Closes the simulation badge without clearing waypoints from the map.
  const dismissSimulation = useCallback(() => {
    setActiveMission(null)
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
    dismissSimulation,
    handleMissionReady,
  }
}