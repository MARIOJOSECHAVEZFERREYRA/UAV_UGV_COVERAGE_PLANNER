import { useEffect, useRef, useState, useCallback } from 'react'

/**
 * Opens a WebSocket to /simulation/{missionId} when the mission is completed.
 *
 * Returns:
 *   vehicles      — array of VehicleSimState objects (current frame)
 *   connected     — bool, WebSocket is open
 *   simTimeS      — elapsed simulation seconds
 *   playbackSpeed — current multiplier
 *   setPlayback   — fn(speed: number) sends SimulationConfig to server
 */
export function useSimulation(missionId, missionStatus) {
  const [vehicles, setVehicles] = useState([])
  const [connected, setConnected] = useState(false)
  const [simTimeS, setSimTimeS] = useState(0)
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0)
  const wsRef = useRef(null)
  const localPlaybackRef = useRef(1.0)

  const setPlayback = useCallback((speed) => {
    localPlaybackRef.current = speed
    setPlaybackSpeed(speed)
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ playback_speed: speed, ugv_mode: 'static' }))
    }
  }, [])

  useEffect(() => {
    if (!missionId || missionStatus !== 'completed') return

    const ws = new WebSocket(`ws://localhost:8000/simulation/${missionId}`)
    wsRef.current = ws

    ws.onopen = () => setConnected(true)
    ws.onmessage = (e) => {
      const frame = JSON.parse(e.data)
      setVehicles(frame.vehicles)
      setSimTimeS(frame.sim_time_s)
      // Don't override playback speed from frame — keep local selection
    }
    ws.onclose = () => {
      setConnected(false)
      setVehicles([])
    }
    ws.onerror = () => setConnected(false)

    return () => ws.close()
  }, [missionId, missionStatus])

  return { vehicles, connected, simTimeS, playbackSpeed, setPlayback }
}
