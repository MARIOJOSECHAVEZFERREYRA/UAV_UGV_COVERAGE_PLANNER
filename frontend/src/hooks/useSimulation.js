import { useEffect, useRef, useState, useCallback } from 'react'

/**
 * Opens a WebSocket to /simulation/{missionId} when the mission is completed.
 *
 * Returns:
 *   vehicles      — array of VehicleSimState objects (current frame)
 *   connected     — bool, WebSocket is open
 *   simTimeS      — elapsed simulation seconds
 *   playbackSpeed — current speed multiplier (ignored while paused)
 *   isPaused      — bool
 *   setPlayback   — fn(speed) set speed and resume if paused
 *   pause         — fn() send speed=0 to server
 *   resume        — fn() restore last speed
 *   restart       — fn() reconnect from the beginning
 *   disconnect    — fn() close WS without clearing mission state
 */
export function useSimulation(missionId, missionStatus, enabled) {
  const [vehicles, setVehicles] = useState([])
  const [connected, setConnected] = useState(false)
  const [simTimeS, setSimTimeS] = useState(0)
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0)
  const [isPaused, setIsPaused] = useState(false)
  const [reconnectKey, setReconnectKey] = useState(0)
  const wsRef = useRef(null)
  const localPlaybackRef = useRef(1.0)

  const _send = useCallback((payload) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(payload))
    }
  }, [])

  const setPlayback = useCallback((speed) => {
    localPlaybackRef.current = speed
    setPlaybackSpeed(speed)
    setIsPaused(false)
    _send({ playback_speed: speed })
  }, [_send])

  const pause = useCallback(() => {
    setIsPaused(true)
    _send({ playback_speed: 0 })
  }, [_send])

  const resume = useCallback(() => {
    setIsPaused(false)
    _send({ playback_speed: localPlaybackRef.current })
  }, [_send])

  const restart = useCallback(() => {
    if (wsRef.current) wsRef.current.close()
    setSimTimeS(0)
    setVehicles([])
    setIsPaused(false)
    setReconnectKey(k => k + 1)
  }, [])

  const disconnect = useCallback(() => {
    if (wsRef.current) wsRef.current.close()
    setConnected(false)
    setVehicles([])
    setIsPaused(false)
  }, [])

  useEffect(() => {
    if (!missionId || missionStatus !== 'completed' || !enabled) return

    const ws = new WebSocket(`ws://localhost:8000/simulation/${missionId}`)
    wsRef.current = ws

    ws.onopen = () => setConnected(true)
    ws.onmessage = (e) => {
      const frame = JSON.parse(e.data)
      setVehicles(frame.vehicles)
      setSimTimeS(frame.sim_time_s)
    }
    ws.onclose = () => {
      setConnected(false)
      setVehicles([])
    }
    ws.onerror = () => setConnected(false)

    return () => ws.close()
  }, [missionId, missionStatus, reconnectKey, enabled])

  return {
    vehicles,
    connected,
    simTimeS,
    playbackSpeed,
    isPaused,
    setPlayback,
    pause,
    resume,
    restart,
    disconnect,
  }
}
