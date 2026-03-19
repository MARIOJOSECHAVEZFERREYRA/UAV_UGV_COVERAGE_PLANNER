import { useEffect, useRef, useState } from 'react'

/**
 * Opens a WebSocket to /telemetry/{missionId} when the mission is completed.
 * Returns { vehicles, connected }.
 */
export function useTelemetry(missionId, missionStatus) {
  const [vehicles, setVehicles]   = useState([])
  const [connected, setConnected] = useState(false)
  const wsRef                     = useRef(null)

  useEffect(() => {
    if (!missionId || missionStatus !== 'completed') return

    // WebSocket goes directly to the backend (not proxied by Vite)
    const ws = new WebSocket(`ws://localhost:8000/telemetry/${missionId}`)
    wsRef.current = ws

    ws.onopen    = () => setConnected(true)
    ws.onmessage = (e) => {
      const frame = JSON.parse(e.data)
      setVehicles(frame.vehicles)
    }
    ws.onclose   = () => { setConnected(false); setVehicles([]) }
    ws.onerror   = () => setConnected(false)

    return () => ws.close()
  }, [missionId, missionStatus])

  return { vehicles, connected }
}
