import { useCallback, useRef, useState } from 'react'

const POLL_INTERVAL_MS = 1_500
const TERMINAL_STATUSES = new Set(['completed', 'failed'])

export function useMission() {
  const [mission, setMission]   = useState(null)
  const [loading, setLoading]   = useState(false)
  const [error, setError]       = useState(null)
  const pollTimer               = useRef(null)

  const stopPolling = useCallback(() => {
    if (pollTimer.current) {
      clearInterval(pollTimer.current)
      pollTimer.current = null
    }
  }, [])

  const fetchMission = useCallback(async (id) => {
    const res = await fetch(`/mission/${id}`)
    if (!res.ok) throw new Error(`GET /mission/${id} → ${res.status}`)
    return res.json()
  }, [])

  const startPolling = useCallback((id) => {
    stopPolling()
    pollTimer.current = setInterval(async () => {
      try {
        const data = await fetchMission(id)
        setMission(data)
        if (TERMINAL_STATUSES.has(data.status)) stopPolling()
      } catch (e) {
        setError(e.message)
        stopPolling()
      }
    }, POLL_INTERVAL_MS)
  }, [fetchMission, stopPolling])

  const compute = useCallback(async (payload) => {
    setLoading(true)
    setError(null)
    setMission(null)
    stopPolling()
    try {
      const res = await fetch('/mission/compute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      })
      if (!res.ok) {
        const detail = await res.json().catch(() => ({ detail: res.statusText }))
        throw new Error(detail.detail ?? res.statusText)
      }
      const data = await res.json()
      setMission(data)
      if (!TERMINAL_STATUSES.has(data.status)) startPolling(data.id)
      return data
    } catch (e) {
      setError(e.message)
      return null
    } finally {
      setLoading(false)
    }
  }, [startPolling, stopPolling])

  return { mission, loading, error, compute }
}
