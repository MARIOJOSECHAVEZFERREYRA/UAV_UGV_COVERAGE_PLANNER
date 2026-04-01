function formatSimTime(seconds) {
  const mins = Math.floor(seconds / 60)
  const secs = Math.floor(seconds % 60)
  return `${mins}:${secs.toString().padStart(2, '0')}`
}

export default function SimulationBadge({ connected, simTimeS, playbackSpeed, onPlaybackChange }) {
  const speeds = [1, 2, 5, 10]

  return (
    <div
      style={{
        position: 'absolute',
        bottom: 74,
        left: 16,
        zIndex: 10,
        display: 'flex',
        gap: 12,
        alignItems: 'center',
        background: connected ? '#0d1117cc' : '#37474f88',
        backdropFilter: 'blur(6px)',
        border: `1px solid ${connected ? '#21262d' : '#607D8B'}`,
        borderRadius: 8,
        padding: '8px 14px',
        fontSize: 12,
        color: '#e8eaf6',
      }}
    >
      <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
        <span style={{ color: connected ? '#4CAF50' : '#999' }}>
          {connected ? '●' : '○'}
        </span>
        Simulating · {formatSimTime(simTimeS)} sim
      </span>

      <div style={{ display: 'flex', gap: 4 }}>
        {speeds.map((speed) => (
          <button
            key={speed}
            onClick={() => onPlaybackChange(speed)}
            style={{
              padding: '3px 8px',
              border: `1px solid ${playbackSpeed === speed ? '#4CAF50' : '#555'}`,
              background: playbackSpeed === speed ? 'rgba(76,175,80,0.2)' : 'rgba(0,0,0,0.3)',
              color: playbackSpeed === speed ? '#4CAF50' : '#999',
              borderRadius: 3,
              cursor: 'pointer',
              fontSize: 11,
              fontFamily: 'inherit',
              transition: 'all 0.15s',
            }}
          >
            {speed}x
          </button>
        ))}
      </div>
    </div>
  )
}
