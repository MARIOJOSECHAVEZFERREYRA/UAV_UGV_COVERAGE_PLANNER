export default function TelemetryBadge({ connected }) {
  return (
    <div
      style={{
        position: 'absolute',
        bottom: 74,
        left: 16,
        zIndex: 10,
        background: connected ? '#1b5e2088' : '#37474f88',
        backdropFilter: 'blur(4px)',
        border: `1px solid ${connected ? '#4CAF50' : '#607D8B'}`,
        borderRadius: 8,
        padding: '6px 14px',
        fontSize: 12,
        color: '#e8eaf6',
      }}
    >
      {connected ? '● Live · UAV & UGV streaming' : '○ Telemetry disconnected'}
    </div>
  )
}