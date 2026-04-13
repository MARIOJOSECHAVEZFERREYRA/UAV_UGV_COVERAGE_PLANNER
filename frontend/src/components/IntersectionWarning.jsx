import FloatingPanel from './FloatingPanel.jsx'

export default function IntersectionWarning({ message }) {
  return (
    <FloatingPanel
      tone="error"
      style={{
        top: 52,
        left: '50%',
        transform: 'translateX(-50%)',
        zIndex: 30,
        pointerEvents: 'none',
        padding: '8px 18px',
        fontSize: 13,
        whiteSpace: 'nowrap',
      }}
    >
      {message ?? 'Invalid shape: polygon edges cannot intersect'}
    </FloatingPanel>
  )
}
