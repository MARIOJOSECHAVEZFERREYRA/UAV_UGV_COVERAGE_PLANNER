export default function IntersectionWarning() {
  return (
    <div
      style={{
        position: 'absolute',
        top: 52,
        left: '50%',
        transform: 'translateX(-50%)',
        zIndex: 30,
        pointerEvents: 'none',
        background: '#3d0f0fcc',
        backdropFilter: 'blur(6px)',
        border: '1px solid #f85149',
        borderRadius: 8,
        padding: '8px 18px',
        fontSize: 13,
        color: '#ffa198',
        whiteSpace: 'nowrap',
      }}
    >
      Invalid shape: polygon edges cannot intersect
    </div>
  )
}