const ctrlGroup = {
  background: '#fff',
  borderRadius: 4,
  boxShadow: '0 0 0 2px rgba(0,0,0,.1)',
  overflow: 'hidden',
}
const ctrlBtn = {
  background: 'transparent',
  border: 0,
  boxSizing: 'border-box',
  cursor: 'pointer',
  display: 'block',
  height: 29,
  width: 29,
  outline: 'none',
  padding: 0,
  fontSize: 18,
  lineHeight: 1,
  color: '#333',
  fontFamily: 'inherit',
}

export default function ZoomControls({ onZoomIn, onZoomOut }) {
  return (
    <div style={{ position: 'absolute', top: 10, right: 10, ...ctrlGroup }}>
      <button style={ctrlBtn} onClick={onZoomIn} title="Zoom in">+</button>
      <button style={{ ...ctrlBtn, borderTop: '1px solid #ddd' }} onClick={onZoomOut} title="Zoom out">-</button>
    </div>
  )
}
