import FloatingPanel from './FloatingPanel.jsx'

const DRAWING_HINT_STYLE = {
  bottom: 24,
  left: '50%',
  transform: 'translateX(-50%)',
  padding: '8px 18px',
  fontSize: 13,
  zIndex: 10,
}

export default function DrawingHint({ mode }) {
  let text = ''

  if (mode === 'draw_polygon') {
    text = 'Click to add points · Right-click to undo · Press "Finish" when done'
  } else if (mode === 'set_base_point') {
    text = 'Click on the map to set the base or recharge point'
  } else if (mode === 'draw_ugv_route') {
    text = 'Click to add UGV waypoints · Right-click to undo · Press "Finish UGV Route" when done (min 2 points)'
  } else {
    text = 'Click to add obstacle points · Right-click to undo · Press "Finish Obstacle" when done'
  }

  return (
    <FloatingPanel tone="accent" style={DRAWING_HINT_STYLE}>
      {text}
    </FloatingPanel>
  )
}
